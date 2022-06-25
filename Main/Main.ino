//Proof of concept sound test usign bitluni's ULP sound alg and a bit of massaging
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <esp32/ulp.h>
#include <driver/rtc_io.h>
#include <driver/dac.h>
#include <soc/rtc.h>
#include <math.h>

#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>

#include "Sounds.h"
#include "numbers.h"
/*
   SOUND SELECTION INDEX
   1 - SOMETHINGTOSAY
   2 - LAUGHTER
   3 - LOVEYOU
   4 - one
   5 - two
   6 - three
   7 - four
   8 - five
   9 - six
   10 - seven
   11 - eight
   12 - nine
   13 - ten





*/

#define TXPIN 3
#define RXPIN 2
#define GPSBAUD 9600

//Sound Key
#define SOMETHINGTOSAY 1
#define SOMETHINGTOSAYLENGTH 2000

#define LAUGHTER 2
#define LAUGHTERLENGTH 1000

#define LOVEYOU 3
#define LOVEYOULENGTH 1500

#define one 4
#define ONELENGTH 1000

#define two 5
#define TWOLENGTH 800

#define three 6
#define THREELENGTH 1200

#define four 7
#define FOURLENGTH 1000

#define five 8
#define FIVELENGTH 1000

#define six 9
#define SIXLENGTH 700

#define seven 10
#define SEVENLENGTH 800

#define eight 11
#define EIGHTLENGTH 800

#define nine 12
#define NINELENGTH 1000

#define ten 13
#define TENLENGTH 800

#define eleven 14
#define ELEVENLENGTH 1000

#define twelve 15
#define TWELVELENGTH 1000

#define thirteen 16
#define THIRTEENLENGTH 1000

#define fourteen 17
#define FOURTEENLENGTH 1000

#define fifteen 18
#define FIFTEENLENGTH 1000

#define sixteen 19
#define SIXTEENLENGTH 1000

#define seventeen 20
#define SEVENTEENLENGTH 1000

#define eightteen 21
#define EIGHTTEENLENGTH 1000

#define nineteen 22
#define NINETEENLENGTH 1000

#define twenty 23
#define TWENTYLENGTH 1000

#define thirty 24
#define THIRTYLENGTH 1000

#define fourty 25
#define FOURTYLENGTH 1000

#define fifty 26
#define FIFTYLENGTH 1000

#define OCLOCK 27
#define OCLOCKLENGTH 1000

const unsigned long DelayArray [29] = {0, SOMETHINGTOSAYLENGTH, LAUGHTERLENGTH, LOVEYOULENGTH, ONELENGTH, TWOLENGTH, THREELENGTH, FOURLENGTH, FIVELENGTH, SIXLENGTH, SEVENLENGTH, EIGHTLENGTH, NINELENGTH, TENLENGTH, ELEVENLENGTH,
                                       TWELVELENGTH, THIRTEENLENGTH, THIRTEENLENGTH, FOURTEENLENGTH, FIFTEENLENGTH, SIXTEENLENGTH, SEVENTEENLENGTH, EIGHTTEENLENGTH, NINETEENLENGTH, TWENTYLENGTH, THIRTYLENGTH, FOURTYLENGTH,
                                       FIFTYLENGTH, OCLOCKLENGTH
                                      };

//Sound Play Control
bool PlayTrigger = false;
bool StartLeft = true;
bool StartRight = true;
unsigned long SoundTrigger = 0;
byte SelectedSound = 0;

const unsigned long samplingRate = 22050;

const int opcodeCount = 17;
const int dacTableStart1 = 2048 - 512;
const int dacTableStart2 = dacTableStart1 - 512;
const int totalSampleWords = 2048 - 512 - 512 - (opcodeCount + 1);
const int totalSamples = totalSampleWords * 2;
const int indexAddress = opcodeCount;
const int bufferStart = indexAddress + 1;
int currentSample = RTC_SLOW_MEM[indexAddress] & 0xffff;

//SoftwareSerial GPSSerial(RXPIN, TXPIN);
TinyGPSPlus gps;
SoftwareSerial GPSSerial(33, 13);
TaskHandle_t TaskHandle_1;

void startULPSound() {

  //calculate the actual ULP clock
  unsigned long rtc_8md256_period = rtc_clk_cal(RTC_CAL_8MD256, 1000);
  unsigned long rtc_fast_freq_hz = 1000000ULL * (1 << RTC_CLK_CAL_FRACT) * 256 / rtc_8md256_period;

  //initialize DACs
  dac_output_enable(DAC_CHANNEL_1);
  dac_output_enable(DAC_CHANNEL_2);
  dac_output_voltage(DAC_CHANNEL_1, 128);
  dac_output_voltage(DAC_CHANNEL_2, 128);

  int retAddress1 = 9;
  int retAddress2 = 13;

  int loopCycles = 100;

  Serial.print("Real RTC clock: ");
  Serial.println(rtc_fast_freq_hz);

  int dt = (rtc_fast_freq_hz / samplingRate) - loopCycles;

  //if(dt < 0)
  const ulp_insn_t stereo[] = {
    //reset offset register
    I_MOVI(R3, 0),
    //delay to get the right sampling rate
    I_DELAY(dt), // 6 + dt
    //reset sample index
    I_MOVI(R0, 0), // 6
    //write the index back to memory for the main cpu
    I_ST(R0, R3, indexAddress), // 8
    //load the samples
    I_LD(R1, R0, bufferStart), // 8
    //mask the lower 8 bits
    I_ANDI(R2, R1, 255), // 6
    //multiply by 2
    I_LSHI(R2, R2, 1), // 6
    //add start position
    I_ADDI(R2, R2, dacTableStart1),// 6
    //jump to the dac opcode
    I_BXR(R2), // 4
    //back from first dac
    //mask the upper 8 bits
    I_ANDI(R1, R1, 0xff00), // 6
    //shift the upper bits to right and multiply by 2
    I_RSHI(R1, R1, 8 - 1), // 6
    //add start position of second dac table
    I_ADDI(R1, R1, dacTableStart2),// 6
    //jump to the dac opcode
    I_BXR(R1), // 4
    //here we get back from writing the second sample
    //increment the sample index
    I_ADDI(R0, R0, 1), // 6
    //if reached end of the buffer, jump relative to index reset
    I_BGE(-13, totalSampleWords), // 4
    //wait to get the right sample rate (2 cycles more to compensate the index reset)
    I_DELAY((unsigned int)dt + 2), // 8 + dt
    //if not, jump absolute to where index is written to memory
    I_BXI(3)
  }; // 4
  // write io and jump back another 12 + 4 + 12 + 4

  size_t load_addr = 0;
  size_t size = sizeof(stereo) / sizeof(ulp_insn_t);
  ulp_process_macros_and_load(load_addr, stereo, &size);
  // this is how to get the opcodes

  //create DAC opcode tables
  for (int i = 0; i < 256; i++)
  {
    RTC_SLOW_MEM[dacTableStart1 + i * 2] = 0x1D4C0121 | (i << 10); //dac1
    RTC_SLOW_MEM[dacTableStart1 + 1 + i * 2] = 0x80000000 + retAddress1 * 4;
    RTC_SLOW_MEM[dacTableStart2 + i * 2] = 0x1D4C0122 | (i << 10); //dac2
    RTC_SLOW_MEM[dacTableStart2 + 1 + i * 2] = 0x80000000 + retAddress2 * 4;
  }

  //set all samples to 128 (silence)
  for (int i = 0; i < totalSampleWords; i++)
    RTC_SLOW_MEM[bufferStart + i] = 0x8080;

  //start
  RTC_SLOW_MEM[indexAddress] = 0;
  ulp_run(0);

  //wait until sure the index of current sample was written
  while (RTC_SLOW_MEM[indexAddress] == 0) vTaskDelay(100 / portTICK_PERIOD_MS);

}

unsigned char nextSampleLeft()
{
  static long pos = 0;
  if (StartLeft == true)
  {
    pos = 0;
    StartLeft = false;
  }

  switch (SelectedSound)
  {
    case SOMETHINGTOSAY:
      {
        if (pos >= SOMETHINGTOSAYOffsets[2])
        {
          pos = 0;
        }
        return (unsigned char)((int)SOMETHINGTOSAYSamples[pos++] + 128);
      }
      break;
    case LAUGHTER:
      {
        if (pos >= LAUGHTEROffsets[2])
        {
          pos = 0;
        }
        return (unsigned char)((int)LAUGHTERSamples[pos++] + 128);
      }
      break;
    case LOVEYOU:
      {
        if (pos >= LOVEYOUOffsets[2])
        {
          pos = 0;
        }
        return (unsigned char)((int)LOVEYOUSamples[pos++] + 128);
      }
      break;
    case one:
      {
        if (pos >= ONEOffsets[2])
        {
          pos = 0;
        }
        return (unsigned char)((int)ONESamples[pos++] + 128);
      }
    case two:
      {
        if (pos >= TWOOffsets[2])
        {
          pos = 0;
        }
        return (unsigned char)((int)TWOSamples[pos++] + 128);
      }
    case three:
      {
        if (pos >= THREEOffsets[2])
        {
          pos = 0;
        }
        return (unsigned char)((int)THREESamples[pos++] + 128);
      }
      break;
    case four:
      {
        if (pos >= FOUROffsets[2])
        {
          pos = 0;
        }
        return (unsigned char)((int)FOURSamples[pos++] + 128);
      }
      break;
    case five:
      {
        if (pos >= FIVEOffsets[2])
        {
          pos = 0;
        }
        return (unsigned char)((int)FIVESamples[pos++] + 128);
      }
      break;
    case six:
      {
        if (pos >= SIXOffsets[2])
        {
          pos = 0;
        }
        return (unsigned char)((int)SIXSamples[pos++] + 128);
      }
      break;
    case seven:
      {
        if (pos >= SEVENOffsets[2])
        {
          pos = 0;
        }
        return (unsigned char)((int)SEVENSamples[pos++] + 128);
      }
      break;
    case eight:
      {
        if (pos >= EIGHTOffsets[2])
        {
          pos = 0;
        }
        return (unsigned char)((int)EIGHTSamples[pos++] + 128);
      }
      break;
    case nine:
      {
        if (pos >= NINEOffsets[2])
        {
          pos = 0;
        }
        return (unsigned char)((int)NINESamples[pos++] + 128);
      }
      break;
    case ten:
      {
        if (pos >= TENOffsets[2])
        {
          pos = 0;
        }
        return (unsigned char)((int)TENSamples[pos++] + 128);
      }
      break;
    case eleven:
      {
        if (pos >= ELEVENOffsets[2])
        {
          pos = 0;
        }
        return (unsigned char)((int)ELEVENSamples[pos++] + 128);
      }
      break;
    case twelve:
      {
        if (pos >= TWELVEOffsets[2])
        {
          pos = 0;
        }
        return (unsigned char)((int)TWELVESamples[pos++] + 128);
      }
      break;
    case thirteen:
      {
        if (pos >= THIRTEENOffsets[2])
        {
          pos = 0;
        }
        return (unsigned char)((int)THIRTEENSamples[pos++] + 128);
      }
      break;
    case fourteen:
      {
        if (pos >= FOURTEENOffsets[2])
        {
          pos = 0;
        }
        return (unsigned char)((int)FOURTEENSamples[pos++] + 128);
      }
      break;
    case fifteen:
      {
        if (pos >= TENOffsets[2])
        {
          pos = 0;
        }
        return (unsigned char)((int)FIFTEENSamples[pos++] + 128);
      }
      break;
    case sixteen:
      {
        if (pos >= SIXTEENOffsets[2])
        {
          pos = 0;
        }
        return (unsigned char)((int)SIXTEENSamples[pos++] + 128);
      }
      break;
    case seventeen:
      {
        if (pos >= SEVENTEENOffsets[2])
        {
          pos = 0;
        }
        return (unsigned char)((int)SEVENTEENSamples[pos++] + 128);
      }
      break;
    case eightteen:
      {
        if (pos >= EIGHTTEENOffsets[2])
        {
          pos = 0;
        }
        return (unsigned char)((int)EIGHTTEENSamples[pos++] + 128);
      }
      break;
    case nineteen:
      {
        if (pos >= NINETEENOffsets[2])
        {
          pos = 0;
        }
        return (unsigned char)((int)NINETEENSamples[pos++] + 128);
      }
      break;
    case twenty:
      {
        if (pos >= TWENTYOffsets[2])
        {
          pos = 0;
        }
        return (unsigned char)((int)TWENTYSamples[pos++] + 128);
      }
      break;
    case thirty:
      {
        if (pos >= THIRTYOffsets[2])
        {
          pos = 0;
        }
        return (unsigned char)((int)THIRTYSamples[pos++] + 128);
      }
      break;
    case fourty:
      {
        if (pos >= FOURTYOffsets[2])
        {
          pos = 0;
        }
        return (unsigned char)((int)FOURTYSamples[pos++] + 128);
      }
      break;
    case fifty:
      {
        if (pos >= FIFTYOffsets[2])
        {
          pos = 0;
        }
        return (unsigned char)((int)FIFTYSamples[pos++] + 128);
      }
      break;
    case OCLOCK:
      {
        if (pos >= OCLOCKOffsets[2])
        {
          pos = 0;
        }
        return (unsigned char)((int)OCLOCKSamples[pos++] + 128);
      }
      break;
  }
}


unsigned char nextSampleRight()
{
  static long pos = 0;
  if (StartRight == true) //If we're at the start of the sound set the array pos to 0
  {
    pos = 0;
    StartRight = false;
  }

  switch (SelectedSound)
  {
    case SOMETHINGTOSAY:
      {
        if (pos >= SOMETHINGTOSAYOffsets[1])
        {
          pos = 0;
        }
        return (unsigned char)((int)SOMETHINGTOSAYSamples[pos++] + 128);
      }
      break;
    case LAUGHTER:
      {
        if (pos >= LAUGHTEROffsets[1])
        {
          pos = 0;
        }
        return (unsigned char)((int)LAUGHTERSamples[pos++] + 128);
      }
      break;
    case LOVEYOU:
      {
        if (pos >= LOVEYOUOffsets[1])
        {
          pos = 0;
        }
        return (unsigned char)((int)LOVEYOUSamples[pos++] + 128);
      }
      break;
    case one:
      {
        if (pos >= ONEOffsets[1])
        {
          pos = 0;
        }
        return (unsigned char)((int)ONESamples[pos++] + 128);
      }
      break;
    case two:
      {
        if (pos >= TWOOffsets[1])
        {
          pos = 0;
        }
        return (unsigned char)((int)TWOSamples[pos++] + 128);
      }
      break;
    case three:
      {
        if (pos >= THREEOffsets[1])
        {
          pos = 0;
        }
        return (unsigned char)((int)THREESamples[pos++] + 128);
      }
      break;
    case four:
      {
        if (pos >= FOUROffsets[1])
        {
          pos = 0;
        }
        return (unsigned char)((int)FOURSamples[pos++] + 128);
      }
      break;
    case five:
      {
        if (pos >= FIVEOffsets[1])
        {
          pos = 0;
        }
        return (unsigned char)((int)FIVESamples[pos++] + 128);
      }
      break;
    case six:
      {
        if (pos >= SIXOffsets[1])
        {
          pos = 0;
        }
        return (unsigned char)((int)SIXSamples[pos++] + 128);
      }
      break;
    case seven:
      {
        if (pos >= SEVENOffsets[1])
        {
          pos = 0;
        }
        return (unsigned char)((int)SEVENSamples[pos++] + 128);
      }
      break;
    case eight:
      {
        if (pos >= EIGHTOffsets[1])
        {
          pos = 0;
        }
        return (unsigned char)((int)EIGHTSamples[pos++] + 128);
      }
      break;
    case nine:
      {
        if (pos >= NINEOffsets[1])
        {
          pos = 0;
        }
        return (unsigned char)((int)NINESamples[pos++] + 128);
      }
      break;
    case ten:
      {
        if (pos >= TENOffsets[1])
        {
          pos = 0;
        }
        return (unsigned char)((int)TENSamples[pos++] + 128);
      }
      break;
    case eleven:
      {
        if (pos >= ELEVENOffsets[1])
        {
          pos = 0;
        }
        return (unsigned char)((int)ELEVENSamples[pos++] + 128);
      }
      break;
    case twelve:
      {
        if (pos >= TWELVEOffsets[1])
        {
          pos = 0;
        }
        return (unsigned char)((int)TWELVESamples[pos++] + 128);
      }
      break;
    case thirteen:
      {
        if (pos >= THIRTEENOffsets[1])
        {
          pos = 0;
        }
        return (unsigned char)((int)THIRTEENSamples[pos++] + 128);
      }
      break;
    case fourteen:
      {
        if (pos >= FOURTEENOffsets[1])
        {
          pos = 0;
        }
        return (unsigned char)((int)FOURTEENSamples[pos++] + 128);
      }
      break;
    case fifteen:
      {
        if (pos >= FIFTEENOffsets[1])
        {
          pos = 0;
        }
        return (unsigned char)((int)FIFTEENSamples[pos++] + 128);
      }
      break;
    case sixteen:
      {
        if (pos >= SIXTEENOffsets[1])
        {
          pos = 0;
        }
        return (unsigned char)((int)SIXTEENSamples[pos++] + 128);
      }
      break;
    case seventeen:
      {
        if (pos >= SEVENTEENOffsets[1])
        {
          pos = 0;
        }
        return (unsigned char)((int)SEVENTEENSamples[pos++] + 128);
      }
      break;
    case eightteen:
      {
        if (pos >= EIGHTTEENOffsets[1])
        {
          pos = 0;
        }
        return (unsigned char)((int)EIGHTTEENSamples[pos++] + 128);
      }
      break;
    case nineteen:
      {
        if (pos >= NINETEENOffsets[1])
        {
          pos = 0;
        }
        return (unsigned char)((int)NINETEENSamples[pos++] + 128);
      }
      break;
    case twenty:
      {
        if (pos >= TWENTYOffsets[1])
        {
          pos = 0;
        }
        return (unsigned char)((int)TWENTYSamples[pos++] + 128);
      }
      break;
    case thirty:
      {
        if (pos >= THIRTYOffsets[1])
        {
          pos = 0;
        }
        return (unsigned char)((int)THIRTYSamples[pos++] + 128);
      }
      break;
    case fourty:
      {
        if (pos >= FOURTYOffsets[1])
        {
          pos = 0;
        }
        return (unsigned char)((int)FOURTYSamples[pos++] + 128);
      }
      break;
    case fifty:
      {
        if (pos >= FIFTYOffsets[1])
        {
          pos = 0;
        }
        return (unsigned char)((int)FIFTYSamples[pos++] + 128);
      }
      break;
    case OCLOCK:
      {
        if (pos >= OCLOCKOffsets[1])
        {
          pos = 0;
        }
        return (unsigned char)((int)OCLOCKSamples[pos++] + 128);
      }
      break;
  }
}

int lastFilledWord = 0;

void sound_task(void *pvParameters)
{
  startULPSound();

  while (1)
  {
    int currentSample = RTC_SLOW_MEM[indexAddress] & 0xffff;
    int currentWord = currentSample >> 1;

    while (lastFilledWord != currentWord)
    {
      unsigned int w = nextSampleLeft();
      w |= nextSampleRight() << 8;
      RTC_SLOW_MEM[bufferStart + lastFilledWord] = w;
      lastFilledWord++;
      if (lastFilledWord == totalSampleWords)
        //break;
        lastFilledWord = 0;
    }

    vTaskDelay(10 / portTICK_PERIOD_MS);
  }

  vTaskDelete(NULL);
}

void PlaySound(byte SoundSelection, bool LoopLogic)
{
  if (PlayTrigger == true) //puts play time on to the clock
  {
    dac_output_enable(DAC_CHANNEL_1);
    dac_output_enable(DAC_CHANNEL_2);
    SelectedSound = SoundSelection;
    Serial.println(DelayArray[SoundSelection]);
    SoundTrigger = millis() + DelayArray[SoundSelection];
    xTaskCreatePinnedToCore(sound_task, "sound task", 1024 * 6, NULL, 2, NULL, 1); //Assign a new task
    //startULPSound(); //Restart the sound routine
    StartLeft = true;
    StartRight = true;
    currentSample = RTC_SLOW_MEM[indexAddress] & 0xffff;
    PlayTrigger = false;
    if (LoopLogic == true)
    {
      while (millis() < SoundTrigger)
      {
        delay(1);
        //Loop until we're done!
      }
      dac_output_disable(DAC_CHANNEL_1);
      dac_output_disable(DAC_CHANNEL_2);
      //vTaskDelete(TaskHandle_1);
    }
  }
  else
  {
    if (millis() > SoundTrigger) //Sound Time Up
    {
      dac_output_disable(DAC_CHANNEL_1);
      dac_output_disable(DAC_CHANNEL_2);
      //vTaskDelete(TaskHandle_1);
    }
  }
}


void setup()
{
  Serial.begin(115200);
  Serial.println(totalSampleWords);
  Serial.print("Total stereo samples :");
  Serial.print("Buffer length: ");
  Serial.println((float)totalSampleWords / samplingRate, 3);

  GPSSerial.begin(GPSBAUD); //9600

  PlayTrigger = true;

}
byte HourTime = 0;
byte MinuteTime = 0;
byte PrevMinute = 0;
bool TimeUpdated = false;
//PlayTrigger = false
void loop()
{
  while (GPSSerial.available() > 0)
  {
    //Serial.print(char(GPSSerial.read()));
    if (gps.encode(GPSSerial.read()))
    {
      displayInfo();
    }
  }
  yield();

  if(gps.time.minute() != PrevMinute)
  {
    TimeUpdated = false;
  }
  
  if (TimeUpdated == false)
  {
    HourTime = gps.time.hour();
    if (HourTime == 0)
    {
      HourTime = 12;
    }
    ReadNumber(HourTime);

    MinuteTime = gps.time.minute();
    ReadNumber(MinuteTime);
    TimeUpdated = true;
    PrevMinute = MinuteTime;
  }
  
}

void ReadNumber(byte Tminute)
{
  if (Tminute < 20)
  {
    switch (Tminute)
    {
      case 0:
        {
          PlaySound(OCLOCK, true);
        }
      case 1:
        {
          PlaySound(one, true);
        }
        break;
      case 2:
        {
          PlaySound(two, true);
        }
        break;
      case 3:
        {
          PlaySound(three, true);
        }
        break;
      case 4:
        {
          PlaySound(four, true);
        }
        break;
      case 5:
        {
          PlaySound(five, true);
        }
        break;
      case 6:
        {
          PlaySound(six, true);
        }
        break;
      case 7:
        {
          PlaySound(seven, true);
        }
        break;
      case 8:
        {
          PlaySound(eight, true);
        }
        break;
      case 9:
        {
          PlaySound(nine, true);
        }
        break;
      case 10:
        {
          PlaySound(ten, true);
        }
        break;
      case 11:
        {
          PlaySound(eleven, true);
        }
        break;
      case 12:
        {
          PlaySound(twelve, true);
        }
        break;
      case 13:
        {
          PlaySound(thirty, true);
        }
        break;
      case 14:
        {
          PlaySound(fourteen, true);
        }
        break;
      case 15:
        {
          PlaySound(fifteen, true);
        }
        break;
      case 16:
        {
          PlaySound(sixteen, true);
        }
        break;
      case 17:
        {
          PlaySound(seventeen, true);
        }
        break;
      case 18:
        {
          PlaySound(eightteen, true);
        }
        break;
      case 19:
        {
          PlaySound(nineteen, true);
        }
        break;
    }
  }
  else
  {
    byte SubNumber = Tminute;
    if ((Tminute > 20) && (Tminute < 30))
    {
      PlayTrigger = true;
      PlaySound(twenty, true);
      SubNumber = SubNumber - 20;
    }
    else if ((Tminute > 30) && (Tminute < 40))
    {
      PlayTrigger = true;
      PlaySound(thirty, true);
      SubNumber = SubNumber - 30;
    }
    else if ((Tminute > 40) && (Tminute < 50))
    {
      PlayTrigger = true;
      PlaySound(fourty, true);
      SubNumber = SubNumber - 40;
    } else if ((gps.time.minute() > 50) && (Tminute < 60))
    {
      PlayTrigger = true;
      PlaySound(fifty, true);
      SubNumber = SubNumber - 50;
    }

    PlayTrigger = true;
    PlaySubNumber(SubNumber);
  }
}

void PlaySubNumber(byte Number)
{
  switch (Number)
  {
    case 1:
      {
        PlaySound(one, true);
      }
      break;
    case 2:
      {
        PlaySound(two, true);
      }
      break;
    case 3:
      {
        PlaySound(three, true);
      }
      break;
    case 4:
      {
        PlaySound(four, true);
      }
      break;
    case 5:
      {
        PlaySound(five, true);
      }
      break;
    case 6:
      {
        PlaySound(six, true);
      }
      break;
    case 7:
      {
        PlaySound(seven, true);
      }
      break;
    case 8:
      {
        PlaySound(eight, true);
      }
      break;
    case 9:
      {
        PlaySound(nine, true);
      }
      break;
    default:
      {

      }
      break;
  }
}

void displayInfo()
{
  Serial.print(F("Location: "));
  if (gps.location.isValid())
  {
    Serial.print(gps.location.lat(), 6);
    Serial.print(F(","));
    Serial.print(gps.location.lng(), 6);
  }
  else
  {
    Serial.print(F("INVALID"));
  }

  Serial.print(F("  Date/Time: "));
  if (gps.date.isValid())
  {
    Serial.print(gps.date.month());
    Serial.print(F("/"));
    Serial.print(gps.date.day());
    Serial.print(F("/"));
    Serial.print(gps.date.year());
  }
  else
  {
    Serial.print(F("INVALID"));
  }

  Serial.print(F(" "));
  if (gps.time.isValid())
  {
    if (gps.time.hour() < 10) Serial.print(F("0"));
    Serial.print(gps.time.hour());
    Serial.print(F(":"));
    if (gps.time.minute() < 10) Serial.print(F("0"));
    Serial.print(gps.time.minute());
    Serial.print(F(":"));
    if (gps.time.second() < 10) Serial.print(F("0"));
    Serial.print(gps.time.second());
    Serial.print(F("."));
    if (gps.time.centisecond() < 10) Serial.print(F("0"));
    Serial.print(gps.time.centisecond());
  }
  else
  {
    Serial.print(F("INVALID"));
  }

  Serial.println();
}
