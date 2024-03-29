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

#define ARMHIGH 12
#define ARMLOW 14
#define TXPIN 17
#define RXPIN 16

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
#define SEVENLENGTH 700

#define eight 11
#define EIGHTLENGTH 700

#define nine 12
#define NINELENGTH 900

#define ten 13
#define TENLENGTH 700

#define eleven 14
#define ELEVENLENGTH 1000

#define twelve 15
#define TWELVELENGTH 900

#define thirteen 16
#define THIRTEENLENGTH 800

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
#define TWENTYLENGTH 900

#define thirty 24
#define THIRTYLENGTH 1000

#define fourty 25
#define FOURTYLENGTH 700

#define fifty 26
#define FIFTYLENGTH 1000

#define OCLOCK 27
#define OCLOCKLENGTH 1000

#define TIMEIS 28
#define TIMEISLENGTH 1400

#define NOTIME 29
#define NOTIMELENGTH 3000

#define STOPLAUGH 30
#define STOPLAUGHLENGTH 1500

#define ELMOSPICY 31
#define ELMOSPICYLENGTH 1500

#define TIMEINVALIDTRIGGERVALUE 3600000

#define CHIMETIMERDELAY 600000

const unsigned long DelayArray [33] = {0, SOMETHINGTOSAYLENGTH, LAUGHTERLENGTH, LOVEYOULENGTH, ONELENGTH, TWOLENGTH, THREELENGTH, FOURLENGTH, FIVELENGTH, SIXLENGTH, SEVENLENGTH, EIGHTLENGTH, NINELENGTH, TENLENGTH, ELEVENLENGTH,
                                       TWELVELENGTH, THIRTEENLENGTH, THIRTEENLENGTH, FOURTEENLENGTH, FIFTEENLENGTH, SIXTEENLENGTH, SEVENTEENLENGTH, EIGHTTEENLENGTH, NINETEENLENGTH, TWENTYLENGTH, THIRTYLENGTH, FOURTYLENGTH,
                                       FIFTYLENGTH, OCLOCKLENGTH, TIMEISLENGTH, NOTIMELENGTH, STOPLAUGHLENGTH, ELMOSPICYLENGTH
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

bool TimeUpdated = false;

bool ValidTime = false;
unsigned long TimeInvalid = millis() + TIMEINVALIDTRIGGERVALUE;
unsigned long ChimeTimer = 0;
bool TimeInvalidTrigger = false;


TinyGPSPlus gps;
SoftwareSerial GPSSerial(RXPIN, TXPIN);
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
    case TIMEIS:
      {
        if (pos >= TIMEISOffsets[2])
        {
          pos = 0;
        }
        return (unsigned char)((int)TIMEISSamples[pos++] + 128);
      }
      break;
    case NOTIME:
      {
        if (pos >= NOTIMEOffsets[2])
        {
          pos = 0;
        }
        return (unsigned char)((int)NOTIMESamples[pos++] + 128);
      }
      break;
    case STOPLAUGH:
      {
        if (pos >= STOPLAUGHOffsets[2])
        {
          pos = 0;
        }
        return (unsigned char)((int)STOPLAUGHSamples[pos++] + 128);
      }
      break;
      case ELMOSPICY:
      {
        if (pos >= ELMOSPICYOffsets[2])
        {
          pos = 0;
        }
        return (unsigned char)((int)ELMOSPICYSamples[pos++] + 128);
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
    case TIMEIS:
      {
        if (pos >= TIMEISOffsets[1])
        {
          pos = 0;
        }
        return (unsigned char)((int)TIMEISSamples[pos++] + 128);
      }
      break;
    case NOTIME:
      {
        if (pos >= NOTIMEOffsets[1])
        {
          pos = 0;
        }
        return (unsigned char)((int)NOTIMESamples[pos++] + 128);
      }
      break;
    case STOPLAUGH:
      {
        if (pos >= STOPLAUGHOffsets[1])
        {
          pos = 0;
        }
        return (unsigned char)((int)STOPLAUGHSamples[pos++] + 128);
      }
      break;
      case ELMOSPICY:
      {
        if (pos >= ELMOSPICYOffsets[1])
        {
          pos = 0;
        }
        return (unsigned char)((int)ELMOSPICYSamples[pos++] + 128);
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
    //Serial.println(DelayArray[SoundSelection]);
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
  pinMode(ARMHIGH, OUTPUT);
  pinMode(ARMLOW, OUTPUT);

  digitalWrite(ARMHIGH, LOW);
  digitalWrite(ARMLOW, LOW);

  Serial.begin(115200);
  Serial.println(totalSampleWords);
  Serial.print("Total stereo samples :");
  Serial.print("Buffer length: ");
  Serial.println((float)totalSampleWords / samplingRate, 3);

  GPSSerial.begin(GPSBAUD); //9600

}



//PlayTrigger = false

void loop()
{
  //
  yield();
  UpdateTime();

  //Serial.println(ValidTime);
  if (ValidTime == true)
  {
    TimeInvalidTrigger = false;
    if ((((gps.time.minute() == 0) && (gps.time.second() > 2))  || (Serial.available() > 0)) && (millis() > ChimeTimer))
    {
      byte HourValue = gps.time.hour();
      digitalWrite(ARMHIGH, HIGH);
      HourChime(HourValue);
      ChimeTimer = millis() + CHIMETIMERDELAY; //delay so we dont get multiple repeats
      digitalWrite(ARMHIGH, LOW);
    }
  }
  else
  {
    if (TimeInvalidTrigger == false)
    {
      Serial.println("Invalid Sound Trigger Set");
      TimeInvalid = millis() + TIMEINVALIDTRIGGERVALUE;
      TimeInvalidTrigger = true;
    }
    else
    {
      //Serial.println(TimeInvalid - millis());
      if (millis() > TimeInvalid)
      {
        PlayTrigger = true;
        PlaySound(NOTIME, true);
        TimeInvalid = millis() + TIMEINVALIDTRIGGERVALUE;
      }
    }
  }
}

void HourChime(byte HourValue) //Little Elmo Chimes on the hour
{
  PlayTrigger = true;
  unsigned long Timer = millis() + 2000;
  while (Timer > millis()) //Time for Elmo to standup
  {
    StandUp(true);
    PlaySound(SOMETHINGTOSAY, false);
  }
  PlaySound(SOMETHINGTOSAY, true);

  PlayTrigger = true;
  Timer = millis() + 2000;

  while (Timer > millis()) //Time for Elmo to standup
  {
    ArmUp(true);
  }

  PlayTrigger = true;
  PlaySound(TIMEIS, true);
  delay(500);

  if (HourValue == 0)
  {
    ReadNumber(12);
  }

  if (HourValue > 12)
  {
    HourValue = HourValue - 12;
  }

  PlayTrigger = true;
  Serial.println(HourValue);
  ReadNumber(HourValue);
  delay(300);
  PlayTrigger = true;
  PlaySound(OCLOCK, true);


  Timer = millis() + 2000;
  while (Timer > millis()) //Time for Elmo to standup
  {
    ArmUp(false);
  }

  Texture();

  Timer = millis() + 2000;
  while (Timer > millis()) //Time for Elmo to standup
  {
    StandUp(false);
  }
}

void Texture()
{
  int RandomVoice = random(1, 10);
  switch (RandomVoice)
  {
    case 1:
      {
        PlayTrigger = true;
        PlaySound(LOVEYOU, true);
      }
      break;
    case 2:
      {
        PlayTrigger = true;
        PlaySound(LAUGHTER, true);
      }
      break;
    default:
      {

      }
      break;
  }
}

void StandUp(bool UpDown)
{

}

void ArmUp(bool UpDown) //arm movement true = up
{

}

void UpdateTime()
{
  while (GPSSerial.available() > 0)
  {
    //Serial.print(char(GPSSerial.read()));
    gps.encode(GPSSerial.read());
  }

  //Serial.println(gps.time.isValid());
  if (gps.time.isValid() == true)
  {
    ValidTime = true;
  }
  else
  {
    ValidTime = false;
  }



  int CurrentSecond = ((millis() / 1000U) % 10); //Gets the seconds in millis
  CurrentSecond = CurrentSecond % 2;
  if (CurrentSecond == 0)
  {
    displayInfo();
  }
  //Serial.println(ValidTime);
}

void ReadNumber(byte Tminute)
{
  //Serial.println(Tminute);
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
          PlaySound(thirteen, true);
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
    if ((Tminute > 19) && (Tminute < 30))
    {
      PlayTrigger = true;
      PlaySound(twenty, true);
      SubNumber = SubNumber - 20;
    }
    else if ((Tminute > 29) && (Tminute < 40))
    {
      PlayTrigger = true;
      PlaySound(thirty, true);
      SubNumber = SubNumber - 30;
    }
    else if ((Tminute > 39) && (Tminute < 50))
    {
      PlayTrigger = true;
      PlaySound(fourty, true);
      SubNumber = SubNumber - 40;
    } else if ((Tminute > 49) && (Tminute < 60))
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
