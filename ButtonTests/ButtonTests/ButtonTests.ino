/*
    _   _    _____  __ __      _____   ___  ___  _    _____   __
   /_\ | |  | __\ \/ / \ \    / / _ \ / _ \|   \| |  | __\ \ / /
  / _ \| |__| _| >  <   \ \/\/ / (_) | (_) | |) | |__| _| \ V / 
 /_/ \_\____|___/_/\_\   \_/\_/ \___/ \___/|___/|____|___| |_|  
                                                                
*/

// CONSTANTS //
#define FACEBUTTON 4
#define MIDBUTTON 2

// GLOBALS //



void setup()
{
  pinMode(FACEBUTTON, INPUT);
  pinMode(MIDBUTTON, INPUT);
  Serial.begin(115200);
}

void loop()
{
  Serial.print(analogRead(FACEBUTTON));
  Serial.print(", ");
  Serial.println(analogRead(MIDBUTTON));
  delay(100);

}
