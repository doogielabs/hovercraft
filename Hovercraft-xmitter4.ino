// This program is the remote control for the joystick end. This calibrates, the axis, then reads the values and
// and sends them over NRF24L01 to be received by the controller.
#include <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"
#include <EEPROM.h>
#include <string.h>
const int yAxisinput = A0;  // Analog input pin that the potentiometer is attached to
const int xAxisinput = A1;
const int throttleAxisinput = A2;
RF24 radio(9,53);

byte addresses[][6] = {"1Node","2Node"};
const char* screenbuffer[25][80];
byte pipes[2] = { 0xAB, 0xCD };              // Radio pipe addresses for the 2 nodes to communicate.
int yAxis = 0;        // value read from the pot
int xAxis = 0;
int throttleAxis = 0;
// Minimum and maximum values for each axis
int xMin = 1024; // Also EEPROM Address 0
int xMax = 0; // Also EEPROM Address 2
int yMin = 1024; // Also EEPROM Address 4
int yMax = 0; // Also EEPROM Address 6
int throttleMin = 1024; // Also EEPROM Address 8
int throttleMax = 0; // Also EEPROM Address 10
// Calculated ranges of each axis
int xRange=0; // Also EEPROM Address 12
int yRange=0; // Also EEPROM Address 14
int throttleRange=0; // Also EEPROM Address 16
// Resting positions of each axis
int xRestpos=0; // Also EEPROM Address 18
int yRestpos=0; // Also EEPROM Address 20
int throttleRestpos=0; // Also EEPROM Address 22
// Axis Increment Values compared to servos, this value is the the range divided by 180
int xAxisincrement=0; // Also EEPROM Address 24
int yAxisincrement=0; // Also EEPROM Address 26
int throttleAxisincrement=0; // Also EEPROM Address 28
int nullzone=100; // Also EEPROM Address 30 //nullzone 40
int thrustmaxL=160;
int thrustmaxR=160; //Was 110
int thrustminL=40;
int thrustminR=40;
int thrustneutralL=92; //Neutral position on the ESC for the thrust motors
int thrustneutralR=92;
int fwdrevL=thrustneutralL; //Fwd / Reverse thrust value
int fwdrevR=thrustneutralR;
byte radiodata[10];
byte telemetrydata[10];
float batteryvoltage=15;
float amperage=0;
float maxamps=0; //Maximum and minimum values to be recording during a run.
float maxvolts=0;
float minvolts=99;
//Variables to do averaging
float amps1=0;
float amps2=0;
float amps3=0;
float volts1=15; // These values are 15.5 for a 4S lipo. This is so we don't skew the
float volts2=15; // Minimum and maximums during a run
float volts3=15;
float volts4=15;
float volts5=15;
float volts6=15;
float volts7=15;
float volts8=15;
bool hasbeenoutofrange = false;
bool outofrangemsgshown = false;
void setup() {
  // initialize serial communications at 115200 bps:
  Serial.begin(115200);
  readcal();// Read in all values in EEPROM into Variables
  radio.begin();
  radio.setPALevel(RF24_PA_HIGH); //Maximum transceiver power, 100mw
  radio.setAutoAck(1); //Auto ack
  radio.enableAckPayload(); //Setup the capability to receive data as the ackoknowledge
  radio.setRetries(0,15); //Smallest time between retries, maximum number of retries
  radio.setPayloadSize(10); //10 byte data packet goes over the radio
  radio.openWritingPipe(pipes[0]); //Open radio pipes, we are the transmitter
  radio.openReadingPipe(1,pipes[1]);
  radio.enableDynamicPayloads(); //Probably don't need
  radio.startListening(); //Time to start listening for a potential transmission
  menubegin(); //Jump to the menu, which is displayed on the Serial Console
}

void loop() { //Main loop of the program
  int serialinput;
  serialinput = waitkeypress(); // Wait for input from the serial terminal
  if(serialinput == 'C' || serialinput == 'c')calibratejoystick();
  if(serialinput == 'V' || serialinput == 'v')viewrawinput();
  if(serialinput == 'R' || serialinput == 'r')printcal();
  if(serialinput == 'X' || serialinput == 'x')xmit();

}

void calibratejoystick(){ //This function will take the joystick POT values and store them
  xMin = 1024;            // inside NVRAM to be used as calibration values
  xMax = 0;               // Pre-set the minimum and maximum values for each axis
  yMin = 1024;
  yMax = 0;
  throttleMin = 1024;
  throttleMax = 0;
  int input;
  delay(20);
  Serial.print("Set all Axis to home position and then press C to continue or any other key to exit.. \r \n");
  delay(5000); //Allow 5 seconds for things to settle
  input = waitkeypress();
  if(input=='c' || input=='C'){
    yAxis = analogRead(yAxisinput); //Read Axis values at their resting positions and store these values
    xAxis = analogRead(xAxisinput);
    throttleAxis = analogRead(throttleAxisinput);
    yRestpos = yAxis;
    xRestpos = xAxis;
    throttleRestpos = throttleAxis;
  }
  else{
     menubegin();
  }
  Serial.print("Move all axis throughout their full ranges press d key once done..  \r \n");
  int i=1;
  while(i==1){
    yAxis = analogRead(yAxisinput);
    xAxis = analogRead(xAxisinput);
    throttleAxis = analogRead(throttleAxisinput);
    if(yAxis > yMax)yMax=yAxis;
    if(yAxis < yMin)yMin=yAxis;
    if(xAxis > xMax)xMax=xAxis;
    if(xAxis < xMin)xMin=xAxis;
    if(throttleAxis > throttleMax)throttleMax=throttleAxis;
    if(throttleAxis < throttleMin)throttleMin=throttleAxis;
    xRange = xMax - xMin;
    yRange = yMax - yMin;
    throttleRange = throttleMax - throttleMin;
  // wait 2 milliseconds before the next loop
  // for the analog-to-digital converter to settle
  // after the last reading:
  delay(2);
    input=' ';
    input = Serial.read();
    if(input=='d' || input=='D')i=0;
  }
  printcal(); //Show our calibrated values on the screen
  yAxisincrement = yRange/180;
  xAxisincrement = xRange/180;
  throttleAxisincrement = throttleRange/180;
  writecal();
  menubegin();
}

int waitkeypress (void){ //This function waits for a keypress on the serial user interface
  int keypress;
  while((keypress=Serial.read())==-1);
  return keypress;
}

void viewrawinput(){ // View raw joystick analog values on serial terminal, used for debugging.
  int x=1;
  char input;
  Serial.print("Press E to Exit this test mode \r \n");
  while(x==1){
    yAxis = analogRead(yAxisinput);
    xAxis = analogRead(xAxisinput);
    throttleAxis = analogRead(throttleAxisinput);
    Serial.print("yAxis = ");
    Serial.print(yAxis);
    Serial.print("\t");
    Serial.print("xAxis = ");
    Serial.print(xAxis);
    Serial.print("\t");
    Serial.print("throttleAxis = ");
    Serial.print(throttleAxis);
    Serial.print("\r \n");
    delay(1000);
    input = Serial.read();
    if(input=='e' || input=='E')x=0;
  }
  menubegin();
}

void menubegin(){
  Serial.println("Press a key to activate function.. (C)alibrate (V)iewstickpos (R)ead Calibration (X)mit mode");
  loop();  
}

void writeprom(int16_t data, int address){ //Write data to eeprom at specified address
  EEPROM.write(address, data&0xFF);
  EEPROM.write(address+1, data>>8);
}

int16_t readprom(int address){ //Read data from eeprom at specified address
  int16_t data;
  data=EEPROM.read(address);
  data|=(int)EEPROM.read(address+1)<<8;
  return data;
}

void writecal(){
// Minimum and maximum values for each axis
  writeprom(xMin,0);
  writeprom(xMax,2);
  writeprom(yMin,4);
  writeprom(yMax,6);
  writeprom(throttleMin,8); 
  writeprom(throttleMax,10); 
// Calculated ranges of each axis
  writeprom(xRange,12);
  writeprom(yRange,14); 
  writeprom(throttleRange,16);
// Resting positions of each axis
  writeprom(xRestpos,18); 
  writeprom(yRestpos,20); 
  writeprom(throttleRestpos,22); 
//Axis Increment Values compared to servos, this value is the the range divided by 180
  writeprom(xAxisincrement,24);
  writeprom(yAxisincrement,26); 
  writeprom(throttleAxisincrement,28); 
}

void readcal(){
  xMin=readprom(0);
  xMax=readprom(2);
  yMin=readprom(4);
  yMax=readprom(6);
  throttleMin=readprom(8);
  throttleMax=readprom(10);
  xRange=readprom(12);
  yRange=readprom(14);
  throttleRange=readprom(16);
  xRestpos=readprom(18);
  yRestpos=readprom(20);
  throttleRestpos=readprom(22);
  xAxisincrement=readprom(24);
  yAxisincrement=readprom(26);
  throttleAxisincrement=readprom(28);
}

void printcal(){
  Serial.println ("xMin, xMax, yMin, yMax, throttleMin, throttleMax:");
  Serial.print (xMin);
  Serial.print (", ");
  Serial.print (xMax);
  Serial.print (", ");
  Serial.print (yMin);
  Serial.print (", ");
  Serial.print (yMax);
  Serial.print (", ");
  Serial.print (throttleMin);
  Serial.print (", ");
  Serial.println (throttleMax);
  Serial.println ("xRange, yRange, throttleRange, xRestpos, yRestpos, throttleRestpos");
  Serial.print (xRange);
  Serial.print (", ");
  Serial.print (yRange);
  Serial.print (", ");
  Serial.print (throttleRange);
  Serial.print (", ");
  Serial.print (xRestpos);
  Serial.print (", ");
  Serial.print (yRestpos);
  Serial.print (", ");
  Serial.println (throttleRestpos);
  Serial.println ("xAxisincrement, yAxisincrement, throttleAxisincrement");
  Serial.print (xAxisincrement);
  Serial.print (", ");
  Serial.print (yAxisincrement);
  Serial.print (", ");
  Serial.println (throttleAxisincrement);
  menubegin();
}

void xmit(){
  char* result = (char*)malloc(5);
  int Skirt=0; //Skirt Thrust
  int LeftThruster=0;
  int RightThruster=0;
  int FwdThrust=0;
  int FwdThrustL=0;
  int FwdThrustR=0;
  volts1 = 15; // Reset the counters for the beginning of a run
  volts2 = 15;
  volts3 = 15;
  volts4 = 15;
  volts5 = 15;
  volts6 = 15;
  volts7 = 15;
  volts8 = 15;
  minvolts = 99;
  maxvolts = 0;
  batteryvoltage = 15;
  int x=1;
  byte pipeNo;
  char input;
  while(x==1){
    input = Serial.read();
    if(input=='e' || input=='E')x=0;
    Serial.println("Out of Range");
    delay(500);
    while(radio.available(&pipeNo)){
    radiodata[0]=0x77; //Blank the data with a predetermined value, lucky #7's
    radiodata[4]=0x77;
    radiodata[5]=0x77;
    radiodata[6]=0x77;
    radiodata[7]=0x77;
    radiodata[8]=0x77;
    radiodata[9]=0x77;
    telemetrydata[0]=0x77;
    telemetrydata[1]=0x77;
    telemetrydata[2]=0x77;
    telemetrydata[3]=0x77;
    telemetrydata[4]=0x77;
    telemetrydata[5]=0x77;
    telemetrydata[6]=0x77;
    telemetrydata[7]=0x77;
    telemetrydata[8]=0x77;
    telemetrydata[9]=0x77;
    yAxis = analogRead(yAxisinput);
    xAxis = analogRead(xAxisinput);
    throttleAxis = analogRead(throttleAxisinput);
    LeftThruster=thrustneutralL;
    RightThruster=thrustneutralR;
    FwdThrust=0;
    Skirt = map(throttleAxis, throttleRestpos, throttleMin, 0, 180); //The correct way to scale the throttle input

    // The following calculations take the joystick input and produce a value between 0 and 180, 
    // referencing 0 to 180 degrees on the servo. about 90* is considered neutral on the thrusters.
    // 
   
    // Take the analog stick input for fwd / reverse and determine thrust
    if(yAxis < (yRestpos-nullzone)){
      fwdrevR = map(yAxis,(yRestpos-nullzone),yMin,thrustneutralR,thrustmaxR);
      fwdrevL = map(yAxis,(yRestpos-nullzone),yMin,thrustneutralL,thrustmaxL);
    }
    else if(yAxis > (yRestpos+nullzone)){
      fwdrevR = map(yAxis,(yRestpos+nullzone),yMax,thrustneutralR,thrustminR);
      fwdrevL = map(yAxis,(yRestpos+nullzone),yMax,thrustneutralL,thrustminL);
    }
    else{
      fwdrevL = thrustneutralL;
      fwdrevR = thrustneutralR;
    }
     //Take the given analog stick input for left and right and increase thrust
    if(xAxis < (xRestpos-nullzone)){
      RightThruster=map(xAxis,(xRestpos-nullzone),xMin,thrustneutralR,thrustmaxR);
    }
    else if(xAxis > (xRestpos+nullzone)){ 
      LeftThruster=map(xAxis,(xRestpos+nullzone),xMax,thrustneutralL,thrustmaxL);
    }
    else { // Neutral position
      LeftThruster = thrustneutralL;
      RightThruster = thrustneutralR;
    }

    // Blend steering input with forward / reverse demand
    // Apply lower value / reverse to opposite motor for more aggressive steering response
    int blendR = (LeftThruster - thrustneutralL);
    int blendL = (RightThruster - thrustneutralR);
    // Average the forward stick with right / left stick
    LeftThruster = ((LeftThruster-blendL)+fwdrevL)/2;
    RightThruster = ((RightThruster-blendR)+fwdrevR)/2;

    if(LeftThruster < thrustminL){ //Enforce ultimate Min limits on thruster demands
      LeftThruster=thrustminL;
    }
    if(RightThruster < thrustminR){
      RightThruster=thrustminR;
    }
    if(LeftThruster > thrustmaxL){ //Enforce ultimate Max limits on thruster demands
      LeftThruster=thrustmaxL;
    }
    if(RightThruster > thrustmaxR){
      RightThruster=thrustmaxR;
    }
    
    if(RightThruster <= 0)RightThruster=0; //Never becomes a negative number
    if(LeftThruster <= 0)LeftThruster=0;
    if(Skirt > 180){ //Absolute max on skirt
      Skirt=180;
    }
    Serial.print("Left: ");
    sprintf(result, "%04d", LeftThruster);
    Serial.print(result);
    //Serial.print(LeftThruster);
    Serial.print("\t");
    Serial.print("Right: ");
    sprintf(result, "%04d", RightThruster);
    Serial.print(result);
    //Serial.print(RightThruster);
    Serial.print("\t");
    Serial.print("Skirt: ");
    sprintf(result, "%04d", Skirt);
    Serial.print(result);
    radiodata[1]=LeftThruster;
    radiodata[2]=RightThruster;
    radiodata[3]=Skirt;
    delay(10);
      radio.read( &telemetrydata, 10);
      radio.writeAckPayload(pipeNo,&radiodata, 4);
      amps1 = amps2;
      amps2 = amps3;
      amps3 = bytes2Float(telemetrydata, 1);
      amperage = ((amps1 + amps2 + amps3) / 3);
      volts1 = volts2;
      volts2 = volts3;
      volts3 = volts4;
      volts4 = volts5;
      volts5 = volts6;
      volts6 = volts7;
      volts8 = volts8;
      volts8 = bytes2Float(telemetrydata, 0);
      batteryvoltage = ((volts1 + volts2 + volts3 + volts4 + volts5 + volts6 + volts7 + volts8) / 8);
      if(batteryvoltage > maxvolts)maxvolts = batteryvoltage;
      if(batteryvoltage < minvolts)minvolts = batteryvoltage;
      if(amperage > maxamps)maxamps = amperage;
     Serial.print(" Batt: ");
     //sprintf(result, "%04d", batteryvoltage);
     //Serial.print(result);
     Serial.print(batteryvoltage);
     Serial.print("V | ");
     //sprintf(result, "%04d", amperage);
     //Serial.print(result);
     Serial.print(amperage);
     Serial.print("A.\r");
     hasbeenoutofrange = (telemetrydata[9] >> 7) & 1;
     if(outofrangemsgshown == false){
      if(hasbeenoutofrange)Serial.println("Poor quality at this distance detected!\n");
      outofrangemsgshown=true;
     }
     delay(10);
    }
    
  } // end of x=1 loop
  Serial.println("End of run detected..");
  Serial.print("BAT VMAX: ");
  Serial.print(maxvolts);
  Serial.print(" VMIN: ");
  Serial.print(minvolts);
  Serial.print(" AmpsMAX: ");
  Serial.println(maxamps);
  maxvolts=0; //Clear values for next run
  minvolts=99;
  maxamps=0;
  menubegin();
}

void flushbuffer(){
  int i=0;
  int y=0;
  while(i < 26){ // Dump buffer to serial terminal
    y=0;
    while(y<81){
        Serial.print(screenbuffer[i][y]);
        y++;  
    }
    Serial.print('\n');
    i++;
  }
  //Clear buffer for next data
  i=0;
  y=0;
  while(i < 26){
    y=0;
    while(y < 81 ){
      screenbuffer[i][y] = " ";
      y++ ;
    }
    i++;
  }

}

float bytes2Float(byte inputarray[], int startlocation){
  float outputfloat=0;
  memcpy(&outputfloat, &inputarray[sizeof(float)*startlocation], sizeof(float));
  return outputfloat;
}

