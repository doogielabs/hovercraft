/*
  // March 2014 - TMRh20 - Updated along with High Speed RF24 Library fork
  // Parts derived from examples by J. Coliz <maniacbug@ymail.com>
*/
/**
 * Example for efficient call-response using ack-payloads 
 *
 * This example continues to make use of all the normal functionality of the radios including
 * the auto-ack and auto-retry features, but allows ack-payloads to be written optionally as well.
 * This allows very fast call-response communication, with the responding radio never having to 
 * switch out of Primary Receiver mode to send back a payload, but having the option to if wanting
 * to initiate communication instead of respond to a commmunication.
 */
 

#include <Servo.h>
#include <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"

// Hardware configuration: Set up nRF24L01 radio on SPI bus plus pins 7 & 8 
RF24 radio(9,53);
// Topology
const uint64_t pipes[2] = { 0xAB, 0xCD };              // Radio pipe addresses for the 2 nodes to communicate.
Servo liftmotor1;
Servo liftmotor2;                                                
Servo thrustLeft;
Servo thrustRight;

int leftInput=92;
int rightInput=92;
int skirtInput=0;
int VRaw;
int IRaw;
float VFinal; //This will store the converted data
float IFinal;
bool hasbeenoutofrange = false;
unsigned long timeoutcounter=0;
void setup(){
  liftmotor1.attach(2);  // attaches the servo on pin 9 to the servo object
  liftmotor2.attach(3); 
  thrustLeft.attach(4);
  thrustRight.attach(5);
  //escrangecal(); // Setup throttle range on ESCs during intitalization.
  Serial.begin(115200);
  Serial.println("Setup..");
  
  // Setup and configure rf radio
  radio.begin();
  radio.setPALevel(RF24_PA_HIGH);
  radio.setAutoAck(1);                    // Ensure autoACK is enabled
  radio.enableAckPayload();               // Allow optional ack payloads
  radio.setRetries(0,15);                 // Smallest time between retries, max no. of retries
  radio.setPayloadSize(10);                // Here we are sending 5-byte payloads to test the call-response speed
  radio.openWritingPipe(pipes[1]);        // Both radios listen on the same pipes by default, and switch when writing
  radio.openReadingPipe(1,pipes[0]);
  radio.enableDynamicPayloads();
  radio.startListening();                 // Start listening
  //radio.printDetails();                   // Dump the configuration of the rf unit for debugging

}

void loop(void) {
    //Serial.println("Run..");
    byte radiodata[4];
    byte telemetrydata[10];
    byte pipeNo;
    radiodata[0]=0x00;
    radiodata[1]=0x00;
    radiodata[2]=0x00;
    radiodata[3]=0x00;    
    radiodata[4]=0x00;
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
//    timeoutcounter++;
//    if(timeoutcounter>15000){
//       Serial.print("Out of range, Skirt thrust held at ");
//       Serial.println(skirtInput);
//       thrustRight.write(0);
//       thrustLeft.write(0);
//     }
    //delay(100);

    //while( radio.available(&pipeNo)){
      delay(10);
      VRaw = analogRead(A0);
      IRaw = analogRead(A1);
      VFinal = VRaw/12.99; //90 Amp board
      //VFinal = VRaw/23.4; //90 Amp board Calibrated to multimeter
      IFinal = IRaw/7.4; //90 Amp board
      float2Bytes(VFinal,&telemetrydata[0]);
      float2Bytes(IFinal,&telemetrydata[4]);
      telemetrydata[9] ^=(-hasbeenoutofrange ^ telemetrydata[9]) & (1 << 7);
      radio.stopListening();                                    // First, stop listening so we can talk.  
      if (!radio.write( &telemetrydata, 10 )){
        timeoutcounter++;
        if(timeoutcounter>200){
          //Serial.print("Out of Range, Skirt Thrust held at ");
          //Serial.println(skirtInput);
          hasbeenoutofrange = true;
          thrustRight.write(0);
          thrustLeft.write(0);
          timeoutcounter=2001;
        }
     }else{  
      unsigned long started_waiting_at = micros();               // Set up a timeout period, get the current microseconds
      boolean timeout = false;                                   // Set up a variable to indicate if a response was received or not
    
      while ( ! radio.available() ){                             // While nothing is received
        if (micros() - started_waiting_at > 200000 ){            // If waited longer than 200ms, indicate timeout and exit while loop
          timeout = true;
          break;
        }      
      }

      if (timeout){
        //Serial.print("Out of Range, Skirt Thrust held at ");
        //Serial.println(skirtInput);
        thrustRight.write(0);
        thrustLeft.write(0);
      }else{
        radio.read(&radiodata, 4);
        timeoutcounter=0;
      }
      skirtInput=radiodata[3];
      rightInput=radiodata[1];
      leftInput=radiodata[2];  
      liftmotor1.write(skirtInput);
      liftmotor2.write(skirtInput);
      thrustRight.write(rightInput);
      thrustLeft.write(leftInput);  
      //Serial.print(VFinal);
      //Serial.println("   Volts");
      //Serial.print(IFinal);
      //Serial.println("   Amps");
      //Serial.print("Left Thruster ");
      //Serial.print(leftInput);
      //Serial.print("\t");
      //Serial.print("Right Thruster ");
      //Serial.print(rightInput);
      //Serial.print("\t");
      //Serial.print("Skirt ");
      //Serial.println(skirtInput);
      timeoutcounter=0;
      radio.startListening();
     }
}

void escrangecal(){ // Setup ESCs for range calibration. 
  int value = 180;
  delay(10);
      liftmotor1.write(value);
      liftmotor2.write(value);
      thrustRight.write(value);
      thrustLeft.write(value);
      delay(2000);
  while(value > 0){
      liftmotor1.write(value);
      liftmotor2.write(value);
      thrustRight.write(value);
      thrustLeft.write(value);
      delay(10);
      value--;
  }
  liftmotor1.write(90);
  liftmotor2.write(90);
  thrustRight.write(90);
  thrustLeft.write(90);
}

void float2Bytes(float val,byte* bytes_array){
  // Create union of shared memory space
  union {
    float float_variable;
    byte temp_array[4];
  } u;
  // Overite bytes of union with float variable
  u.float_variable = val;
  // Assign bytes to input array
  memcpy(bytes_array, u.temp_array, 4);
}
