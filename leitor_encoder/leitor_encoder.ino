#include <Wire.h>
#include <Encoder.h>

#define ENCODER_USE_INTERRUPTS
#define ENCODER_OPTIMIZE_INTERRUPTS

// Initializations
#define encoderPinA 2
#define encoderPinB 3
// #define encoderPinZ 4 // encoder zero alignment position signal
#define zero 4

Encoder myEnc(encoderPinA, encoderPinB);

long ePos = -999; // encoder position
bool LEDstate = false;

union u_tag { // allow long to be read as 4 seperate bytes
   byte b[4]; // 4 bytes to be sent over I2C
   long LePos; // encoder position as 4 byte long
} u;

void setup() {
  Wire.setClock(400000);
  Wire.begin(8);                // join i2c bus with address #8
  Wire.onRequest(requestEvent); // register event
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(zero,INPUT);
}

void loop() {
  if(ePos != myEnc.read()) ePos = myEnc.read(); // read encoder value (4 byte long))
  
//  if(digitalRead(zero)==1){
//    myEnc.write(96);
//  }
 

}


void requestEvent() { // i2c sending function
 // respond with message of 4 bytes as expected by master
 digitalWrite(LED_BUILTIN, LEDstate); // write to LED if i2c is sending
 LEDstate = !LEDstate; // change LED every other time (to avoid delays but still be visible)
 u.LePos = ePos;
 Wire.write(u.b[0]);  // send 4 bytes over I2C
 Wire.write(u.b[1]);  // reconstructed in the same order by master
 Wire.write(u.b[2]);
 Wire.write(u.b[3]);
}
