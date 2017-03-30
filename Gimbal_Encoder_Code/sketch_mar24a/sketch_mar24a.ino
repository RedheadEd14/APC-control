#include <AS5048A.h>

//documentation: https://github.com/ZoetropeLabs/AS5048A-Arduino
AS5048A angleSensor(9);
AS5048A angleSensor2(10);
//AS5048A angleSensor3(11);

void setup()
{
  Serial.begin(57600);
  angleSensor.init();
  angleSensor.setZeroPosition(0x28ED);
  angleSensor.close();// close after each init to allow spi to start again
  
  angleSensor2.init();// i made the clock 10Mhz. it was 1Mhz to star
  angleSensor2.setZeroPosition(0x238);
  angleSensor2.close();// close after each init to allow spi to start again
}
int dt = 0;
long past = 0;

void loop()
{
//  delay(1000);// code runs at about 105us so 10kHz

//  past = micros();
//  word val = angleSensor.getRawRotation();
//  Serial.print("Got rotation of: 0x");
//  Serial.println(val, HEX);
//  Serial.print("Sen_1: ");
  //Serial.println(val);
  double pos = angleSensor.getRotation();
  pos = (pos/16384) * 2 * 3.1415926535;
  //Serial.print("absolute orientation of sensor 1 is : ");
//  Serial.println(pos);
//Serial.print(" : State: ");
//angleSensor.printState();
//Serial.print(" : Errors: ");
//Serial.println(angleSensor.getErrors());
angleSensor.getErrors();

//  word val2 = angleSensor2.getRawRotation();
//  Serial.print("Sen_2: ");
//  Serial.println(val2,HEX);
  double pos2 = angleSensor2.getRotation();
  pos2 = (pos2/16384) * 2 * 3.1415926535;
//  Serial.print("absolute orientation of sensor 2 is : ");
//  Serial.println(pos2);

  Serial.print("[ ");
  Serial.print(pos2,4);
  Serial.print(" ");
  Serial.print(pos,4);
  Serial.println(" ]");
//  Serial.print(" : State: ");
  //angleSensor2.printState();
//  Serial.print(" : Errors: ");
//  Serial.println(angleSensor2.getErrors());
  angleSensor2.getErrors();

//  dt = micros()-past;
 //Serial.print(" |Time:  ");
 // Serial.println(dt);
}
