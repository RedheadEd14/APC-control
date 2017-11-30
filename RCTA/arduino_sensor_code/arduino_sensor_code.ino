/*Maxon Motor driver uses:
  GND - GND
  Digital Pin 8 - Dir
  Digital Pin 9 - PWM

  Relative Encoder uses:
  GND - GND - Green
  Digital Pin 3 - CHB  -Brown
  Digital Pin 2 - CHA  -Yellow
  5v - 5v - Orange

  Encoders Use:
  GND - GND
  5v - 5v
  Digital Pin SCK  - SCK
  Digital Pin MOSI - MOSI
  Digital Pin MISO - MISO
  Digital PIN 10 - Cs Encoder 1
  Digital Pin 11 - Cs Encoder 2
  Digital Pin 12 - Cs Encoder 3

  Matts Motor Use:
  TX - TX
  RX - RX
  GND - GND
*/
#include<EEPROM.h>
#include <Wire.h>
//#include <Encoder.h>
#include <AS5048A.h>
#include <Stream.h>
#include <generic_interface.hpp>
#include <safe_brushless_drive_client.hpp>
#include <multi_turn_angle_control_client.hpp>
//#include <complex_motor_control_client.hpp>
//#include <buffered_voltage_monitor_client.hpp>

//sensor struct definition
AS5048A angleSensor(10);
AS5048A angleSensor2(11);
//Encoder encoder3(2, 3);

// Make a communication interface object
GenericInterface com;
// Make a objects that talk to the module
SafeBruhslessDriveClient drive(0);
//StepDirectionInputClient step_dir(0);
MultiTurnAngleControlClient angle(0);
//ComplexMotorControlClient torque(0);
//BufferedVoltageMonitorClient volts(0);

String inputString;                   // a string to hold incoming data
char charFlag, inChar = '\0';         //code state flag, data input holder

// IQinetics motor global variables
float previous_angle = 0, current_angle = 0, BBcounter = 0, BBcounter_old = 0, state = 0;
// This buffer is for passing around messages. We use one buffer here to save space.
uint8_t communication_buffer[256];
// Stores length of message to send or receive
uint8_t communication_length;
// Time in milliseconds since we received a packet
unsigned long communication_time_last;
  
void setup() {

  Serial.begin (115200);
  
  //encoder init. continuous.
  angleSensor.init();
  angleSensor.setZeroPosition(5380); //0x386F for Haltura arm
  angleSensor.close();// close after each init to allow spi to start again

  angleSensor2.init();// i made the clock 10Mhz. it was 1Mhz to star
  angleSensor2.setZeroPosition(1734);//0x303 for Haltura arm
  angleSensor2.close();// close after each init to allow spi to start again

  //column motor init. timer based pwm.
  pinMode(9, OUTPUT);
  TCCR1A = _BV(COM1A1) | _BV(WGM10);//  _BV(WGM11) | _BV(COM1A0)
  TCCR1B = _BV(CS10); //_BV(WGM12)_BV(WGM13) |
  digitalWrite(8, LOW);
  OCR1A = 0;

  //IQinetics Motor init
  Serial1.begin(115200);
  communication_time_last = millis();

  angle.angle_Kp_.set(com, 50);
  angle.angle_Ki_.set(com, 0);
  angle.angle_Kd_.set(com, 1);
  //  drive.volts_limit_.set(com,8);

  if (com.GetTxBytes(communication_buffer, communication_length)) {
    // Use Arduino serial hardware to send messages
    Serial1.write(communication_buffer, communication_length);
  }
  pinMode(4, INPUT);
  digitalWrite(4, HIGH);
}

void loop() {
  //variable init
  float desired_torque, desired_speed = 0, new_val, old_val, delta3, rad_per_tick; //iqinetics motor command, maxon motor command
  int Read, time_elapsed, old_time;

  rad_per_tick = 2 * 3.14159 / 912;
  while (1) {
    /*When an input from the computer happens, collect the the first letter into a char and all
      subsequent info into a string. the char acts as an identifier for what task the computer wants
      the micro to complete the subsequent string is a value that will be read into the specified function. */
    
    while (Serial.available()) { // get the new byte:
      inChar = Serial.read();
      if (inChar >= 'a' && inChar <= 'z') {
        charFlag = inChar;
      }
      else {
        inputString += inChar; //if not a switch case input.  probably data for said case.
      }
    }
    //    Serial.print(charFlag);

    //When an input from the computer port isn't transmitting, do the last command
    //the computer sent.
    while (!Serial.available()) {
//      time_elapsed = micros() - old_time;
//      old_time = micros();
//      Serial.println(time_elapsed);

      if (desired_speed > 1) {
          Read = digitalRead(4);
          if (Read != state) {
            state = Read;
            BBcounter++;
          }
        } 
      if(desired_speed < -1) {
          Read = digitalRead(4);
          if (Read != state) {
             state = Read;
             BBcounter --;
          }
        }
      
        switch (charFlag) {
          case 'i' :
            get_encoder_read(delta3);  //encoder reading
            //          get_encoder_read();  //encoder reading
            break;
          case 'j' :
            //          get_lidar_read();   //lidar reading
            break;
          case 'k' :             //Maxon Motor command
            desired_speed = inputString.toFloat();
            set_maxon_speed((int)desired_speed);
            break;
          case 'l' :           //IQinetics Motor get sensor data
            break;
          case 'm' :           //IQinetics Motor set motor command data
            desired_torque = inputString.toFloat();
            volts_set_IQinetics_motor(desired_torque);
            break;
          case 'n' :
            brake_IQinetics_motor();
            break;
          case 'o' :
            slack_IQinetics_motor();
            break;
          default:
            break;
        }
        //flag reset
        inputString = "\0";
        charFlag = '\0';
        inChar = '\0';

      }
    }
  }

  void get_encoder_read(float delta3) {
    //void get_encoder_read() {
    int relcount;
    // unless user input.  Only output encoder data.
    //       double pos = angleSensor.getRawRotation();
    //        Serial.print("Sen_1 : ");
    //        Serial.print(val, DEC);
    double pos = angleSensor.getRotation();
    //  if (pos < -13000) {  //fixes an sensor rollover error that causes the output to jump
    //    pos = 16384 + pos;
    //  }
    pos = (pos / 16384) * 2 * 3.1415926535;
    angleSensor.getErrors();
    //
    //        double pos2 = angleSensor2.getRawRotation();
    //        Serial.print("Sen_2: ");
    //        Serial.println(val2,DEC);

    double pos2 = angleSensor2.getRotation();
    pos2 = (pos2 / 16384) * 2 * 3.1415926535;
    angleSensor2.getErrors();

    double pos3 = delta3;
//      double pos3 = angleSensor3.getRawRotation();
//      angleSensor3.getErrors();
    //
    Serial.print("[ ");
    Serial.print(pos,4);
    Serial.print(" ");
    Serial.print(pos2,4);
    Serial.print(" ");
//    Serial.print(pos3);
    relcount = BBcounter - BBcounter_old;
    Serial.print(relcount);
    Serial.println(" ]");
    BBcounter_old = BBcounter;
  }

  void set_maxon_speed(int target_speed) {

    if (target_speed < 0) {
      digitalWrite(8, LOW);
      OCR1A = -1 * target_speed;
      //      Serial.print("<");
      //      Serial.print(target_speed);
      //      Serial.println(">");
    }
    else {
      digitalWrite(8, HIGH);
      OCR1A = target_speed;
      //      Serial.print("<");
      //      Serial.print(target_speed);
      //      Serial.println(">");
    }
  }

  void velocity_set_IQinetics_motor(float desired_torque) {

    angle.ctrl_velocity_.set(com, desired_torque);
    if (com.GetTxBytes(communication_buffer, communication_length)) {
      // Use Arduino serial hardware to send messages
      Serial1.write(communication_buffer, communication_length);
    }
  }

  void volts_set_IQinetics_motor(float desired_torque) {

    //        angle.ctrl_angle_.set(com, desired_torque);
    //        angle.ctrl_coast_.set(com);
    drive.drive_spin_volts_.set(com, desired_torque);
    //      drive.drive_spin_volts_.get(com);
    //
    //  Serial.print("{ ");
    //  Serial.print(desired_torque, 2);
    //  Serial.println(" }");
    if (com.GetTxBytes(communication_buffer, communication_length)) {
      // Use Arduino serial hardware to send messages
      Serial1.write(communication_buffer, communication_length);
    }
  }

  void brake_IQinetics_motor() {

    angle.ctrl_velocity_.set(com, 0.0);

    if (com.GetTxBytes(communication_buffer, communication_length)) {
      // Use Arduino serial hardware to send messages
      Serial1.write(communication_buffer, communication_length);
    }
  }

  void slack_IQinetics_motor() {

    angle.ctrl_coast_.set(com);

    if (com.GetTxBytes(communication_buffer, communication_length)) {
      // Use Arduino serial hardware to send messages
      Serial1.write(communication_buffer, communication_length);
    }
  }

