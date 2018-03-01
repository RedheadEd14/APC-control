/*Column Motor driver uses:
  GND - GND
  Digital Pin 8 - Dir
  Digital Pin 9 - PWM
  Digital Pin 7 - 5v (SLP pin)

  Relative Encoders use:
  GND - GND - Green
  5v- 5v
  Analog Pin 6 (Digital Pin 4)
  Analog Pin 11 (Digital Pin 12)
  5v - 5v - Orange

  Encoders Use:
  GND - GND
  5v - 5v
  Digital Pin SCK  - SCK
  Digital Pin MOSI - MOSI
  Digital Pin MISO - MISO
  Digital PIN 10 - Cs Encoder 1
  Digital Pin 11 - Cs Encoder 2

  Iqinetics Motor Use:
  TX - TX
  RX - RX
  GND - GND
*/
#include <AS5048A.h>
#include <generic_interface.hpp>
#include <safe_brushless_drive_client.hpp>
#include <multi_turn_angle_control_client.hpp>

//sensor struct definition
AS5048A angleSensor(10);
AS5048A angleSensor2(11);

// Make a communication interface object
GenericInterface com;
// Make a objects that talk to the module
SafeBruhslessDriveClient drive(0);
MultiTurnAngleControlClient angle(0);

//reflectance sensor variables


// This buffer is for passing around messages. We use one buffer here to save space.
uint8_t communication_buffer[256];
// Stores length of message to send or receive
uint8_t communication_length;
// Time in milliseconds since we received a packet
unsigned long communication_time_last;

void setup() {

  Serial.begin (57600);

  //encoder init. continuous.
  angleSensor.init();
  angleSensor.setZeroPosition(5380); //0x386F for Haltura arm
  angleSensor.close();// close after each init to allow spi to start again

  angleSensor2.init();// i made the clock 10Mhz. it was 1Mhz to star
  angleSensor2.setZeroPosition(5555);//0x303 for Haltura arm
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
  float desired_torque, desired_speed = 0, delta3, rad_per_tick; //iqinetics motor command, maxon motor command
  int Read, Read2, bitcountUp, bitcountDown; //time_elapsed, old_time, 
  int BBcounter = 0, BBcounter_old = 0, column_val = 0, column_number = 0;
  bool state = 0, prior_state = 0, state2 = 0, prior_state2 = 0, startflagUp = 0, startflagDown;
  String inputString;                   // a string to hold incoming data
  char charFlag, inChar = '\0';         //code state flag, data input holder

  inputString.reserve(10);  //creates a buffer to optimize string use in code.  Shoudn't be > 5 really.
  
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
//        desired_speed = 1;
      if (desired_speed > 0) {
        if (startflagDown == 1){
          bitcountDown = -2;
          startflagDown = 0;
        }
        Read = analogRead(A6);
//        Serial.println(Read);
        if (Read > 600) {
          state = 1;
        }
        if (Read < 300) {
          state = 0;
        }
        if (prior_state != state) {
          BBcounter++;
          prior_state = state;
          if (startflagUp == 1 && state == 1) {
            bitcountUp--;
          }
        }
        
        Read2 = analogRead(A11);
//        Serial.println(Read2);

        if (Read2 < 250 && startflagUp == 0) {
          startflagUp = 1;
          prior_state2 = 1;
          state2 = 1;
          bitcountUp = 5;
        }
        if (bitcountUp == -1 && state == 1) {
          startflagUp = 0;
          column_number = column_val;
          column_val = 0;
          bitcountUp = -2;
        }

        if (Read2 < 250 && startflagUp == 1) {
          state2 = 1;
        }
        if (Read2 > 450 && startflagUp == 1) {
          state2 = 0;
        }

        if (prior_state2 != state2) {
          prior_state2 = state2;
          if (state2 == 1) {
          bitSet(column_val, bitcountUp);
          }
        }

      }
//      Serial.println(Read2);
//      Serial.println(bitcountUp);
//      Serial.println(column_number);
      if (desired_speed < 0) {
        
        if (startflagUp == 1){
          bitcountUp = -2;
          startflagUp = 0;
        }
        Read = analogRead(A6);
        if (Read > 600) {
          state = 1;
        }
        if (Read < 300) {
          state = 0;
        }
        if (prior_state != state) {
          BBcounter--;
          prior_state = state;
          if (startflagDown == 1 && state == 1) {
            bitcountDown++;
          }
        }
        
        Read2 = analogRead(A11);
//        Serial.println(Read2);

        if (Read2 < 250 && startflagDown == 0) {
          startflagDown = 1;
          prior_state2 = 1;
          state2 = 1;
          bitcountDown = -1;
        }
        if (bitcountDown == 5 && state == 1) {
          startflagDown = 0;
          column_number = column_val;
          column_val = 0;
          bitcountDown = 6;
        }

        if (Read2 < 250 && startflagDown == 1) {
          state2 = 1;
        }
        if (Read2 > 450 && startflagDown == 1) {
          state2 = 0;
        }

        if (prior_state2 != state2) {
          prior_state2 = state2;
          if (state2 == 1) {
          bitSet(column_val, bitcountDown);
          }
        }
      
      }

      switch (charFlag) {
        case 'i' :
          get_encoder_read(BBcounter, column_number);  //encoder reading
          column_number = 0;
          break;
        case 'j' :
          //          get_lidar_read();   //lidar reading
          break;
        case 'k' :             //Column Motor command
          desired_speed = inputString.toFloat();
          set_column_speed((int)desired_speed);
          break;
        case 'l' :           //IQinetics Motor get sensor data. Defunct
          break;
        case 'm' :           //IQinetics Motor set motor command data
          desired_torque = inputString.toFloat();
          volts_set_IQinetics_motor(desired_torque);
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

void get_encoder_read(int BBcount, int column_num) {
  // Outputs encoder data.
//         double pos = angleSensor.getRawRotation();  //uncomment this code when calibrating encoders for the first time.
//          Serial.print("Sen_1 : ");
//          Serial.print(pos, DEC);
  double pos = angleSensor.getRotation();
  //  if (pos < -13000) {  //fixes an sensor rollover error that causes the output to jump
  //    pos = 16384 + pos;
  //  }
  pos = (pos / 16384) * 2 * 3.1415926535;
  angleSensor.getErrors();
  //
//          double pos2 = angleSensor2.getRawRotation();
//          Serial.print("Sen_2: ");
//          Serial.println(pos2,DEC);

  double pos2 = angleSensor2.getRotation();
  pos2 = (pos2 / 16384) * 2 * 3.1415926535;
  angleSensor2.getErrors();

  Serial.print("[ ");
  Serial.print(pos, 4);
  Serial.print(" ");
  Serial.print(pos2, 4);
  Serial.print(" ");
  Serial.print(BBcount);
  Serial.print(" ");
  Serial.print(column_num);
  Serial.println(" ]");
  Serial.flush();
}

void set_column_speed(int target_speed) {
  //motor operates via a timer based PWM.  changing OCR1A updates a register that varies the mid point of a pwm signal between 0 and 1.  
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

void volts_set_IQinetics_motor(float desired_torque) {

    drive.drive_spin_volts_.set(com, desired_torque); //sets a voltage for the iQinetics Motor
  
  if (com.GetTxBytes(communication_buffer, communication_length)) {
    // Use Arduino serial hardware to send messages
    Serial1.write(communication_buffer, communication_length);
  }
}

void slack_IQinetics_motor() {

  angle.ctrl_coast_.set(com);  //turns off voltage to the motor allowing it to spin freely

  if (com.GetTxBytes(communication_buffer, communication_length)) {
    // Use Arduino serial hardware to send messages
    Serial1.write(communication_buffer, communication_length);
  }
}

//void brake_IQinetics_motor() {
//
//  angle.ctrl_velocity_.set(com, 0.0); //sets a velocity of zero to the motor which effectively brakes it.
//
//  if (com.GetTxBytes(communication_buffer, communication_length)) {
//    // Use Arduino serial hardware to send messages
//    Serial1.write(communication_buffer, communication_length);
//  }
//}

