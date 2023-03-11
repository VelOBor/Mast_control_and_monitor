/*
Drilling mast control system V0.1 by Oleg "Salty NUggeTZ" Velichko and Igor "Terrano"
Designed to control and monitor a system of undisclosed origin and specs
Hypothetical prototype designed with Fritzing and built on a breadboard
The system is built around an Arduino NANO
*/

//==================include necessary libraries==================
#include <Arduino.h> //include arduino library for use in VSCode PlatformIO (NOT NEEDED WHEN USING ARDUINO IDE!!!)
#include <Wire.h> //include I2C library
#include <INA219_WE.h> //include the INA library by Wolfgang Ewald
#include <TM1637.h> //include the TM1637 library from hard drive, located in .pio folder (download from (https://drive.google.com/file/d/1DN5pDko7D1-F4POXICIfd8m_1sAKBZt7/view)

#define I2C_ADDRESS 0x40 //define I2C address for the INA219 module

//==================pin definitions==================

//inputs
//pressure sensor is connected to the INA219 module which is initialized later on
int axis_1 = A0; //potentiometer joystick for vertical axis 1
int axis_2 = A1; //potentiometer joystick for vertical axis 2
int axis_fingers = A2;  //potentiometer joystick for "fingers" axis

int neutral_switch_1 = 9; //neutral button within the joystick, neutral state is when the button is NOT pressed
int neutral_switch_2 = 8; //neutral button within the joystick, neutral state is when the button is NOT pressed
int neutral_switch_3 = 7; //neutral button within the joystick, neutral state is when the button is NOT pressed

//outputs
int up_state = 2; //status lights up LED when "up" command is received from any of the vertical axes (ON/OFF output)
int down_state = 4; //status lights up LED when "down" command is received from any of the vertical axes (ON/OFF output)

int out_power_1 = 6; //PWM output for upwards power of axis 1
int out_power_2 = 5; //PWM output for upwards power of axis 2
int out_power_fingers = 3; //PWM output for closing power of "fingers" axis

//TM1637 display module
int display_clk = 12; //CLK pin of the TM1637 display module, perhaps rename later
int display_dio = 13; //DIO pin of the TM1637 display module, perhaps rename later

//==================variables==================
float current_mA = 0.0; //float variable for reading current from the INA module

bool axis_1_neutral = true; //flag for axis 1 being in neutral state
bool axis_1_up = true; //flag for axis 1 being in UP state
bool axis_1_down = true; //flag for axis 1 being in DOWN state

bool axis_2_neutral = true; //flag for axis 1 being in neutral state
bool axis_2_up = true; //flag for axis 2 being in UP state
bool axis_2_down = true; //flag for axis 2 being in DOWN state

bool axis_fingers_neutral = true; //flag for axis 1 being in neutral state
bool axis_fingers_close = true; //flag for axis 1 being in UP state
bool axis_fingers_open = true; //flag for axis 1 being in DOWN state

bool lock_1 = false; //flag for locking out axis 1
bool lock_2 = false; //flag for locking out axis 2
bool lock_3 = false; //flag for locking out fingers axis

int neutral_switch_1_state = 0; //neutral button 1 state
int neutral_switch_2_state = 0; //neutral button 2 state
int neutral_switch_3_state = 0; //neutral button 3 state

int pressure_val = 0; //mapped value from the INA219 module from FLOAT to INT
int pressure_actual = 0; //mapped actual pressure in bar (atmospheres)

int axis_1_val = 0; //for storing raw input value from the joystick
int axis_1_out = 0; //for storing output value mapped to appropriate PWM output

int axis_2_val = 0; //for storing raw input value from the joystick
int axis_2_out = 0; //for storing output value mapped to appropriate PWM output

int axis_fingers_val = 0; //for storing raw input value from the joystick
int axis_fingers_out = 0; //for storing output value mapped to appropriate PWM output

//==================create instances of library objects==================
INA219_WE ina219 = INA219_WE(I2C_ADDRESS); //create an instance of the INA I2C module
TM1637 tm1637(display_clk, display_dio); //create an instance of the TM1637 module

void setup() {
Serial.begin(115200); //initialize serial communication
Wire.begin(); //initialize the I2C interface

//===remove next 2 lines after code is completed and checked and double checked
  if(!ina219.init()){ //check if the INA module is online 
    Serial.println("INA219 not connected!");} //return error if the module is not responding
//===remove previous 2 lines after code is completed and checked and double checked

tm1637.init(); //initialize the tm1637 module
  tm1637.set(BRIGHT_TYPICAL); //set brightness level for the display module

//pinmode definitions for inputs
pinMode(axis_1, INPUT); //vertical axis 1
pinMode(neutral_switch_1, INPUT_PULLUP); //vertical axis 1 neutral switch, !!!SWITCH OPEN == NEUTRAL STATE!!!

pinMode(axis_2, INPUT); //vertical axis 2
pinMode(neutral_switch_2, INPUT_PULLUP); //vertical axis 2 neutral switch, !!!SWITCH OPEN == NEUTRAL STATE!!!

pinMode(axis_fingers, INPUT); //fingers axis
pinMode(neutral_switch_3, INPUT_PULLUP); //fingers axis neutral switch, !!!SWITCH OPEN == NEUTRAL STATE!!!

//pinMode(pressure_sensor, INPUT);

//pinmode definitions for outputs
pinMode(up_state, OUTPUT);
pinMode(down_state, OUTPUT);
pinMode(display_clk, OUTPUT);
pinMode(display_dio, OUTPUT);

//reset lock status to FALSE for all three axes, probably get rid of this once buttons for neutral state are implemented
lock_1 = false;
lock_2 = false;
lock_3 = false;

}


//NOTE - center value for joysticks is 512, include hysteresis +-10 (probably get rid of the hysteresis)
//NOTE - pushing UP on the fingers control should CLOSE the fingers, equivalent to UP_POWER

void loop() {

tm1637.clearDisplay(); //clear the display to avoid stuck readings

//==================set flags==================
if (axis_2_neutral == false || axis_fingers_neutral == false){lock_1 = true;} //lock axis if EITHER of two other axes are NOT neutral
if (axis_2_neutral == true && axis_fingers_neutral == true){lock_1 = false;} //unlock if BOTH other axes are neutral

if (axis_1_neutral == false || axis_fingers_neutral == false){lock_2 = true;} //lock axis if EITHER of two other axes are NOT neutral
if (axis_1_neutral == true && axis_fingers_neutral == true){lock_2 = false;} //unlock if BOTH other axes are neutral

if (axis_1_neutral == false || axis_2_neutral == false){lock_3 = true;} //lock axis if EITHER of two other axes are NOT neutral
if (axis_1_neutral == true && axis_2_neutral == true){lock_3 = false;} //unlock if BOTH other axes are neutral

if (axis_1_neutral == true){digitalWrite(out_power_1, LOW);}
if (axis_2_neutral == true){digitalWrite(out_power_2, LOW);}
if (axis_fingers_neutral == true){digitalWrite(out_power_fingers, LOW);}


//==================get raw data from the pressure sensor==================
current_mA = ina219.getCurrent_mA(); //read current from the INA module
current_mA = constrain (current_mA, 0, 30); //limit the value IN SOFTWARE from 0 to 30mA for ease of calculations

//==================convert data from float to int==================
pressure_val = current_mA*100;

//==================convert current from sensor to actual pressure value, !!!MUST BE CALIBRATED!!!==================
pressure_actual = map(pressure_val, 0, 3000, 0, 250);

//==================read values of axis==================
axis_1_val = analogRead(axis_1);
axis_2_val = analogRead(axis_2);
axis_fingers_val = analogRead(axis_fingers);

//==================read neutral switches==================
neutral_switch_1_state = digitalRead(neutral_switch_1);
neutral_switch_2_state = digitalRead(neutral_switch_2);
neutral_switch_3_state = digitalRead(neutral_switch_3);


//==================read pressure sensor==================
//==================LEGACY, OBSOLETE==================
//pressure_val = analogRead(pressure_sensor);
//pressure_actual = map(pressure_val, 0, 1023, 0, 255);

//==================process and map axis 1==================

    if (axis_1_val >= 522){
        axis_1_out = map(axis_1_val, 522, 1023, 0, 255);
        //axis_1_neutral = false;
        axis_1_up = true;
        axis_1_down = false;
        }
        
    if (axis_1_val <= 502){
        axis_1_out = map(axis_1_val, 0, 502, 255, 0);
        //axis_1_neutral = false;
        axis_1_up = false;
        axis_1_down = true;
        }
    if (axis_1_val > 502 && axis_1_val <= 522){/*axis_1_neutral = true, */axis_1_up = false, axis_1_down = false, axis_1_out = 0;
    }    

//==================process and map axis 2==================

    if (axis_2_val >= 522){
        axis_2_out = map(axis_2_val, 522, 1023, 0, 255);
        //axis_2_neutral = false;
        axis_2_up = true;
        axis_2_down = false;
        }
        
    if (axis_2_val <= 502){
        axis_2_out = map(axis_2_val, 0, 502, 255, 0);
        //axis_2_neutral = false;
        axis_2_up = false;
        axis_2_down = true;
        }
    if (axis_2_val > 502 && axis_2_val <= 522){/*axis_2_neutral = true, */axis_2_up = false, axis_2_down = false, axis_2_out = 0;
    }

//==================process and map axis fingers==================

    if (axis_fingers_val >= 522){
        axis_fingers_out = map(axis_fingers_val, 522, 1023, 0, 255);
        //axis_fingers_neutral = false;
        axis_fingers_close = true;
        axis_fingers_open = false;
        }
        
    if (axis_fingers_val <= 502){
        axis_fingers_out = map(axis_fingers_val, 0, 502, 255, 0);
        //axis_fingers_neutral = false;
        axis_fingers_close = false;
        axis_fingers_open = true;
        }
    if (axis_fingers_val > 502 && axis_fingers_val <= 522){/*axis_fingers_neutral = true, */axis_fingers_close = false, axis_fingers_open = false, axis_fingers_out = 0;
    }

//activate appropriate status LED for up or down and output data to LED on PWM pins
if (axis_1_neutral == true && lock_1 == false){digitalWrite(up_state, LOW), digitalWrite(down_state, LOW);}
if (axis_2_neutral == true && lock_2 == false){digitalWrite(up_state, LOW), digitalWrite(down_state, LOW);}
if (axis_fingers_neutral == true && lock_3 == false){digitalWrite(up_state, LOW), digitalWrite(down_state, LOW);}

if (axis_1_up == true && lock_1 == false){digitalWrite(up_state, HIGH), digitalWrite(down_state, LOW), analogWrite(out_power_1, axis_1_out);}
if (axis_2_up == true && lock_2 == false){digitalWrite(up_state, HIGH), digitalWrite(down_state, LOW), analogWrite(out_power_2, axis_2_out);}
if (axis_fingers_close == true && lock_3 == false){digitalWrite(up_state, HIGH), digitalWrite(down_state, LOW), analogWrite(out_power_fingers, axis_fingers_out);}

if (axis_1_down == true && lock_1 == false){digitalWrite(up_state, LOW), digitalWrite(down_state, HIGH), analogWrite(out_power_1, axis_1_out);}
if (axis_2_down == true && lock_2 == false){digitalWrite(up_state, LOW), digitalWrite(down_state, HIGH), analogWrite(out_power_2, axis_2_out);}
if (axis_fingers_open == true && lock_3 == false){digitalWrite(up_state, LOW), digitalWrite(down_state, HIGH), analogWrite(out_power_fingers, axis_fingers_out);}

tm1637.displayInt(pressure_actual);

//serial debugging
//output data on axis 1
Serial.print("A1V:"); Serial.print(axis_1_val); Serial.print(" A1O:"); Serial.print(axis_1_out);
Serial.print(" A1N:"); Serial.print(axis_1_neutral);
Serial.print(" A1U:"); Serial.print(axis_1_up);
Serial.print(" A1D:"); Serial.print(axis_1_down);
Serial.print(" A1Lock:"); Serial.print(lock_1);
//output data on axis 2
Serial.print(" A2V:"); Serial.print(axis_2_val); Serial.print(" A2O:"); Serial.print(axis_2_out);
Serial.print(" A2N:"); Serial.print(axis_2_neutral);
Serial.print(" A2U:"); Serial.print(axis_2_up);
Serial.print(" A2D:"); Serial.print(axis_2_down);
Serial.print(" A2Lock:"); Serial.print(lock_2);
//output data on axis fingers
Serial.print(" FV:"); Serial.print(axis_fingers_val); Serial.print(" FO:"); Serial.print(axis_fingers_out);
Serial.print(" AFN: "); Serial.print(axis_fingers_neutral);
Serial.print(" AFU: "); Serial.print(axis_fingers_close);
Serial.print(" AFD: "); Serial.print(axis_fingers_open);
Serial.print(" AFLock: "); Serial.print(lock_3);

Serial.print(" Current[mA]:"); Serial.print(current_mA); //print current read from INA module

Serial.print(" Pressure val:"); Serial.print(pressure_val);
Serial.print(" Pressure actual:"); Serial.print(pressure_actual);

Serial.println("  ");

}
