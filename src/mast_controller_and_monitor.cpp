/*
Drilling mast control system V0.1 by Oleg "Salty NUggeTZ" Velichko and Igor "Terrano"
Designed to control and monitor a hydraulic system of undisclosed origin and specs
Hypothetical prototype designed with Fritzing and built on a breadboard
The system is built around an Arduino NANO
*/

//include arduino library for use in VSCode PlatformIO
#include <Arduino.h>

//pin definitions

//inputs
int control_axis_1 = A0; //potentiometer joystick for vertical axis 1
int control_axis_2 = A1; //potentiometer joystick for vertical axis 2
int fingers_axis = A2;  //potentiometer joystick for "fingers" axis
int pressure_sensor = A3; //currently simulated by a potentiometer

//outputs

int up_state = 2; //status lights up LED when "up" command is received from any of the vertical axes (ON/OFF output)
int down_state = 4; //status lights up LED when "down" command is received from any of the vertical axes (ON/OFF output)

int power_1 = 6; //PWM output for upwards power of axis 1
int power_2 = 5; //PWM output for upwards power of axis 2
int power_fingers = 3; //PWM output for closing power of "fingers" axis

int display_clk = 12; //CLK pin of the TM1637 display module, perhaps rename later
int display_dio = 13; //DIO pin of the TM1637 display module, perhaps rename later

//variables
bool lock_1 = false; //flag for locking out axis 1
bool lock_2 = false; //flag for locking out axis 2
bool lock_3 = false; //flag for locking out fingers axis

bool axis_1_neutral = true; //flag for axis 1 being in neutral state
bool axis_1_up = true; //flag for axis 1 being in UP state
bool axis_1_down = true; //flag for axis 1 being in DOWN state

bool axis_2_neutral = true; //flag for axis 1 being in neutral state
bool axis_2_up = true; //flag for axis 2 being in UP state
bool axis_2_down = true; //flag for axis 2 being in DOWN state

bool axis_fingers_neutral = true; //flag for axis 1 being in neutral state
bool axis_fingers_close = true; //flag for axis 1 being in UP state
bool axis_fingers_open = true; //flag for axis 1 being in DOWN state


int pressure_val = 0; //raw voltage on the pressure sensor input pin
int pressure_actual = 0; //mapped actual pressure value in bar (atmospheres)

int axis_1_val = 0; //for storing raw input values from the pots
int axis_1_out = 0; //for storing output value mapped to appropriate PWM output

int axis_2_val = 0; //for storing raw input values from the pots
int axis_2_out = 0; //for storing output value mapped to appropriate PWM output

int axis_fingers_val = 0; //for storing raw input values from the pots
int axis_fingers_out = 0; //for storing output value mapped to appropriate PWM output


void setup() {
Serial.begin(115200); //initialize serial communication

//pinmode definitions for inputs
pinMode(control_axis_1, INPUT);
pinMode(control_axis_2, INPUT);
pinMode(fingers_axis, INPUT);
pinMode(pressure_sensor, INPUT);

//pinmode definitions for outputs
pinMode(up_state, OUTPUT);
pinMode(down_state, OUTPUT);
pinMode(display_clk, OUTPUT);
pinMode(display_dio, OUTPUT);


}


//NOTE - center value for joysticks is 512, include hysteresis +-50
//NOTE - pushing UP on the fingers control should CLOSE the fingers, equivalent to UP_POWER

void loop() {

//==================set flags==================
if (axis_2_neutral == false || axis_fingers_neutral == false){lock_1 = true;} //lock axis if EITHER of two other axes are NOT neutral
if (axis_2_neutral == true && axis_fingers_neutral == true){lock_1 = false;} //unlock if BOTH other axes are neutral

if (axis_1_neutral == false || axis_fingers_neutral == false){lock_2 = true;} //lock axis if EITHER of two other axes are NOT neutral
if (axis_1_neutral == true && axis_fingers_neutral == true){lock_2 = false;} //unlock if BOTH other axes are neutral

if (axis_1_neutral == false || axis_2_neutral == false){lock_3 = true;} //lock axis if EITHER of two other axes are NOT neutral
if (axis_1_neutral == true && axis_2_neutral == true){lock_3 = false;} //unlock if BOTH other axes are neutral

if (axis_1_neutral == true){digitalWrite(power_1, LOW);}
if (axis_2_neutral == true){digitalWrite(power_2, LOW);}
if (axis_fingers_neutral == true){digitalWrite(power_fingers, LOW);}

//==================read values if axis is not locked==================
axis_1_val = analogRead(control_axis_1);
axis_2_val = analogRead(control_axis_2);
axis_fingers_val = analogRead(fingers_axis);

////==================read pressure sensor==================
pressure_val = analogRead(pressure_sensor);

//==================process and map axis 1==================
if (axis_1_val >= 562){
        axis_1_out = map(axis_1_val, 562, 1023, 0, 255);
        axis_1_neutral = false;
        axis_1_up = true;
        axis_1_down = false;
        }
        
    if (axis_1_val <= 462){
        axis_1_out = map(axis_1_val, 0, 462, 255, 0);
        axis_1_neutral = false;
        axis_1_up = false;
        axis_1_down = true;
        }
    if (axis_1_val > 462 && axis_1_val <= 562){axis_1_neutral = true, axis_1_up = false, axis_1_down = false, axis_1_out = 0;
    }    
    
//==================process and map axis 2==================
if (axis_2_val >= 562){
        axis_2_out = map(axis_2_val, 562, 1023, 0, 255);
        axis_2_neutral = false;
        axis_2_up = true;
        axis_2_down = false;
        }
        
    if (axis_2_val <= 462){
        axis_2_out = map(axis_2_val, 0, 462, 255, 0);
        axis_2_neutral = false;
        axis_2_up = false;
        axis_2_down = true;
        }
    if (axis_2_val > 462 && axis_2_val <= 562){axis_2_neutral = true, axis_2_up = false, axis_2_down = false, axis_2_out = 0;
    }
    
//==================process and map axis fingers==================
if (axis_fingers_val >= 562){
        axis_fingers_out = map(axis_fingers_val, 562, 1023, 0, 255);
        axis_fingers_neutral = false;
        axis_fingers_close = true;
        axis_fingers_open = false;
        }
        
    if (axis_fingers_val <= 462){
        axis_fingers_out = map(axis_fingers_val, 0, 462, 255, 0);
        axis_fingers_neutral = false;
        axis_fingers_close = false;
        axis_fingers_open = true;
        }
    if (axis_fingers_val > 462 && axis_fingers_val <= 562){axis_fingers_neutral = true, axis_fingers_close = false, axis_fingers_open = false, axis_fingers_out = 0;
    }
    
//activate appropriate status LED for up or down and output data to LED on PWM pins

if (axis_1_neutral == true && lock_1 == false){digitalWrite(up_state, LOW), digitalWrite(down_state, LOW);}
if (axis_2_neutral == true && lock_2 == false){digitalWrite(up_state, LOW), digitalWrite(down_state, LOW);}
if (axis_fingers_neutral == true && lock_3 == false){digitalWrite(up_state, LOW), digitalWrite(down_state, LOW);}

if (axis_1_up == true && lock_1 == false){digitalWrite(up_state, HIGH), digitalWrite(down_state, LOW), analogWrite(power_1, axis_1_out);}
if (axis_2_up == true && lock_2 == false){digitalWrite(up_state, HIGH), digitalWrite(down_state, LOW), analogWrite(power_2, axis_2_out);}
if (axis_fingers_close == true && lock_3 == false){digitalWrite(up_state, HIGH), digitalWrite(down_state, LOW), analogWrite(power_fingers, axis_fingers_out);}

if (axis_1_down == true && lock_1 == false){digitalWrite(up_state, LOW), digitalWrite(down_state, HIGH), analogWrite(power_1, axis_1_out);}
if (axis_2_down == true && lock_2 == false){digitalWrite(up_state, LOW), digitalWrite(down_state, HIGH), analogWrite(power_2, axis_2_out);}
if (axis_fingers_open == true && lock_3 == false){digitalWrite(up_state, LOW), digitalWrite(down_state, HIGH), analogWrite(power_fingers, axis_fingers_out);}


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

Serial.print(" Pressure val:"); Serial.print(pressure_val);

Serial.println("  ");

}
