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

int up_power_1 = 6; //PWM output for upwards power of axis 1
int up_power_2 = 5; //PWM output for upwards power of axis 2
int fingers_close_power = 3; //PWM output for closing power of "fingers" axis

int down_power_1 = 9; //PWM output for downwards power of axis 1
int down_power_2 = 10; //PWM output for downwards power of axis 2
int fingers_open_power = 11; //PWM output for opening power of "fingers" axis

int display_clk = 12; //CLK pin of the TM1637 display module, perhaps rename later
int display_dio = 13; //DIO pin of the TM1637 display module, perhaps rename later

//variables
bool locked = false; //flag for locking out other controls when one is activated
int pressure_val = 0; //raw voltage on the pressure sensor input pin
int pressure_actual = 0; //mapped actual pressure value in bar (atmospheres)
int display_value = 0; //integer to output to the TM1637 display module

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
pinMode(up_power_1, OUTPUT);
pinMode(up_power_2, OUTPUT);
pinMode(fingers_close_power, OUTPUT);
pinMode(down_power_1, OUTPUT);
pinMode(down_power_2, OUTPUT);
pinMode(fingers_open_power, OUTPUT);
pinMode(display_clk, OUTPUT);
pinMode(display_dio, OUTPUT);


}


//NOTE - center value for joysticks is 512, include hysteresis +-50

void loop() {
locked = false;
//digitalWrite(down_state, HIGH);

axis_1_val = analogRead(control_axis_1);
    if (axis_1_val <= 462) {axis_1_out = map(axis_1_val, 0, 462, 255, 0), digitalWrite(down_state, HIGH), analogWrite(down_power_1, axis_1_out);}
    else if (axis_1_val >= 562) {axis_1_out = map(axis_1_val, 562, 1023, 0, 255), digitalWrite(up_state, HIGH), analogWrite(up_power_1, axis_1_out);}
    else axis_1_out = 0, digitalWrite(up_state, LOW), digitalWrite(down_state, LOW), digitalWrite(down_power_1, LOW), digitalWrite(up_power_1, LOW);

axis_2_val = analogRead(control_axis_2);
    if (axis_2_val <= 462) {axis_2_out = map(axis_2_val, 0, 462, 255, 0), digitalWrite(down_state, HIGH), analogWrite(down_power_2, axis_2_out);}
    else if (axis_2_val >= 562) {axis_2_out = map(axis_2_val, 562, 1023, 0, 255), digitalWrite(up_state, HIGH), analogWrite(up_power_2, axis_2_out);}
    else axis_2_out = 0, digitalWrite(up_state, LOW), digitalWrite(down_state, LOW), digitalWrite(down_power_2, LOW), digitalWrite(up_power_2, LOW);

axis_fingers_val = analogRead(fingers_axis);

pressure_val = analogRead(pressure_sensor);


//serial debugging
Serial.print("A1V: "); Serial.print(axis_1_val); Serial.print(" A1O: "); Serial.print(axis_1_out);
Serial.print(" A2V: "); Serial.print(axis_2_val); Serial.print(" A2O: "); Serial.print(axis_2_out);
Serial.print(" FV: "); Serial.print(axis_fingers_val); Serial.print(" FO: "); Serial.print(axis_fingers_out);
Serial.print(" Pressure val: "); Serial.println(pressure_val);

}
