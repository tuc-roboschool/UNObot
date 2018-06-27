/*
  UNObot.h - Library for UNObot V1 and V2 by Roboschool .
  Created by Florian Zylla, June 27, 2018.

*/
#ifndef UNObot_h
#define UNObot_h
#include "Arduino.h"

void set_motors(int left_speed, int right_speed);
void read_line_sensors(int values[]);
int get_distance();
void scan_front(int data[]);
bool wait_for_button(int button);
int wait_for_button(char button); //only for use in V1
int wait_for_any_button();
void left_led(boolean state); //only for use in V1
void right_led(boolean state); //only for use in V1
void r_led(boolean state);
void y_led(boolean state);
void g_led(boolean state);
void r_led_pwm(int brightness);
void y_led_pwm(int brightness);
void g_led_pwm(int brightness);
void clear();
void print(String data);
void print(int data);
void print_long(int data);  //only for use in V1
void lcd_goto_xy(int x, int y);
void buzzer(int frequency, int duration);
void UNObot_init();
void check_battery();   //only for use in V1
int batterymillivolts();    //only for use in V1
int get_hardware_version();
boolean get_software_version();

#endif

