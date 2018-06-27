/*
  This is V2.0, not compatible with UNObot Shields of V1.0
  UNObot.cpp - Library for UNObot V2.
  Created by Florian Zylla, February 8, 2018.
*/
#include "Arduino.h"
#include "UNObot.h"
#include <LiquidCrystal.h>

//version control
    //view hardware version on the shield
    int hardware_version=2;
    //later the library with the basic functions will be extended with
    //more functions ;in the start up, that will be shown with a + behind the version
    boolean upgrade = false;

//Display declaration
    int rs=13;
    int enable=12;
    int d4=7;
    int d5=4;
    int d6=3;
    int d7=2;
    LiquidCrystal lcd(rs, enable, d4, d5, d6, d7);

//motordriver declaration
      // motordriver will be used in Phase/Enable Mode (Mode PIN is HIGH)
    int A_IN1_EN=5;//Motor A PWM needed for Speed Adjustment
    int A_IN2_PH=0;//Motor A Direction Setting
    int B_IN1_EN=6;//Motor B PWM needed for Speed Adjustment
    int B_IN2_PH=1;//Motor B Direction Setting
    int v_max=255; //max. speed limit changeable (0<=v_max<=255)

// buzzer declaration
    int BUZZER=8;//driven by tone(PIN,Frequency,Duration)-Command

// BUTTON and LED declaration
      // some BUTTONs and LEDs are using the same Pins
      // to activate the LED you need to declare the PINs as OUTPUT
      // to get the BUTTON state you need to use the PINs as INPUT
      // the LED always will light up when you press the BUTTON
    int B_2=9;//right button with red LED
    int B_1=10;//middle button with yellow LED
    int B_0 = 11;//left button with green LED

// Line Sensor declaration
      // The robot uses a 3 IR-Reflectance-Sensor-Array, the input values
      // are between 0V and 5V or 0 and 1023 as resolution of the 8bit ADC
      // (Analog-Digital-Converter).
      // small values mean small reflectance (black)
      // high values mean high reflectance (white)
    #define LS_0 A3 //left IR-Reflectance-Sensor
    #define LS_1 A2 //middle IR-Reflectance-Sensor
    #define LS_2 A1 //right IR-Reflectance-Sensor
    int line_sensor_reading=1; // how often the linesensors are readed
    // a smaller number of readings means a faster function
// Ultrasonic Sensor Declaration
        int echo=18;    //A4 is echo
        int trigger=19; //A5 is trig
        int ultrasonic_reading=1; // how often the distance is readed
    // a smaller number of readings means a faster function
//Servo Declaration
        int servo=14;//A0 is Servo Data

#define BUTTON_A 0
#define BUTTON_B 1
#define BUTTON_C 2
#define ANY_BUTTON 3
// PINs A1 and 5 are free, but the should be used to attach an IR-Distance
// -Sensor on a Servo with a 180° angle to detect objects

//motors
    void set_motors(int left_speed, int right_speed)
    {
        pinMode(A_IN1_EN,OUTPUT);
        pinMode(A_IN2_PH,OUTPUT);
        pinMode(B_IN1_EN,OUTPUT);
        pinMode(B_IN2_PH,OUTPUT);
        //turn off both canals
        analogWrite(A_IN1_EN,0);
        analogWrite(B_IN1_EN,0);
        digitalWrite(A_IN2_PH,LOW);
        digitalWrite(B_IN2_PH,LOW);
        //detect backward driving
        if(right_speed<0)
        {
            digitalWrite(A_IN2_PH,HIGH);

        }
        if(left_speed<0)
        {
            digitalWrite(B_IN2_PH,HIGH);

        }
        if(left_speed<0)
        {

            left_speed=left_speed*-1;
        }
        if(right_speed<0)
        {

            right_speed=right_speed*-1;
        }
        //detect and change too high values
        if(left_speed>v_max)
        {
            left_speed=v_max;
        }
        if(right_speed>v_max)
        {
            right_speed=v_max;
        }
        //set speed
        analogWrite(A_IN1_EN,right_speed);
        analogWrite(B_IN1_EN,left_speed);

        return;
    }

//sensors
    //linesensors
    void read_line_sensors(int values[])
    {
        int sum[3]={0,0,0};
        //Read and write values
        for(int i=0;i<line_sensor_reading;i++)
        {
            sum[0]= sum[0]+analogRead(LS_0);
            sum[1]= sum[1]+analogRead(LS_1);
            sum[2]= sum[2]+analogRead(LS_2);
        }
        values[0]=sum[0]/(1.0*line_sensor_reading);
        values[1]=sum[1]/(1.0*line_sensor_reading);
        values[2]=sum[2]/(1.0*line_sensor_reading);
        return;
    }

    //ultrasonic sensor measurement
    int get_distance()
    {
        int sum=0;
        int real_values=ultrasonic_reading;
        for(int i=0; i<ultrasonic_reading;i++)
        {
            digitalWrite(trigger, LOW);
            delay(5);
            digitalWrite(trigger, HIGH);
            delay(10);
            digitalWrite(trigger, LOW);
            int dauer = pulseIn(echo, HIGH,17500);
            int entfernung = (dauer/2) * 0.03432;
            if (entfernung >= 300 || entfernung <= 0)
                {
                    entfernung=0;
                    real_values--;
                }
            sum += entfernung;
        }
        if(real_values==0)
            return 0;
        return sum/real_values;

    }

    void scan_front(int data[])
    {
        //not need until now
    }
//buttons
    bool wait_for_button(int button)
    {
        int b_pressed=3;
        switch(button)
        {
            case 0:
                pinMode(B_0,INPUT);
                while(digitalRead(B_0)==LOW)
                {

                }
                return 1;
            case 1:
                pinMode(B_1,INPUT);
                while(digitalRead(B_1)==LOW)
                {

                }
                return 1;
            case 2:
                pinMode(B_2,INPUT);
                while(digitalRead(B_2)==LOW)
                {

                }
                return 1;
        }
    }
    int wait_for_any_button()
    {
        pinMode(B_0,INPUT);
        pinMode(B_1,INPUT);
        pinMode(B_2,INPUT);
        int b_pressed=3;
        while(b_pressed==3)
        {
          if(digitalRead(B_0)==HIGH)
          {
              b_pressed=0;
          }
          if(digitalRead(B_1)==HIGH)
          {
              b_pressed=1;
          }
          if(digitalRead(B_2)==HIGH)
          {
              b_pressed=2;
          }

        }
                return b_pressed;
    }




//leds
    void r_led(boolean state)
    {
        pinMode(B_2,OUTPUT);
        if(state==HIGH)
        {
            digitalWrite(B_2,HIGH);
            return;
        }
        else
            digitalWrite(B_2,LOW);
        return;
    }
    void y_led(boolean state)
    {
        pinMode(B_1,OUTPUT);
        if(state==HIGH)
        {
            digitalWrite(B_1,HIGH);
            return;
        }
        else
            digitalWrite(B_1,LOW);
        return;
    }
    void g_led(boolean state)
    {
        pinMode(B_0,OUTPUT);
        if(state==HIGH)
        {
            digitalWrite(B_0,HIGH);
            return;
        }
        else
            digitalWrite(B_0,LOW);
        return;
    }

    //PWM
    //right led
    void r_led_pwm(int brightness)
    {
    pinMode(B_2,OUTPUT);
    //minimal value is 0
    if(brightness<0)
        brightness=0;
    //maximal value is 255
    if(brightness>255)
        brightness=255;
    //set brightness
    analogWrite(B_2,brightness);
    return;
    }
    //middle led
    void y_led_pwm(int brightness)
    {
    pinMode(B_1,OUTPUT);
    //minimal value is 0
    if(brightness<0)
        brightness=0;
    //maximal value is 255
    if(brightness>255)
        brightness=255;
    //set brightness
    analogWrite(B_1,brightness);
    return;
    }
    //left led
    void g_led_pwm(int brightness)
    {
    pinMode(B_0,OUTPUT);
    //minimal value is 0
    if(brightness<0)
        brightness=0;
    //maximal value is 255
    if(brightness>255)
        brightness=255;
    //set brightness
    analogWrite(B_0,brightness);
    return;
    }

//display
void clear()
{
    lcd.clear();
}
void print(String data)
{
    lcd.print(data);
}
void print(int data)
{
    lcd.print(data);
}
void lcd_goto_xy(int x, int y)
{
    lcd.setCursor(x,y);
}

//buzzer

void buzzer(int frequency, int duration)
{
    tone(BUZZER,frequency,duration);
}

//others
    //init function
    void UNObot_init()
    {
        r_led(1);
        pinMode(servo,OUTPUT);
        for(int i=0; i<20;i++)  //generate PWM 50 Hz
        {
            digitalWrite(servo,HIGH);
            delayMicroseconds(1500);// 1500 is neutral
            digitalWrite(servo,LOW);
            delayMicroseconds(1500);
            delay(17);

        }
        y_led(1);
        pinMode(A_IN1_EN,OUTPUT);
        pinMode(A_IN2_PH,OUTPUT);
        pinMode(B_IN1_EN,OUTPUT);
        pinMode(B_IN2_PH,OUTPUT);
        pinMode(trigger,OUTPUT);
        pinMode(echo,INPUT);
        pinMode(BUZZER,OUTPUT);
        set_motors(0,0);

        lcd.begin(8,2);
        lcd.clear();
        lcd.setCursor(1,0);
        lcd.print("UNObot");
        lcd.setCursor(3,1);
        lcd.print("V");
        lcd.setCursor(4,1);
        lcd.print(get_hardware_version());
        lcd.setCursor(5,1);
        if(get_software_version())
            lcd.print("+");
        buzzer(5000,500);//buzzer deactivates pwm on red led
        delay(500);
        g_led(1);
        delay(500);
        r_led(0);
        y_led(0);
        g_led(0);






        lcd.clear();
        lcd.home();

    }

    //versions
    int get_hardware_version()
    {
        return hardware_version;
    }
    boolean get_software_version()
    {
        return upgrade;
    }

