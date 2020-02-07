/*
  This library is for UNObots of Version 1, 2, 3 and 4.
  UNObot.cpp - Library for UNObot V4, V3, V2 and V1
  Created by Florian Zylla, October 5, 2017.
  Last review by Florian Zylla, February 7 , 2020
*/

//Library is set by default to version 4.
//For version 1 or 2 or 3 change '#define V 4' to '#define V 1' or '#define V 2' or '#define V 3'
#define V 4



#include "Arduino.h"
#include "UNObot.h"
#include <LiquidCrystal.h>

#if (V==4)

    //version control
        //view hardware version on the shield
        int hardware_version=4;
        //later the library with the basic functions will be extended with
        //more functions ;in the start up, that will be shown with a + behind the version
        boolean upgrade = false; // will be changed when Bluetooth and Check Button are implemented 

    //Display declaration
        int rs=13;
        int enable=12;
        int d4=14; //lowee(11) -> A0
        int d5=4;
        int d6=15; //upper (13) -> A1
        int d7=2;
        LiquidCrystal lcd(rs, enable, d4, d5, d6, d7);

    //motordriver declaration
          // motordriver will be used in Phase/Enable Mode (Mode PIN is HIGH)
        int A_IN1_EN=5;//Motor A PWM needed for Speed Adjustment
        int A_IN2_PH=3;//Motor A Direction Setting
        int B_IN1_EN=6;//Motor B PWM needed for Speed Adjustment
        int B_IN2_PH=7;//Motor B Direction Setting
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
        #define LS_0 A4 //left IR-Reflectance-Sensor
        #define LS_1 A3 //middle IR-Reflectance-Sensor
        #define LS_2 A2 //right IR-Reflectance-Sensor
    // Ultrasonic Sensor Declaration
        int us_pin=19;    //A5 is echo and trigger
            


    // Custom Character Declaration for bar-functions
        byte bar2[] = {
          B00000,
          B00000,
          B00000,
          B00000,
          B00000,
          B11111,
          B11111,
          B00000
        };
        byte bar3[] = {
          B00000,
          B00000,
          B00000,
          B00000,
          B11111,
          B11111,
          B11111,
          B00000
        };
        byte bar4[] = {
          B00000,
          B00000,
          B00000,
          B11111,
          B11111,
          B11111,
          B11111,
          B00000
        };
        byte bar5[] = {
          B00000,
          B00000,
          B11111,
          B11111,
          B11111,
          B11111,
          B11111,
          B00000
        };
        byte bar6[] = {
          B00000,
          B11111,
          B11111,
          B11111,
          B11111,
          B11111,
          B11111,
          B00000
        };
        byte bar7[] = {
          B11111,
          B11111,
          B11111,
          B11111,
          B11111,
          B11111,
          B11111,
          B00000
        };

    //motors
        void set_motors(int left_speed, int right_speed)
        {
            //set pins to output (pins 0 and 1 also used for serial communication
            pinMode(A_IN1_EN,OUTPUT);
            pinMode(A_IN2_PH,OUTPUT);
            pinMode(B_IN1_EN,OUTPUT);
            pinMode(B_IN2_PH,OUTPUT);
            //turn off both canals
            analogWrite(A_IN1_EN,0);
            analogWrite(B_IN1_EN,0);
            digitalWrite(A_IN2_PH,LOW);
            digitalWrite(B_IN2_PH,LOW);
            //detect backward driving and set phase pin
            if(right_speed<0)
            {
                digitalWrite(A_IN2_PH,HIGH);

            }
            if(left_speed<0)
            {
                digitalWrite(B_IN2_PH,HIGH);

            }
            //make parameters positive
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
            values[0]=analogRead(LS_0);
            values[1]=analogRead(LS_1);
            values[2]=analogRead(LS_2);
            return;
        }

        //ultrasonic sensor
        int get_distance()
        {   
            pinMode(us_pin,OUTPUT);
            digitalWrite(us_pin, LOW);
            delay(5);
            digitalWrite(us_pin, HIGH);
            delay(10);
            digitalWrite(us_pin, LOW);
            pinMode(us_pin,INPUT);
            int dauer = pulseIn(us_pin, HIGH,14600);
            int entfernung = (dauer/2.0) * 0.03432;
            if (entfernung >= 300 )
            {
                entfernung=-2;
            }
            else if (entfernung <= 0)
            {
                entfernung=-1;
            }
            return entfernung;

        }

       
    //buttons
        bool wait_for_button(int button)
        {
            int b_pressed=3;
            switch(button)
            {
                case 0:
                    pinMode(B_0,INPUT_PULLUP);
                    while(digitalRead(B_0)==HIGH)
                    {

                    }
                    return 1;
                case 1:
                    pinMode(B_1,INPUT_PULLUP);
                    while(digitalRead(B_1)==HIGH)
                    {

                    }
                    return 1;
                case 2:
                    pinMode(B_2,INPUT_PULLUP);
                    while(digitalRead(B_2)==HIGH)
                    {

                    }
                    return 1;
            }
        }
        int wait_for_any_button()
        {
            pinMode(B_0,INPUT_PULLUP);
            pinMode(B_1,INPUT_PULLUP);
            pinMode(B_2,INPUT_PULLUP);
            int b_pressed=3;
            while(b_pressed==3)
            {
              if(digitalRead(B_0)==LOW)
              {
                  b_pressed=0;
              }
              if(digitalRead(B_1)==LOW)
              {
                  b_pressed=1;
              }
              if(digitalRead(B_2)==LOW)
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
                digitalWrite(B_2,LOW);
                return;
            }
            else
                digitalWrite(B_2,HIGH);
            return;
        }
        void y_led(boolean state)
        {
            pinMode(B_1,OUTPUT);
            if(state==HIGH)
            {
                digitalWrite(B_1,LOW);
                return;
            }
            else
                digitalWrite(B_1,HIGH);
            return;
        }
        void g_led(boolean state)
        {
            pinMode(B_0,OUTPUT);
            if(state==HIGH)
            {
                digitalWrite(B_0,LOW);
                return;
            }
            else
                digitalWrite(B_0,HIGH);
            return;
        }

        //PWM
        //red led
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
            analogWrite(B_2,255-brightness);
            return;
        }
        //yellow led
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
            analogWrite(B_1,255-brightness);
            return;
        }
        //green led
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
            analogWrite(B_0,255-brightness);
            return;
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
            //turn on first led
            r_led(1);

            delay(500);
            //turn on second led
            y_led(1);
            //set pin usage
            pinMode(A_IN1_EN,OUTPUT);
            pinMode(A_IN2_PH,OUTPUT);
            pinMode(B_IN1_EN,OUTPUT);
            pinMode(B_IN2_PH,OUTPUT);
            
            pinMode(BUZZER,OUTPUT);
            //turn off motors
            set_motors(0,0);
            //initialize display
            lcd.begin(8,2);
            //load custom characters
            lcd.createChar(0, bar2);
            lcd.createChar(1, bar3);
            lcd.createChar(2, bar4);
            lcd.createChar(3, bar5);
            lcd.createChar(4, bar6);
            lcd.createChar(5, bar7);
            //show name and version
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
            //make sound
            buzzer(5000,500);//buzzer deactivates pwm on red led
            //wait and activate green led
            delay(500);
            g_led(1);
            //finish init: turn off leds and set display to normal
            delay(500);
            r_led(0);
            y_led(0);
            g_led(0);
            lcd.clear();
            lcd.home();
        }

        //versions for init
        int get_hardware_version()
        {
            return hardware_version;
        }
        boolean get_software_version()
        {
            return upgrade;
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
        //shows a bar with given height at a given position
        void bar_manually( int height, int column , int row)
        {
            lcd.setCursor(column,row);
            //height 0 and 1 are already part of the character ROM
            switch(height){
                case 0:
                    lcd.write(B00100000);
                break;
                case 1:
                    lcd.write(B01011111);
                break;
                case 2:
                    lcd.write(byte(0));
                break;
                case 3:
                    lcd.write(byte(1));
                break;
                case 4:
                    lcd.write(byte(2));
                break;
                case 5:
                    lcd.write(byte(3));
                break;
                case 6:
                    lcd.write(byte(4));
                break;
                case 7:
                    lcd.write(byte(5));
                break;
            }
        }
        //calculates the height of the bar in a given range by a given value
        //at the requested position
        void bar(int min, int max , int value, int column, int row)
        {

            if(((value>=min)&&(value<=(max/8)))||(value<min))
                bar_manually(0,column, row);
            if((value>(max/8))&&(value<((max*2)/8)))
                bar_manually(1,column, row);
            if((value>((max*2)/8))&&(value<=((max*3)/8)))
                bar_manually(2,column, row);
            if((value>((max*3)/8))&&(value<=((max*4)/8)))
                bar_manually(3,column, row);
            if((value>((max*4)/8))&&(value<=((max*5)/8)))
                bar_manually(4,column, row);
            if((value>((max*5)/8))&&(value<=((max*6)/8)))
                bar_manually(5,column, row);
            if((value>((max*6)/8))&&(value<=((max*7)/8)))
                bar_manually(6,column, row);
            if(((value>((max*7)/8))&&(value<=(max)))||(max<value))
                bar_manually(7,column, row);
        }
#endif



#if (V==3)

    //version control
        //view hardware version on the shield
        int hardware_version=3;
        //later the library with the basic functions will be extended with
        //more functions ;in the start up, that will be shown with a + behind the version
        boolean upgrade = true;

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


    // Custom Character Declaration for bar-functions
        byte bar2[] = {
          B00000,
          B00000,
          B00000,
          B00000,
          B00000,
          B11111,
          B11111,
          B00000
        };
        byte bar3[] = {
          B00000,
          B00000,
          B00000,
          B00000,
          B11111,
          B11111,
          B11111,
          B00000
        };
        byte bar4[] = {
          B00000,
          B00000,
          B00000,
          B11111,
          B11111,
          B11111,
          B11111,
          B00000
        };
        byte bar5[] = {
          B00000,
          B00000,
          B11111,
          B11111,
          B11111,
          B11111,
          B11111,
          B00000
        };
        byte bar6[] = {
          B00000,
          B11111,
          B11111,
          B11111,
          B11111,
          B11111,
          B11111,
          B00000
        };
        byte bar7[] = {
          B11111,
          B11111,
          B11111,
          B11111,
          B11111,
          B11111,
          B11111,
          B00000
        };

    //motors
        void set_motors(int left_speed, int right_speed)
        {
            //set pins to output (pins 0 and 1 also used for serial communication
            pinMode(A_IN1_EN,OUTPUT);
            pinMode(A_IN2_PH,OUTPUT);
            pinMode(B_IN1_EN,OUTPUT);
            pinMode(B_IN2_PH,OUTPUT);
            //turn off both canals
            analogWrite(A_IN1_EN,0);
            analogWrite(B_IN1_EN,0);
            digitalWrite(A_IN2_PH,LOW);
            digitalWrite(B_IN2_PH,LOW);
            //detect backward driving and set phase pin
            if(right_speed<0)
            {
                digitalWrite(A_IN2_PH,HIGH);

            }
            if(left_speed<0)
            {
                digitalWrite(B_IN2_PH,HIGH);

            }
            //make parameters positive
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

        //ultrasonic sensor
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
                    pinMode(B_0,INPUT_PULLUP);
                    while(digitalRead(B_0)==HIGH)
                    {

                    }
                    return 1;
                case 1:
                    pinMode(B_1,INPUT_PULLUP);
                    while(digitalRead(B_1)==HIGH)
                    {

                    }
                    return 1;
                case 2:
                    pinMode(B_2,INPUT_PULLUP);
                    while(digitalRead(B_2)==HIGH)
                    {

                    }
                    return 1;
            }
        }
        int wait_for_any_button()
        {
            pinMode(B_0,INPUT_PULLUP);
            pinMode(B_1,INPUT_PULLUP);
            pinMode(B_2,INPUT_PULLUP);
            int b_pressed=3;
            while(b_pressed==3)
            {
              if(digitalRead(B_0)==LOW)
              {
                  b_pressed=0;
              }
              if(digitalRead(B_1)==LOW)
              {
                  b_pressed=1;
              }
              if(digitalRead(B_2)==LOW)
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
                digitalWrite(B_2,LOW);
                return;
            }
            else
                digitalWrite(B_2,HIGH);
            return;
        }
        void y_led(boolean state)
        {
            pinMode(B_1,OUTPUT);
            if(state==HIGH)
            {
                digitalWrite(B_1,LOW);
                return;
            }
            else
                digitalWrite(B_1,HIGH);
            return;
        }
        void g_led(boolean state)
        {
            pinMode(B_0,OUTPUT);
            if(state==HIGH)
            {
                digitalWrite(B_0,LOW);
                return;
            }
            else
                digitalWrite(B_0,HIGH);
            return;
        }

        //PWM
        //red led
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
        analogWrite(B_2,255-brightness);
        return;
        }
        //yellow led
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
        analogWrite(B_1,255-brightness);
        return;
        }
        //green led
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
        analogWrite(B_0,255-brightness);
        return;
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
            //turn on first led
            r_led(1);
            //set servo to neutral position
            pinMode(servo,OUTPUT);
            for(int i=0; i<20;i++)  //generate PWM 50 Hz
            {
                digitalWrite(servo,HIGH);
                delayMicroseconds(1500);// 1500 is neutral
                digitalWrite(servo,LOW);
                delayMicroseconds(1500);
                delay(17);

            }
            //turn on second led
            y_led(1);
            //set pin usage
            pinMode(A_IN1_EN,OUTPUT);
            pinMode(A_IN2_PH,OUTPUT);
            pinMode(B_IN1_EN,OUTPUT);
            pinMode(B_IN2_PH,OUTPUT);
            pinMode(trigger,OUTPUT);
            pinMode(echo,INPUT);
            pinMode(BUZZER,OUTPUT);
            //turn off motors
            set_motors(0,0);
            //initialize display
            lcd.begin(8,2);
            //load custom characters
            lcd.createChar(0, bar2);
            lcd.createChar(1, bar3);
            lcd.createChar(2, bar4);
            lcd.createChar(3, bar5);
            lcd.createChar(4, bar6);
            lcd.createChar(5, bar7);
            //show name and version
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
            //make sound
            buzzer(5000,500);//buzzer deactivates pwm on red led
            //wait and activate green led
            delay(500);
            g_led(1);
            //finish init: turn off leds and set display to normal
            delay(500);
            r_led(0);
            y_led(0);
            g_led(0);
            lcd.clear();
            lcd.home();
        }

        //versions for init
        int get_hardware_version()
        {
            return hardware_version;
        }
        boolean get_software_version()
        {
            return upgrade;
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
    //shows a bar with given height at a given position
    void bar_manually( int height, int column , int row)
    {
      lcd.setCursor(column,row);
      //height 0 and 1 are already part of the character ROM
      switch(height){
          case 0:
            lcd.write(B00100000);
          break;
          case 1:
            lcd.write(B01011111);
          break;
          case 2:
            lcd.write(byte(0));
          break;
          case 3:
            lcd.write(byte(1));
          break;
          case 4:
            lcd.write(byte(2));
          break;
          case 5:
            lcd.write(byte(3));
          break;
          case 6:
            lcd.write(byte(4));
          break;
          case 7:
            lcd.write(byte(5));
          break;
      }
    }
    //calculates the height of the bar in a given range by a given value
    //at the requested position
    void bar(int min, int max , int value, int column, int row)
    {

          if(((value>=min)&&(value<=(max/8)))||(value<min))
              bar_manually(0,column, row);
          if((value>(max/8))&&(value<((max*2)/8)))
              bar_manually(1,column, row);
          if((value>((max*2)/8))&&(value<=((max*3)/8)))
              bar_manually(2,column, row);
          if((value>((max*3)/8))&&(value<=((max*4)/8)))
              bar_manually(3,column, row);
          if((value>((max*4)/8))&&(value<=((max*5)/8)))
              bar_manually(4,column, row);
          if((value>((max*5)/8))&&(value<=((max*6)/8)))
              bar_manually(5,column, row);
          if((value>((max*6)/8))&&(value<=((max*7)/8)))
              bar_manually(6,column, row);
          if(((value>((max*7)/8))&&(value<=(max)))||(max<value))
              bar_manually(7,column, row);
    }
#endif

#if (V==2)

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
    // -Sensor on a Servo with a 180� angle to detect objects

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
#endif
#if (V==1)
    //display declaration
    int rs=7;
    int enable=6;
    int d4=0;
    int d5=3;
    int d6=1;
    int d7=2;
    LiquidCrystal lcd(rs, enable, d4, d5, d6, d7);

    //motordriver declaration
      // motordriver will be used in Phase/Enable Mode (Mode PIN is HIGH)
    int A_IN1_EN=11;    //Motor A PWM needed for Speed Adjustment
    int A_IN2_PH=12;    //Motor A Direction Setting
    int B_IN1_EN=9;     //Motor B PWM needed for Speed Adjustment
    int B_IN2_PH=8;     //Motor B Direction Setting
    float v_max=255;    //max. speed limit changeable (0<=v_max<=255)

    // buzzer declaration
    int BUZZER=15;//equal A1, driven by tone(PIN,Frequency,Duration)-Command

    /* BUTTON and LED declaration
        some BUTTONs and LEDs are using the same Pins
        to activate the LED you need to declare the PINs as OUTPUT
        to get the BUTTON state you need to use the PINs as INPUT
        the LED always will light up when you press the BUTTON */
    int B_2=10;     //right button with LED
    int B_1=4;      //middle button without LED
    int B_0 = 13;   //left button with LED

    /* Line Sensor declaration
        The robot uses a 3 IR-Reflectance-Sensor-Array, the input values
        are between 0V and 5V or 0 and 1023 as resolution of the 10bit ADC
        (Analog-Digital-Converter).
        small values mean small reflectance (black)
        high values mean high reflectance (white) */
    #define LS_0 A5 //left IR-Reflectance-Sensor
    #define LS_1 A4 //middle IR-Reflectance-Sensor
    #define LS_2 A3 //right IR-Reflectance-Sensor
    int line_sensor_reading=1;
    /*  how often the linesensors are readed
        change that value to 3 or 2 to make the Functions which gets the line
        sensor values faster (Attention: the runtime of each loop changes and
        some programs might run unstable) */

    // Battery Controll declaration
      /* The max.6V Battery Voltage is minimized by a Voltage Divider to
         protect the Arduino Board from a too high voltage input.
         Its necessary to use the INTERNAL reference voltage because the
         DEFAULT reference voltage is rising and falling with the battery
         voltage, so a correct Voltage Calculation is impossible.
         To calculate the Battery Voltage: Divide the Input Value by 1024
         and multiply everything by 1.1 (Volts). */
    #define BAT A2
    int v_ref_internal=1100;//internal reference voltage in millivolts
    int R1=10000;//resistor between VIN Pin(ca. 6V) and the A2 Pin in Ohm
    int R2=1800;//resistor between A2 Pin and GND in Ohm
    int shut_down_voltage=5000;
    /*  checkbattery()-function alerts for battery change,if value is lower
        than the given value; this function was never tested so it might not
        work as expected
    */

    // free pins
    /*  PINs A1 and 5 are free, but the should be used to attach an IR-Distance
        Sensor on a Servo with a 180� angle to detect objects
        There are also other possible usages of the two pins for example the
        Analog Input Pin could be use to measure light intensity with a photo-
        resistor or to add an other IR-Reflectance-Sensor, on the other side
        there are many ways to use Pin 5 because it is also a PWM-Pin
    */

    /* set_motors()-function
        Motors are set to required speed and turning direction
        Too high and too low values are changed to the max speed setting, its
        changeable (v_max)
    */
    void set_motors(int left_speed, int right_speed)
    {
        //turn off both canals
        analogWrite(A_IN1_EN,0);
        analogWrite(B_IN1_EN,0);
        digitalWrite(A_IN2_PH,LOW);
        digitalWrite(B_IN2_PH,LOW);
        //detect backward driving
        if(left_speed<0)
        {
            digitalWrite(A_IN2_PH,HIGH);
            left_speed=left_speed*-1;
        }

        if(right_speed<0)
        {
            digitalWrite(B_IN2_PH,HIGH);
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
        analogWrite(A_IN1_EN,left_speed);
        analogWrite(B_IN1_EN,right_speed);
        return;
    }

    /* read_line_sensors()-function
        Values of the Line Sensors are averaged and written to the values-Array
        The Number of reading is adjustable (line_sensor_reading)
    */
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

    /* wait_for_button()-function
        Function waits till any button or one special button is pressed
        While the function waits the set motor speeds, display, buzzer and
        leds are not changed!
    */
    int wait_for_button(char button)
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
            case 'A':
                pinMode(B_0,INPUT);
                pinMode(B_2,INPUT);
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
    }

    /* led()-functions
        turn on and off the leds
        if left or right buttons are pressed the corresponding led turns on too
    */
    void left_led(boolean state)
    {
        pinMode(B_0,OUTPUT);
        delay(20);
        if(state==HIGH)
        {
            digitalWrite(B_0,HIGH);
            return;
        }
        else
            digitalWrite(B_0,LOW);
        return;
    }
    void right_led(boolean state)
    {
        pinMode(B_2,OUTPUT);
        delay(20);
        if(state==HIGH)
        {
            digitalWrite(B_2,HIGH);
            return;
        }
        else
            digitalWrite(B_2,LOW);
        return;
    }

    /* display functions
        using the liquid crystal library
    */
    void clear()
    {
        lcd.clear();
    }
    void print(String data)
    {
        lcd.print(data);
    }
    void print_long(int data)
    {
        lcd.print(data);
    }
    void lcd_goto_xy(int x, int y)
    {
        lcd.setCursor(x,y);
    }

    /* UNObot_init()-function
        initializes Display and Motors
        should be in the setup-function
        its also possible to do a battery check after each reset or power up
        for that uncomment the check_battery()
        The value where the batteries must be changed is also changeable
        (shut_down_voltage) */
    void UNObot_init()
    {
        set_motors(0,0);
        lcd.begin(8,2);
        lcd.clear();
        //check_battery();
    }

    /* check_battery and batterymillivolts()-functions
        This function was developed because we thought that there might be some
        problems with the falling battery voltage through long time operation
        (changing motor speeds).
        Actually there are no big problems with that, so the function was not
        used in the final version.
        Because of the thing that the hardware is implemented on each board, the
        functions are still here. */
    void check_battery()
    {
        lcd.clear();
        lcd.home();
        lcd.print("Batterie");
        int voltage=batterymillivolts();
        if(voltage>shut_down_voltage)
        {
            lcd.setCursor(2,1);
            lcd.print("okay");
            delay(1000);
            lcd.clear();
            return;
        }
        else
        {
            while(1)
            {
                lcd.home();
                lcd.print("Batterie");
                lcd.setCursor(0,1);
                lcd.print("wechseln");
                left_led(1);
                right_led(1);
                delay(600);
                lcd.clear();
                delay(300);
            }
        }
    }
    int batterymillivolts()
    {
        analogReference(INTERNAL);
        delay(20);
        int input_voltage=analogRead(BAT);
        //batterievoltage=inputvoltage/1023*internal refernece voltage*((R1+R2)/R2)
        int battery_voltage=input_voltage*(1.0/1023.0)*v_ref_internal*((R1+R2)/R2);
        analogReference(DEFAULT);
        delay(20);
        return battery_voltage;
    }

#endif
