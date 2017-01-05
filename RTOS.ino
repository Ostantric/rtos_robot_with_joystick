/* 
   Copyright (c) 2016 Murat Terzi
   Permission is hereby granted, free of charge, to any person obtaining
   a copy of this software and associated documentation files (the
   "Software"), to deal in the Software without restriction, including
   without limitation the rights to use, copy, modify, merge, publish,
   distribute, sublicense, and/or sell copies of the Software, and to
   permit persons to whom the Software is furnished to do so, subject to
   the following conditions:
   The above copyright notice and this permission notice shall be
   included in all copies or substantial portions of the Software.
   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
   EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
   MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
   NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS
   BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
   ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
   CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
   SOFTWARE.

  FreeRTOS.org V5.0.4 - Copyright (C) 2003-2008 Richard Barry.

  This file is part of the FreeRTOS.org distribution.

  FreeRTOS.org is free software; you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation; either version 2 of the License, or
  (at your option) any later version.

  FreeRTOS.org is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with FreeRTOS.org; if not, write to the Free Software
  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA

  A special exception to the GPL can be applied should you wish to distribute
  a combined work that includes FreeRTOS.org, without being obliged to provide
  the source code for any proprietary components.  See the licensing section
  of http://www.FreeRTOS.org for full details of how and when the exception
  can be applied.

    ***************************************************************************
    ***************************************************************************
    *                                                                         *
      SAVE TIME AND MONEY!  We can port FreeRTOS.org to your own hardware,
      and even write all or part of your application on your behalf.
      See http://www.OpenRTOS.com for details of the services we provide to
      expedite your project.
    *                                                                         *
    ***************************************************************************
    ***************************************************************************

  Please ensure to read the configuration and relevant port sections of the
  online documentation.

  http://www.FreeRTOS.org - Documentation, latest information, license and
  contact details.

  http://www.SafeRTOS.com - A version that is certified for use in safety
  critical systems.

  http://www.OpenRTOS.com - Commercial support, development, porting,
  licensing and training services.
*/

/* FreeRTOS.org includes. */
#include "FreeRTOS_AVR.h"
//#include "task.h"
#include <NewPing.h>
#include <Adafruit_PWMServoDriver.h>
#include <Wire.h>
/* Demo includes. */
#include "basic_io_avr.h"
#define TRIGGER_PIN1  4  // Arduino pin tied to trigger pin on the ultrasonic sensor.
#define ECHO_PIN1    5  // Arduino pin tied to echo pin on the ultrasonic sensor.
#define TRIGGER_PIN2  7  // Arduino pin tied to trigger pin on the ultrasonic sensor.
#define ECHO_PIN2     6  // Arduino pin tied to echo pin on the ultrasonic sensor.
#define TRIGGER_PIN3  8  // Arduino pin tied to trigger pin on the ultrasonic sensor.
#define ECHO_PIN3    9  // Arduino pin tied to echo pin on the ultrasonic sensor.
#define TRIGGER_PIN4  10  // Arduino pin tied to trigger pin on the ultrasonic sensor.
#define ECHO_PIN4    11  // Arduino pin tied to echo pin on the ultrasonic sensor.
#define MAX_DISTANCE 300 // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.
int JoyStick_X = A1; // x
int JoyStick_Y = A0; // y

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
int x, y;
uint8_t servonum1 = 0;
uint8_t servonum2 = 1;
uint8_t servonum3 = 2;
uint8_t servonum4 = 3;
uint8_t servonum5 = 4;
char *sonar1_direction;
char *sonar2_direction;
char *direction_value;
char *edge_right, *edge_left;
unsigned int *s1, *s2, *s3, *s4;
int count_right = 60;
int count_left = 60;
int count_front = 60;
int count_back = 60;
NewPing sonar1(TRIGGER_PIN1, ECHO_PIN1, MAX_DISTANCE); // NewPing setup of pins and maximum distance
NewPing sonar2(TRIGGER_PIN2, ECHO_PIN2, MAX_DISTANCE); // NewPing setup of pins and maximum distance
NewPing sonar3(TRIGGER_PIN3, ECHO_PIN3, MAX_DISTANCE); // NewPing setup of pins and maximum distance
NewPing sonar4(TRIGGER_PIN4, ECHO_PIN4, MAX_DISTANCE); // NewPing setup of pins and maximum distance
/* The task function. */
void Top_Servo_Drive( void *pvParameters );
void Sense( void *pvParameters );
void Actuator_Drive( void *pvParameters );
void Alert_Check( void *pvParameters );


/* Define the strings that will be passed in as the task parameters.  These are
  defined const and off the stack to ensure they remain valid when the tasks are
  executing. */
const char *pcTextForTask1 = "Sonar sensing \r\n";
const char *pcTextForTask2 = "Joystick reading \t\n";
const char *pcTextForTask3 = "Driving Top Servo resets here \t\n";
const char *pcTextForTask4 = "LED driving \t\n";

SemaphoreHandle_t sem;
/*-----------------------------------------------------------*/
struct alert_state
{
  boolean left;
  boolean right;
  boolean front;
  boolean back;
  boolean edgeleft;
  boolean edgeright;
};
alert_state alert;
void setup( void )
{
  //Serial.begin(9600);
  Serial.begin(115200); // Open serial monitor at 115200 baud to see ping results.
  pwm.begin();
  pwm.setPWMFreq(70);  // Analog servos run at ~60 Hz updates

  /* note that some tasks are independent therefore i believe priority doesnt matter for those */
  xTaskCreate( Sense, "Task 1", 150, (void*)pcTextForTask1, 1 , NULL ); /*create the first task at priority 1 */
  xTaskCreate( Top_Servo_Drive, "Task 2", 150, (void*)pcTextForTask3, 2, NULL ); /*create the second task at priority 1. */
  xTaskCreate( Alert_Check, "Task 3", 150, (void*)pcTextForTask4, 1, NULL ); /*create the third task at priority 2. */
  xTaskCreate( Actuator_Drive, "Task 4", 150, (void*)pcTextForTask2, 2 , NULL );/* create the fourtg task at priority 3. */



  vTaskStartScheduler();  /* Start the scheduler so our tasks start executing. */
  for ( ;; ); //this is for incase of emergency. Microcontroller sits here if there is a problem with the scheduler.
  //Limbo mode
  //once it enters here, it never goes out.
}
/*-----------------------------------------------------------*/

void Top_Servo_Drive( void *pvParameters )
{
  char *pcTaskName;
  TickType_t xLastWakeTime;
  pcTaskName = ( char * ) pvParameters;
  xLastWakeTime = xTaskGetTickCount();
  for ( ;; )
  {
    //    vPrintString( pcTaskName ); /* Print out the name of this task. */
    //    vPrintString(" ");
    for (int i = 550; i > 150; i = i - 10)
    {
      sonar1_direction = topsensor1detection(i);
      sonar2_direction = topsensor2detection(i);
      //      vPrintString(" ");
      //      vPrintString("Sonar1_Facing:");
      //      vPrintString(sonar1_direction);
      //      vPrintString(" ");
      //      vPrintString("Sonar2_Facing:");
      //      vPrintString(sonar2_direction);
      //      vPrintString(" ");
      pwm.setPWM(servonum5, 0, i);
      vTaskDelayUntil( &xLastWakeTime, ( 20 / portTICK_PERIOD_MS ) );
    }
    for (int i = 150; i < 550; i = i + 10)
    {
      sonar1_direction = topsensor1detection(i);
      sonar2_direction = topsensor2detection(i);
      //      vPrintString(" ");
      //      vPrintString("Sonar1_Facing:");
      //      vPrintString(sonar1_direction);
      //      vPrintString(" ");
      //      vPrintString("Sonar2_Facing:");nj
      //      vPrintString(sonar2_direction);
      //      vPrintString(" ");
      pwm.setPWM(servonum5, 0, i);
      vTaskDelayUntil( &xLastWakeTime, ( 20 / portTICK_PERIOD_MS ) );
    }
  }
}
void Sense( void *pvParameters )
{
  char *pcTaskName;
  TickType_t xLastWakeTime;
  pcTaskName = ( char * ) pvParameters;
  xLastWakeTime = xTaskGetTickCount();

  for ( ;; )
  {
    //    vPrintString( pcTaskName ); /* Print out the name of this task. */

   
    

    unsigned int uS1 = sonar1.ping();
    vTaskDelayUntil( &xLastWakeTime, ( 2 / portTICK_PERIOD_MS ) );
    unsigned int uS2 = sonar2.ping();
    vTaskDelayUntil( &xLastWakeTime, ( 2 / portTICK_PERIOD_MS ) );
    unsigned int uS3 = sonar3.ping();
   vTaskDelayUntil( &xLastWakeTime, ( 2 / portTICK_PERIOD_MS ) );
    unsigned int uS4 = sonar4.ping();
    vTaskDelayUntil( &xLastWakeTime, ( 2 / portTICK_PERIOD_MS ) );
    s1 = sonar1.convert_cm(uS1);
    vTaskDelayUntil( &xLastWakeTime, ( 2 / portTICK_PERIOD_MS ) );
    s2 = sonar2.convert_cm(uS2);
    vTaskDelayUntil( &xLastWakeTime, ( 2 / portTICK_PERIOD_MS ) );
    s3 = sonar3.convert_cm(uS3);
  vTaskDelayUntil( &xLastWakeTime, ( 2 / portTICK_PERIOD_MS ) );
    s4 = sonar4.convert_cm(uS4);
    vTaskDelayUntil( &xLastWakeTime, ( 2 / portTICK_PERIOD_MS ) );
   // vPrintStringAndNumber(" Sonar1:", sonar1.convert_cm(uS1));
    //vPrintStringAndNumber(" Sonar2:", sonar2.convert_cm(uS2));
    //vPrintStringAndNumber(" Sonar3:", sonar3.convert_cm(uS3));
    //vPrintStringAndNumber(" Sonar4:", sonar4.convert_cm(uS4));
    count_right = count_function(s1, sonar1_direction, "right" , count_right);
    count_back = count_function(s1, sonar1_direction, "back", count_back);
    count_left = count_function(s2, sonar2_direction, "left", count_left);
    count_front = count_function(s2, sonar2_direction, "front", count_front);
    //    vPrintStringAndNumber(" count_right:", count_right);
    //    vPrintStringAndNumber(" count_left:", count_left);
    //    vPrintStringAndNumber(" count_front:", count_front);
    //    vPrintStringAndNumber(" count_back:", count_back);
//    vPrintStringAndNumber(" X-axis=>", x);
//    vPrintStringAndNumber(" Y-axis=>", y);
//    vPrintString("\n");
    vTaskDelayUntil( &xLastWakeTime, ( 100 / portTICK_PERIOD_MS ) );
    /* This task executes exactly every 150 milliseconds. */
  }
}
void Alert_Check( void *pvParameters )
{
  char *pcTaskName;
  TickType_t xLastWakeTime;
  pcTaskName = ( char * ) pvParameters;
  xLastWakeTime = xTaskGetTickCount();
  pinMode(13, OUTPUT);
  pinMode(12, OUTPUT);
  pinMode(2, OUTPUT);
  pinMode(3, OUTPUT);
  for (;;)
  {
    //BACK
    if (s1 < 55 && s1 > 5 && sonar1_direction == "back")
    {
      digitalWrite(2, HIGH);
      alert.back = true;
    }
   vTaskDelayUntil( &xLastWakeTime, ( 2 / portTICK_PERIOD_MS ) );
    //FRONT
    if (s2 < 85 && s2 > 5 && sonar2_direction == "front")
    {
      digitalWrite(3, HIGH);
      alert.front = true;
    }
  vTaskDelayUntil( &xLastWakeTime, ( 2 / portTICK_PERIOD_MS ) );
    //RIGHT
    if (s1 < 35 && s1 > 5 && sonar1_direction == "right")
    {
      digitalWrite(13, HIGH);
      alert.right = true;
    }
  vTaskDelayUntil( &xLastWakeTime, ( 2 / portTICK_PERIOD_MS ) );
    //LEFT
    if (s2 < 35 && s2 > 5 && sonar2_direction == "left")
    {
      digitalWrite(12, HIGH);
      alert.left = true;
    }
vTaskDelayUntil( &xLastWakeTime, ( 2 / portTICK_PERIOD_MS ) );
    
    if (s3 > 20)
    {
      alert.edgeright = true;
    }
    else
    {
      alert.edgeright = false;
    }
 
    if (s4 > 20)
    {
      alert.edgeleft = true;
    }
    else
    {
      alert.edgeleft = false;
    }
vTaskDelayUntil( &xLastWakeTime, ( 2 / portTICK_PERIOD_MS ) );
    if (count_front < 25)
    {
      alert.front = false;
      digitalWrite(3, LOW);
    }
  
    if (count_back < 25)
    {
      alert.back = false;
      digitalWrite(2, LOW);
    }
    
    if (count_right < 25)
    {
      alert.right = false;
      digitalWrite(13, LOW);
    }
   
    if (count_left < 25)
    {
      alert.left = false;
      digitalWrite(12, LOW);
    }

    //    vPrintString( pcTaskName ); /* Print out the name of this task. */
    vTaskDelayUntil( &xLastWakeTime, ( 25 / portTICK_PERIOD_MS ) );
    /* This task executes exactly every 150 milliseconds. */
  }
}
void Actuator_Drive( void *pvParameters )
{
  char *pcTaskName;
  TickType_t xLastWakeTime;

  pcTaskName = ( char * ) pvParameters;

  xLastWakeTime = xTaskGetTickCount();
  for ( ;; )
 
  {
     x = analogRead (JoyStick_X);
    y = analogRead (JoyStick_Y);
//    vPrintStringAndNumber(" forward.alert:", alert.front);
//    vPrintStringAndNumber(" backward.alert:", alert.back);
//    vPrintStringAndNumber(" left.alert:", alert.left);
//    vPrintStringAndNumber(" right.alert:", alert.right);
    vPrintStringAndNumber(" edgeright:", alert.edgeright);
    vPrintStringAndNumber(" edgeleft:", alert.edgeleft);
      vPrintString("\n");
    servo_drive(x, y, servonum1, servonum2, servonum3, servonum4, alert);



    vTaskDelayUntil( &xLastWakeTime, ( 100 / portTICK_PERIOD_MS ) );
  }
}

char *topsensor1detection(int i)
{
  char *sonar;
  if (i <= 560 && i >= 400)
  {
    sonar = "back";
  }
  
  if (i < 400 && i >= 220)
  {
    sonar = "right";
  }


  return sonar;
}
char *topsensor2detection(int i)
{
  char *sonar;
  if (i <= 560 && i >= 400)
  {
    sonar = "front";
  }
   if (i < 180 && i >= 150)
  {
    sonar = "back";
  }
  
  if (i < 400 && i >= 180)
  {
    sonar = "left";
  }
  return sonar;
}
int *count_function(unsigned int s, char sonar, char facing, int count )
{
  if ((s < 60 && s > 5) && (sonar == facing))
  {
    count = 50;
  }
  else
  {
    count = count - 1;
  }

  if (count < 5)
  {
    count = 60;
  }
  return count;
}
int *count_function_front(unsigned int s, char sonar, char facing, int count )
{
  if ((s < 65 && s > 5) && (sonar == facing))
  {
    count = 50;
  }
  else
  {
    count = count - 1;
  }

  if (count < 5)
  {
    count = 50;
  }
  return count;
}

int servo_drive(int valX, int valY, int servonum1, int servonum2, int servonum3, int servonum4, alert_state alert_flag)

{
  int  valX_map;   // variable to read the value from the analog pin
  int valY_map;   // variable to read the value from the analog pin
  valX_map = map(valX, 50, 950, 348, 390);
  valY_map = map(valY, 50, 950, 348, 390);
  if ((valY <= 650) && (valY >= 450) && ((valX >= 450) && (valX <= 650)))
  {
    pwm.setPWM(servonum1, 0, 0);
    pwm.setPWM(servonum2, 0, 0);
    pwm.setPWM(servonum3, 0, 0);
    pwm.setPWM(servonum4, 0, 0);
    vPrintString("IDLE");
    vPrintString("\n");
  }
  //FORWARD

  else if  ((valY >= 0 && valY < 650 && valX > 450 && valX < 580 && alert_flag.front == false) && (alert_flag.edgeright == false) && (alert_flag.edgeleft == false))
   //or alert_flag.edgeright == false or alert_flag.edgeleft == false
  {
    pwm.setPWM(servonum1, 0, (380 - valY_map) + 348);
    pwm.setPWM(servonum2, 0, (380 - valY_map) + 348);
    pwm.setPWM(servonum3, 0, valY_map);
    pwm.setPWM(servonum4, 0, valY_map);
    vPrintString("BACKWARD");
    vPrintString("\n");
  }
  //BACKWARD
   else if ((valY <= 1023) && (valY > 650) && (valX > 450) && (valX < 650) && alert_flag.back == false)
  {
    pwm.setPWM(servonum1, 0, (380 - valY_map) + 348);
    pwm.setPWM(servonum2, 0, (380 - valY_map) + 348);
    pwm.setPWM(servonum3, 0, valY_map);
    pwm.setPWM(servonum4, 0, valY_map);
    vPrintString("FORWARD");
    vPrintString("\n");
  }
  //RIGHT
  else if (((valY < 650) && (valY > 450) && (valX >= 0) && (valX < 650) && alert_flag.right == false))// && ((alert_flag.edgeright == true) || (alert_flag.edgeleft == false)) ) //double check this line, before presentation!
  {
    pwm.setPWM(servonum1, 0, valX_map);
    pwm.setPWM(servonum2, 0, valX_map);
    pwm.setPWM(servonum3, 0, valX_map);
    pwm.setPWM(servonum4, 0, valX_map);
    vPrintString("RIGHT");
    vPrintString("\n");
  }
  //LEFT
  else if (((valY < 650) && (valY > 450) && (valX > 650) && (valX <= 1023) && alert_flag.left == false))// && ((alert_flag.edgeleft == true) || (alert_flag.edgeright == false))) //double check this line, before presentation!
  {
    pwm.setPWM(servonum1, 0, valX_map);
    pwm.setPWM(servonum2, 0, valX_map);
    pwm.setPWM(servonum3, 0, valX_map);
    pwm.setPWM(servonum4, 0, valX_map);
    vPrintString("LEFT");
    vPrintString("\n");
  }
  //stop
  else
  {
    pwm.setPWM(servonum1, 0, 0);
    pwm.setPWM(servonum2, 0, 0);
    pwm.setPWM(servonum3, 0, 0);
    pwm.setPWM(servonum4, 0, 0);
    vPrintString("NOT SAFE");
    vPrintString("\n");
  }

  
}
//------------------------------------------------------------------------------
void loop() {}

