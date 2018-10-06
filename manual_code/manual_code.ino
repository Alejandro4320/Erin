////Libraries
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"


Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *motorRight = AFMS.getMotor(1);
Adafruit_DCMotor *motorLeft = AFMS.getMotor(3);


// Keypress read through the serial monitor
char command;
long d0; //IR value form arduino (IR Pin 0)
long d1; //IR value form arduino (IR Pin 1)
long d2; //IR value form arduino (IR Pin 2)
long d3; //IR value form arduino (IR Pin 3)

const int IR_PIN0 = 0; //IR ANG Pin 0
const int IR_PIN1 = 1; //IR ANG Pin 1
const int IR_PIN2 = 2; //IR ANG Pin 2
const int IR_PIN3 = 3; //IR ANG Pin 3

#define IR_SENSOR_0_GND 45
#define IR_SENSOR_0_VCC 42
#define IR_SENSOR_1_GND 41
#define IR_SENSOR_1_VCC 38
#define IR_SENSOR_2_GND 37
#define IR_SENSOR_2_VCC 34
#define IR_SENSOR_3_GND 33
#define IR_SENSOR_3_VCC 30



void setup()
{
  //IR Sensors
  pinMode(IR_PIN0, INPUT);
  pinMode(IR_PIN1, INPUT);
  pinMode(IR_PIN2, INPUT);
  pinMode(IR_PIN3, INPUT);

  pinMode(IR_SENSOR_0_GND, OUTPUT);
  pinMode(IR_SENSOR_0_VCC, OUTPUT);
  pinMode(IR_SENSOR_1_GND, OUTPUT);
  pinMode(IR_SENSOR_1_VCC, OUTPUT);
  pinMode(IR_SENSOR_2_GND, OUTPUT);
  pinMode(IR_SENSOR_2_VCC, OUTPUT);
  pinMode(IR_SENSOR_3_GND, OUTPUT);
  pinMode(IR_SENSOR_3_VCC, OUTPUT);

  digitalWrite(IR_SENSOR_0_GND, LOW);
  digitalWrite(IR_SENSOR_1_GND, LOW);
  digitalWrite(IR_SENSOR_2_GND, LOW);
  digitalWrite(IR_SENSOR_3_GND, LOW);

  digitalWrite(IR_SENSOR_0_VCC, HIGH);
  digitalWrite(IR_SENSOR_1_VCC, HIGH);
  digitalWrite(IR_SENSOR_2_VCC, HIGH);
  digitalWrite(IR_SENSOR_3_VCC, HIGH);

  AFMS.begin();
  motorLeft->setSpeed(255);
  motorRight->setSpeed(255);
  Serial.begin(9600);
}

void loop()
{

  if (Serial.available() > 0 ) {
    command = Serial.read();
    Serial.println(command);

    switch(command){
    case'F':
      forward();
      break;
    case'B':
      backward();
      break;
    case'L':
      left();
      break;
    case'R':
      right();
      break;
    case 'S':
      stop();
      break;   
    default:
      stop();
    } 
  }
}


void forward(){                  
  motorLeft->run(FORWARD);  
  motorRight->run(FORWARD);
}

void backward(){ 
  motorLeft->run(BACKWARD);
  motorRight->run(BACKWARD);
}

void left(){
  motorLeft->run(BACKWARD);
  motorRight->run(FORWARD);
}

void right(){
  motorRight->run(BACKWARD);
  motorLeft->run(FORWARD); 
}

void stop(){
  motorLeft->run(RELEASE);   
  motorRight->run(RELEASE);
}


