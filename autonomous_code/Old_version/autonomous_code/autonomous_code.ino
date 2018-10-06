////Libraries
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"
////=============================================

//======= Variable Initialization =============//

// Creating Motor shield and motor objects
// Create Adafruit_Motorshield Object
// Argument for AFMS.getMotor() is the port on the motor shield that the motor is connected to. NOTE: ASTERIX IS NECESSARY IN THESE STATEMENTS.
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *motorRight = AFMS.getMotor(1);
Adafruit_DCMotor *motorLeft = AFMS.getMotor(3);

// Keypress read through the serial monitor
long input_distance;
double wheel_circum = 12.57;
double encoder_counts;
// variables to store the number of encoder pulses
// for each motor
volatile int leftCount = 0;
volatile int rightCount = 0;

long d0; //IR value form arduino (IR Pin 0)
long d1; //IR value form arduino (IR Pin 1)
long d2; //IR value form arduino (IR Pin 2)
long d3; //IR value form arduino (IR Pin 3)


const int IR_PIN0 = 0; //IR ANG Pin 0
const int IR_PIN1 = 1; //IR ANG Pin 1
const int IR_PIN2 = 2; //IR ANG Pin 2
const int IR_PIN3 = 3; //IR ANG Pin 3

//const int US_PIN4 = 4; //US ANG Pin 4
//const int US_PIN5 = 5; //US ANG Pin 5
//long anVolt1,anVolt2, mm, inches;

#define IR_SENSOR_0_GND 45
#define IR_SENSOR_0_VCC 42
#define IR_SENSOR_1_GND 41
#define IR_SENSOR_1_VCC 38
#define IR_SENSOR_2_GND 37
#define IR_SENSOR_2_VCC 34
#define IR_SENSOR_3_GND 33
#define IR_SENSOR_3_VCC 30

#define US_SENSOR_4_GND 49
#define US_SENSOR_4_VCC 46
#define US_SENSOR_5_GND 53
#define US_SENSOR_5_VCC 50



// pins for the encoder inputs
#define RH_ENCODER_A 2
#define RH_ENCODER_B 4
#define RH_ENCODER_GND 8
#define RH_ENCODER_VCC 9

#define LH_ENCODER_GND 7
#define LH_ENCODER_VCC 6
#define LH_ENCODER_A 3
#define LH_ENCODER_B 5



void setup()
{
  //Encoders
  pinMode(RH_ENCODER_A, INPUT);
  pinMode(RH_ENCODER_B, INPUT);
  pinMode(LH_ENCODER_A, INPUT);
  pinMode(LH_ENCODER_B, INPUT);

  pinMode(RH_ENCODER_GND, OUTPUT);
  pinMode(RH_ENCODER_VCC, OUTPUT);
  pinMode(LH_ENCODER_GND, OUTPUT);
  pinMode(LH_ENCODER_VCC, OUTPUT);

  digitalWrite(RH_ENCODER_GND, LOW);
  digitalWrite(LH_ENCODER_GND, LOW);
  digitalWrite(RH_ENCODER_VCC, HIGH);
  digitalWrite(LH_ENCODER_VCC, HIGH);

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

  // initialize hardware interrupts.
  // Interrupts are useful for making things happen automatically in microcontroller programs, and can help solve timing problems.
  // Often number 0 (for digital pin 2) or number 1 (for digital pin 3) were used
  attachInterrupt(0, rightEncoder, CHANGE);
  attachInterrupt(1, leftEncoder, CHANGE);

  // Initialize mtor shield and motor speeds
  // Set the speed of the motor using setSpeed(speed) where the speed ranges from 0 (stopped) to 255 (full speed)
  // Initialize serial communication:
  AFMS.begin();
  Serial.begin(9600);

  motorLeft->setSpeed(255);
  motorRight->setSpeed(255);
  
  delay(1000);
  Serial.print("Input Distance (in cm):  ");
  
}

void loop()
{
   // Read any serial input given
  if (Serial.available() > 0 ) {
    encoder_counts = 0;
    input_distance = Serial.parseInt();
    Serial.print("You inputed ");
    Serial.print(input_distance);
    Serial.println(" cm.");
    Serial.println();
  }
  // Convert input distance to enoder counts
  // To get 12,000 is 12 multiplied by the gear ratio which is 1000:1
  // whell_circum is the circumerence of the wheel
  encoder_counts = (((input_distance * 11500) / wheel_circum)) / 2;

  d0 = analogRead(IR_PIN0);
  d1 = analogRead(IR_PIN1);
  d2 = analogRead(IR_PIN2);
  d3 = analogRead(IR_PIN3);

  if(rightCount>=encoder_counts || (rightCount*-1)>=encoder_counts || leftCount>=encoder_counts || (leftCount*-1)>=encoder_counts|| d0<= 350 || d1<=350 || d2<350 || d3<350){                     

    
    motorLeft->run(RELEASE);                     // Set the motor directions
    motorRight->run(RELEASE);
    rightCount = encoder_counts + 1 ;
    leftCount = encoder_counts + 1 ; 
    encoder_counts = 0;
    delay(100);
  }
  
  
  
  else{
    motorLeft->run(FORWARD);                     // Set the motor directions
    motorRight->run(FORWARD); 
    Serial.print("Right Count: ");
    Serial.print(rightCount);
    Serial.print(", Left Count: ");
    Serial.println(leftCount);
    delay(10);
  }
}


// Encoder event for the interrupt call
void rightEncoder()
{
  // If the first wave is rising
  if (digitalRead(RH_ENCODER_A) == HIGH)
  {
    // And the second wave is behind
    if (digitalRead(RH_ENCODER_B) == LOW)
    {
      // Right encoder count +1
      rightCount++;
    }
    // Or if the second wave is ahead
    else
    {
      // Right encoder count -1
      rightCount--;
    }
  }
  // If the first wave is falling
  else
  {
    // And the second wave is ahead
    if (digitalRead(RH_ENCODER_B) == LOW)
    {
      // Right encoder count -1
      rightCount--;
    }
    // And the second wave is behind
    else
    {
      // Right encoder count +1
      rightCount++;
    }
  }
}

// Encoder event for the interrupt call
void leftEncoder()
{
  // If the first wave is rising
  if (digitalRead(LH_ENCODER_A) == HIGH)
  {
    // And the second wave is behind
    if (digitalRead(LH_ENCODER_B) == LOW)
    {
      // Left encoder count +1
      leftCount++;
    }
    // Or if the second wave is ahead
    else
    {
      // Left encoder count -1
      leftCount--;
    }
  }
  // If the first wave is falling
  else
  {
    // And the second wave is ahead
    if (digitalRead(LH_ENCODER_B) == LOW)
    {
      // Left encoder count -1
      leftCount--;
    }
    // And the second wave is behind
    else
    {
      // Left encoder count +1
      leftCount++;
    }
  }
} 


////US Sensor Code
//void read_sensor(){
//  anVolt1 = analogRead(US_PIN4);
//  //anVolt2 = analogRead(US_PIN5);
//  mm = anVolt1 * 5;
//  inches = mm/25.4;
//}

//void print_range(){
//  Serial.print(“S1”);
//  Serial.print(“=”);
//  Serial.print(mm);
//  Serial.print(” “);
//  Serial.println(inches);
//}  
