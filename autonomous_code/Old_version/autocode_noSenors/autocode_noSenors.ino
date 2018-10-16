///Libraries
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
}

void loop()
{
   // Read any serial input given'
  if (Serial.available() > 0 ) {
    leftCount = 0;
    rightCount = 0;
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

  if(rightCount>=encoder_counts || leftCount>=encoder_counts){                     

    
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
