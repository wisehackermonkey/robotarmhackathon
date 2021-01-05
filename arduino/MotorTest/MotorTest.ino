#include <SpeedyStepper.h> //https://github.com/Stan-Reifel/SpeedyStepper/blob/master/Documentation.pdf


// Connections to driver
const int dirPin1 = 10;  // Direction for axis 1
const int stepPin1 = 9;// Step for axis 1

const int dirPin2 = 7;  // Direction for axis 2
const int stepPin2 = 6; // Step for axis 2

const int dirPin3 = 4;  // Direction for axis 3
const int stepPin3 = 3; // Step for axis 3

float axis_1_pos = 0;
float axis_2_pos = 0;
float axis_3_pos = 0;

SpeedyStepper axis1;
SpeedyStepper axis2;
SpeedyStepper axis3;

void setup() {
  
  // Setup the steppers with speedy stepper lib
  axis1.connectToPins(stepPin1, dirPin1);
  axis2.connectToPins(stepPin2, dirPin2);
  axis3.connectToPins(stepPin3, dirPin3);

  // set the speed and acceleration rates for the stepper motor
  axis1.setStepsPerRevolution(200);
  axis1.setSpeedInRevolutionsPerSecond(1.0);
  axis1.setAccelerationInRevolutionsPerSecondPerSecond(1.0);
  
  axis2.setStepsPerRevolution(200);
  axis2.setSpeedInRevolutionsPerSecond(1.0);
  axis2.setAccelerationInRevolutionsPerSecondPerSecond(1.0);

  axis3.setStepsPerRevolution(200);
  axis3.setSpeedInRevolutionsPerSecond(1.0);
  axis3.setAccelerationInRevolutionsPerSecondPerSecond(1.0);

  Serial.begin(9600);

}
void loop() {

  axis1.setupRelativeMoveInRevolutions(0.5);
  axis2.setupRelativeMoveInRevolutions(0.5);
  axis3.setupRelativeMoveInRevolutions(0.5);

  while((!axis1.motionComplete()) || (!axis2.motionComplete() || (!axis2.motionComplete()))
    {
      axis1.processMovement();
      axis2.processMovement();
      axis3.processMovement();

    float joint1 = axis1.getCurrentPositionInRevolutions()
    float joint2 = axis1.getCurrentPositionInRevolutions()
    float joint3 = axis1.getCurrentPositionInRevolutions()

    Serial.print("Axis 1: ");
    Serial.print(joint1);
    Serial.print("\t");
    
    Serial.print("Axis 2: ");
    Serial.print(joint2);
    Serial.print("\t");

    Serial.print("Axis 3: ");
    Serial.println(joint3);
    }

  delay(1000);


  axis1.setupRelativeMoveInRevolutions(-0.5);
  axis2.setupRelativeMoveInRevolutions(-0.5);
  axis3.setupRelativeMoveInRevolutions(-0.5);

  while((!axis1.motionComplete()) || (!axis2.motionComplete() || (!axis2.motionComplete()))
    {
      axis1.processMovement();
      axis2.processMovement();
      axis3.processMovement();

    float joint1 = axis1.getCurrentPositionInRevolutions()
    float joint2 = axis1.getCurrentPositionInRevolutions()
    float joint3 = axis1.getCurrentPositionInRevolutions()

    Serial.print("Axis 1: ");
    Serial.print(joint1);
    Serial.print("\t");
    
    Serial.print("Axis 2: ");
    Serial.print(joint2);
    Serial.print("\t");

    Serial.print("Axis 3: ");
    Serial.println(joint3);
    }

  delay(1000);
}
