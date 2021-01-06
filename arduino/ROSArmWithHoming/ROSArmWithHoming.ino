#include <ros.h> //http://wiki.ros.org/rosserial_arduino/Tutorials/Arduino%20IDE%20Setup
#include <std_msgs/String.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int16.h>



#include <FlexyStepper.h> //https://github.com/Stan-Reifel/SpeedyStepper/blob/master/Documentation.pdf

ros::NodeHandle node_handle;

std_msgs::Float64MultiArray move_axis_relative;
std_msgs::Float64MultiArray joints;
std_msgs::Int16MultiArray bump_axis;
std_msgs::Bool home_axis;
std_msgs::Int16 switch1;
std_msgs::Int16 switch2;
std_msgs::Int16 switch3;

// Connections to driver
const int dirPin1 = 10;  // Direction for axis 1
const int stepPin1 = 9;// Step for axis 1
const int dirPin2 = 7;  // Direction for axis 2
const int stepPin2 = 6; // Step for axis 2
const int dirPin3 = 4;  // Direction for axis 3
const int stepPin3 = 3; // Step for axis 3

const int LED_PIN = 13;
const int LIMIT_SWITCH_PIN1 = 1;
const int LIMIT_SWITCH_PIN2 = 1;
const int LIMIT_SWITCH_PIN3 = 1;

float axis_1_pos = 0;
float axis_2_pos = 0;
float axis_3_pos = 0;
bool   begin_relative_move = false;

int axis_1_dir = 0;
int axis_2_dir = 0;
int axis_3_dir = 0;
bool begin_bump = false;
bool begin_homing = false;

FlexyStepper axis1;
FlexyStepper axis2;
FlexyStepper axis3;

ros::Publisher arduino_joint_publisher("arduino_joint_publisher", &joints);
ros::Publisher switch1_publisher("arduino_switch1", &switch1);
ros::Publisher switch2_publisher("arduino_switch2", &switch2);
ros::Publisher switch3_publisher("arduino_switch3", &switch3);

void move_axis_relative(const std_msgs::Float64MultiArray &move_axis_relative) {
  axis_1_pos = move_axis_relative.data[0];
  axis_2_pos = move_axis_relative.data[1];
  axis_3_pos = move_axis_relative.data[2];

  axis1.setupRelativeMoveInRevolutions(axis_1_pos);
  axis2.setupRelativeMoveInRevolutions(axis_2_pos);
  axis3.setupRelativeMoveInRevolutions(axis_3_pos);
  
  begin_relative_move = true;

}

void bump_axis(const std_msgs::Int16MultiArray &bump_axis) {
  //can be -1, 0 or 1 (CW, stop, CCW)
  axis_1_dir = bump_axis.data[0];
  axis_2_dir = bump_axis.data[1];
  axis_3_dir = bump_axis.data[2];

  if( (axis_1_dir==0) && (axis_2_dir==0) && (axis_3_dir==0)){
    stopFlag=true;
    begin_bump = false;
  }
  
  begin_bump = true;

}

void home_axis(const std_msgs::Bool &home_axis) {

  begin_homing = home_axis.data;

}

ros::Subscriber<std_msgs::Float64MultiArray> arduino_sub1("move_axis_relative", &move_axis_relative);
ros::Subscriber<std_msgs::Int16MultiArray> arduino_sub2("bump_axis", &bump_axis);
ros::Subscriber<std_msgs::Bool> arduino_sub3("home_axis", &home_axis);


void setup() {

  node_handle.initNode();
  node_handle.advertise(arduino_publisher);
  joints.data = (float*)malloc(sizeof(float) * 3);
  joints.data_length = 3;
  
  // Setup the steppers with speedy stepper lib
  axis1.connectToPins(stepPin1, dirPin1);
  axis2.connectToPins(stepPin2, dirPin2);
  axis3.connectToPins(stepPin3, dirPin3);

  pinMode(LED_PIN, OUTPUT);
  pinMode(LIMIT_SWITCH_PIN1, INPUT_PULLUP);
  pinMode(LIMIT_SWITCH_PIN2, INPUT_PULLUP);
  pinMode(LIMIT_SWITCH_PIN3, INPUT_PULLUP);

  // set the speed and acceleration rates for the stepper motor
  axis1.setStepsPerRevolution(6400);
  axis1.setSpeedInRevolutionsPerSecond(2.0);
  axis1.setAccelerationInRevolutionsPerSecondPerSecond(1.0);
  
  axis2.setStepsPerRevolution(6400);
  axis2.setSpeedInRevolutionsPerSecond(2.0);
  axis2.setAccelerationInRevolutionsPerSecondPerSecond(1.0);

  axis3.setStepsPerRevolution(6400);
  axis3.setSpeedInRevolutionsPerSecond(2.0);
  axis3.setAccelerationInRevolutionsPerSecondPerSecond(1.0);

  Serial.begin(9600);

}
void loop() {

  node_handle.spinOnce();

  if (begin_relative_move){
      while(!axis1.motionComplete() || !axis2.motionComplete() || !axis3.motionComplete() ){
            axis1.processMovement();
            axis2.processMovement();
            axis3.processMovement();
  
            joints.data[0]=axis1.getCurrentPositionInRevolutions()*8/20;
            joints.data[1]=axis2.getCurrentPositionInRevolutions()*10/44;
            joints.data[2]=axis3.getCurrentPositionInRevolutions()*10/44;

            arduino_joint_publisher.publish( &joints );
      }
      begin_relative_move = 0;
    }

  if (begin_bump){

      axis1.setupRelativeMoveInRevolutions(axis_1_dir*1000);
      axis2.setupRelativeMoveInRevolutions(axis_2_dir*1000);
      axis3.setupRelativeMoveInRevolutions(axis_3_dir*1000);
      
      while(!axis1.motionComplete() || !axis2.motionComplete() || !axis3.motionComplete() )
      {
            axis1.processMovement();
            axis2.processMovement();
            axis3.processMovement();
  
            joints.data[0]=axis1.getCurrentPositionInRevolutions()*8/20;
            joints.data[1]=axis2.getCurrentPositionInRevolutions()*10/44;
            joints.data[2]=axis3.getCurrentPositionInRevolutions()*10/44;
            arduino_joint_publisher.publish( &joints );

            //will change move command if subscriber updates dir values
            axis1.setupRelativeMoveInRevolutions(axis_1_dir*1000);
            axis2.setupRelativeMoveInRevolutions(axis_2_dir*1000);
            axis3.setupRelativeMoveInRevolutions(axis_3_dir*1000);

            if (stopFlag==True) //add stop command, axis limits, timeout here
            {
            axis1.setupStop();
            axis2.setupStop();
            axis3.setupStop();
            begin_bump = false;
            }
            
      }
    }

    if (begin_homing){

        const float homingSpeed = 1.0;
        const float maxHomingRotation = 10;
        const int directionTowardHome1 = -1;// direction to move toward limit switch: 1 goes positive direction, -1 backward
        const int directionTowardHome2 = -1;
        const int directionTowardHome3 = -1;  

        if(axis1.moveToHomeInRevolutions(directionTowardHome1, homingSpeed, maxHomingRotation, LIMIT_SWITCH_PIN1) != true)
        {
          while(true)
          {
            digitalWrite(LED_PIN, HIGH);
            delay(50);
            digitalWrite(LED_PIN, LOW);
            delay(50);
          }
        }

        if(axis2.moveToHomeInRevolutions(directionTowardHome2, homingSpeed, maxHomingRotation, LIMIT_SWITCH_PIN2) != true)
        {
          while(true)
          {
            digitalWrite(LED_PIN, HIGH);
            delay(50);
            digitalWrite(LED_PIN, LOW);
            delay(50);
          }
        }
        if(axis3.moveToHomeInRevolutions(directionTowardHome3, homingSpeed, maxHomingRotation, LIMIT_SWITCH_PIN3) != true)
        {
          while(true)
          {
            digitalWrite(LED_PIN, HIGH);
            delay(50);
            digitalWrite(LED_PIN, LOW);
            delay(50);
          }
        }

        begin_homing= false;
      
    }

  switch1 = digitalRead(LIMIT_SWITCH_PIN1);
  switch2 = digitalRead(LIMIT_SWITCH_PIN2);
  switch3 = digitalRead(LIMIT_SWITCH_PIN3);
  switch1_publisher.publish( &switch1 );
  switch2_publisher.publish( &switch2 );
  switch3_publisher.publish( &switch3 );
  
  delay(100);
}
