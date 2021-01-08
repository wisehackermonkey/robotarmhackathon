#include <ros.h> //http://wiki.ros.org/rosserial_arduino/Tutorials/Arduino%20IDE%20Setup
#include <std_msgs/Float64MultiArray.h>
//#include <std_msgs/Int16MultiArray.h>
#include <FlexyStepper.h> //https://github.com/Stan-Reifel/SpeedyStepper/blob/master/Documentation.pdf

ros::NodeHandle node_handle;

std_msgs::Float64MultiArray move_axis_relative;
std_msgs::Float64MultiArray joints;
//std_msgs::Int16MultiArray bump_axis;

// Connections to driver
const int dirPin1 = 5;  // Direction for axis 1
const int stepPin1 = 2;// Step for axis 1
const int dirPin2 = 6;  // Direction for axis 2
const int stepPin2 = 3; // Step for axis 2
const int dirPin3 = 7;  // Direction for axis 3
const int stepPin3 = 4; // Step for axis 3

const int LED_PIN = 13;
const int LIMIT_SWITCH_PIN1 = 9;
const int LIMIT_SWITCH_PIN2 = 10;
const int LIMIT_SWITCH_PIN3 = 11;

float axis_1_pos = 0;
float axis_2_pos = 0;
float axis_3_pos = 0;
bool  begin_relative_move = false;

int axis_1_dir = 0;
int axis_2_dir = 0;
int axis_3_dir = 0;
//bool begin_bump = false;
bool begin_homing = false;

bool stopFlag = false;

FlexyStepper axis1;
FlexyStepper axis2;
FlexyStepper axis3;

ros::Publisher arduino_joint_publisher("arduino_joint_publisher", &joints);

void move_axis_callback(const std_msgs::Float64MultiArray &move_axis_relative) {
  axis_1_pos = move_axis_relative.data[0];
  axis_2_pos = move_axis_relative.data[1];
  axis_3_pos = move_axis_relative.data[2];

  joints.data[0]=axis_1_pos;
  joints.data[1]=axis_1_pos;
  joints.data[2]=axis_1_pos;
  arduino_joint_publisher.publish( &joints );
  
  joints.data[0]=axis1.getCurrentPositionInRevolutions()*8/20;
  joints.data[1]=axis2.getCurrentPositionInRevolutions()*10/44;
  joints.data[2]=axis3.getCurrentPositionInRevolutions()*10/44;
  arduino_joint_publisher.publish( &joints );
  begin_relative_move = true;

}

//void bump_axis_callback(const std_msgs::Int16MultiArray &bump_axis) {
//  //can be -1, 0 or 1 (CW, stop, CCW)
//  axis_1_dir = bump_axis.data[0];
//  axis_2_dir = bump_axis.data[1];
//  axis_3_dir = bump_axis.data[2];
//  begin_homing = bump_axis.data[3];
//
//  if( (axis_1_dir==0) && (axis_2_dir==0) && (axis_3_dir==0)){
//    stopFlag=true;
//    begin_bump = false;
//  }
//  
//  begin_bump = true;
//
//}


ros::Subscriber<std_msgs::Float64MultiArray> arduino_sub1("move_axis_relative", &move_axis_callback);
//ros::Subscriber<std_msgs::Int16MultiArray> arduino_sub2("bump_axis", &bump_axis_callback);

void setup() {

  node_handle.initNode();
  node_handle.advertise(arduino_joint_publisher);
  node_handle.subscribe(arduino_sub1);
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
  axis1.setStepsPerRevolution(3200);
  axis1.setSpeedInRevolutionsPerSecond(2.0);
  axis1.setAccelerationInRevolutionsPerSecondPerSecond(1.0);
  
  axis2.setStepsPerRevolution(3200);
  axis2.setSpeedInRevolutionsPerSecond(2.0);
  axis2.setAccelerationInRevolutionsPerSecondPerSecond(1.0);

  axis3.setStepsPerRevolution(3200);
  axis3.setSpeedInRevolutionsPerSecond(2.0);
  axis3.setAccelerationInRevolutionsPerSecondPerSecond(1.0);

}
void loop() {


  if (begin_relative_move==true){

      axis1.setTargetPositionRelativeInRevolutions(axis_1_pos);
      axis2.setTargetPositionRelativeInRevolutions(axis_2_pos);
      axis3.setTargetPositionRelativeInRevolutions(axis_3_pos);
    
      while(!axis1.motionComplete() || !axis2.motionComplete() || !axis3.motionComplete() ){
            axis1.processMovement();
            axis2.processMovement();
            axis3.processMovement();
  
            joints.data[0]=axis1.getCurrentPositionInRevolutions()*8/20;
            joints.data[1]=axis2.getCurrentPositionInRevolutions()*10/44;
            joints.data[2]=axis3.getCurrentPositionInRevolutions()*10/44;
            arduino_joint_publisher.publish( &joints );
      }
      begin_relative_move = false;
    }

//  if (begin_bump){
//
//      axis1.setTargetPositionRelativeInRevolutions(axis_1_dir*1000);
//      axis2.setTargetPositionRelativeInRevolutions(axis_2_dir*1000);
//      axis3.setTargetPositionRelativeInRevolutions(axis_3_dir*1000);
//      
//      while(!axis1.motionComplete() || !axis2.motionComplete() || !axis3.motionComplete() )
//      {
//            axis1.processMovement();
//            axis2.processMovement();
//            axis3.processMovement();
//  
//            joints.data[0]=axis1.getCurrentPositionInRevolutions()*8/20;
//            joints.data[1]=axis2.getCurrentPositionInRevolutions()*10/44;
//            joints.data[2]=axis3.getCurrentPositionInRevolutions()*10/44;
//            arduino_joint_publisher.publish( &joints );
//
//            //will change move command if subscriber updates dir values
//            axis1.setTargetPositionRelativeInRevolutions(axis_1_dir*1000);
//            axis2.setTargetPositionRelativeInRevolutions(axis_2_dir*1000);
//            axis3.setTargetPositionRelativeInRevolutions(axis_3_dir*1000);
//
//            if (stopFlag==true) //add stop command, axis limits, timeout here
//            {
//            axis1.setTargetPositionToStop();
//            axis2.setTargetPositionToStop();
//            axis3.setTargetPositionToStop();
//            axis1.processMovement();
//            axis2.processMovement();
//            axis3.processMovement();
//            begin_bump = false;
//            }
//            
//      }
//    }

    if (begin_homing){

        const float homingSpeed = 0.25;
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
  node_handle.spinOnce();
  delay(100);
}
