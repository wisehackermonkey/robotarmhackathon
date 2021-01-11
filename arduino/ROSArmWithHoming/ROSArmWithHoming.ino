#include <ros.h> //http://wiki.ros.org/rosserial_arduino/Tutorials/Arduino%20IDE%20Setup
#include <FlexyStepper.h> //https://github.com/Stan-Reifel/SpeedyStepper/blob/master/Documentation.pdf
#include <geometry_msgs/Pose.h>

ros::NodeHandle node_handle;

geometry_msgs::Pose move_axis_relative;
geometry_msgs::Pose joints;
geometry_msgs::Pose bump_axis;


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

int axis_1_cmd = 0;
int axis_2_cmd = 0;
int axis_3_cmd = 0;
bool  begin_relative_move = false;

int axis_1_dir = 0;
int axis_2_dir = 0;
int axis_3_dir = 0;
bool begin_bump = false;
bool begin_homing = false;

bool stopFlag = false;

FlexyStepper axis1;
FlexyStepper axis2;
FlexyStepper axis3;


ros::Publisher arduino_joint_publisher("arduino_joint_publisher", &joints);

void move_axis_callback(const geometry_msgs::Pose& move_axis_relative) {
  axis_1_cmd = move_axis_relative.position.x;
  axis_2_cmd = move_axis_relative.position.y;
  axis_3_cmd = move_axis_relative.position.z;

  joints.position.x=axis_1_cmd;
  joints.position.y=axis_2_cmd;
  joints.position.z=axis_3_cmd;
  arduino_joint_publisher.publish( &joints );

  delay(2000);
  
  joints.position.x=axis1.getCurrentPositionInSteps();
  joints.position.y=axis2.getCurrentPositionInSteps();
  joints.position.z=axis3.getCurrentPositionInSteps();
  arduino_joint_publisher.publish( &joints );

  delay(2000);
  begin_relative_move = true;


  if ( (axis_1_cmd == 0) && (axis_2_cmd == 0) && (axis_3_cmd == 0)){
    stopFlag = true;
  }
  else{
    stopFlag = false;
  }

}

void bump_axis_callback(const geometry_msgs::Pose& bump_axis) {
  //can be -1, 0 or 1 (CW, stop, CCW)
  axis_1_dir = int(bump_axis.position.x);
  axis_2_dir = int(bump_axis.position.y);
  axis_3_dir = int(bump_axis.position.z);
  begin_homing = bool(bump_axis.orientation.w);

//  if( (axis_1_dir==0) && (axis_2_dir==0) && (axis_3_dir==0)){
//    stopFlag=true;
//    begin_bump = false;
//  }
  
  begin_bump = true;

}


ros::Subscriber<geometry_msgs::Pose> arduino_sub1("move_axis_relative", &move_axis_callback);
ros::Subscriber<geometry_msgs::Pose> arduino_sub2("bump_axis", &bump_axis_callback);

void setup() {

  node_handle.initNode();
  node_handle.advertise(arduino_joint_publisher);
  node_handle.subscribe(arduino_sub1);
  node_handle.subscribe(arduino_sub2);


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

  joints.position.x=axis1.getCurrentPositionInSteps();
  joints.position.y=axis2.getCurrentPositionInSteps();
  joints.position.z=axis3.getCurrentPositionInSteps();
  arduino_joint_publisher.publish( &joints );

  if (begin_relative_move==true){
      begin_relative_move = false;
      axis1.setTargetPositionRelativeInSteps(axis_1_cmd);
      axis2.setTargetPositionRelativeInSteps(axis_2_cmd);
      axis3.setTargetPositionRelativeInSteps(axis_3_cmd);
    
      while(!axis1.motionComplete() || !axis2.motionComplete() || !axis3.motionComplete() ){
            axis1.processMovement();
            axis2.processMovement();
            axis3.processMovement();

            joints.position.x=axis1.getCurrentPositionInSteps();
            joints.position.y=axis2.getCurrentPositionInSteps();
            joints.position.z=axis3.getCurrentPositionInSteps();
            arduino_joint_publisher.publish( &joints );
            node_handle.spinOnce();
            
            if (stopFlag==true) //add stop command, axis limits, timeout here
            {
              axis1.setTargetPositionToStop();
              axis2.setTargetPositionToStop();
              axis3.setTargetPositionToStop();
              stopFlag = false;
              begin_relative_move = false;
              }       
      }

   if ( axis1.motionComplete() && axis2.motionComplete() && axis3.motionComplete() ){
      begin_relative_move = false;
      joints.position.x=55.55;
      joints.position.y=55.55;
      joints.position.z=55.55;
      arduino_joint_publisher.publish( &joints );

      delay(5000);
    }

 }

  if (begin_bump){
      begin_bump = false;
      axis1.setTargetPositionRelativeInSteps(axis_1_dir*500);
      axis2.setTargetPositionRelativeInSteps(axis_2_dir*500);
      axis3.setTargetPositionRelativeInSteps(axis_3_dir*500);

      joints.position.x=axis_1_dir;
      joints.position.y=axis_2_dir;
      joints.position.z=axis_3_dir;
      arduino_joint_publisher.publish( &joints );
      delay(3000);
      
      while(!axis1.motionComplete() || !axis2.motionComplete() || !axis3.motionComplete() )
      {
            axis1.processMovement();
            axis2.processMovement();
            axis3.processMovement();
  
            joints.position.x=axis1.getCurrentPositionInSteps();
            joints.position.y=axis2.getCurrentPositionInSteps();
            joints.position.z=axis3.getCurrentPositionInSteps();
            arduino_joint_publisher.publish( &joints );
            node_handle.spinOnce();

            //will change move command if subscriber updates dir values
            axis1.setTargetPositionRelativeInRevolutions(axis_1_dir*500);
            axis2.setTargetPositionRelativeInRevolutions(axis_2_dir*500);
            axis3.setTargetPositionRelativeInRevolutions(axis_3_dir*500);

            if (stopFlag==true) //add stop command, axis limits, timeout here
            {
            axis1.setTargetPositionToStop();
            axis2.setTargetPositionToStop();
            axis3.setTargetPositionToStop();
            axis1.processMovement();
            axis2.processMovement();
            axis3.processMovement();
            stopFlag=false;
            }
      }
    }

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
  delay(1);
}
