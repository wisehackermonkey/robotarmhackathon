#include <ros.h> //http://wiki.ros.org/rosserial_arduino/Tutorials/Arduino%20IDE%20Setup
#include <geometry_msgs/Pose.h>
#include <AccelStepper.h>

ros::NodeHandle node_handle;
geometry_msgs::Pose move_axis_relative;
geometry_msgs::Pose joints;

ros::Publisher arduino_joint_publisher("arduino_joint_publisher", &joints);

// Connections to driver
#define dirPin1  5  // Direction for axis 1
#define stepPin1  2// Step for axis 1
#define dirPin2  6  // Direction for axis 2
#define stepPin2  3 // Step for axis 2
#define dirPin3  7  // Direction for axis 3
#define stepPin3  4 // Step for axis 3
#define LED_PIN  13
#define LIMIT_SWITCH_PIN1  9
#define LIMIT_SWITCH_PIN2  10
#define LIMIT_SWITCH_PIN3  11

int axis_1_cmd = 0;
int axis_2_cmd = 0;
int axis_3_cmd = 0;

bool begin_move = false;
bool begin_homing = false;
bool stopFlag = false;

AccelStepper axis1(1, stepPin1, dirPin1);
AccelStepper axis2(1, stepPin2, dirPin2);
AccelStepper axis3(1, stepPin3, dirPin3);

void move_axis_callback(const geometry_msgs::Pose& move_axis) {
  axis_1_cmd = move_axis.position.x;
  axis_2_cmd = move_axis.position.y;
  axis_3_cmd = move_axis.position.z;

  axis1.moveTo(axis_1_cmd);
  axis2.moveTo(axis_2_cmd);
  axis3.moveTo(axis_3_cmd);
  
  if ( (axis_1_cmd == 0) && (axis_2_cmd == 0) && (axis_3_cmd == 0)){
    stopFlag = true;
  }
  if (axis_1_cmd == 999)
    begin_homing = true;
  else
    begin_move = true;
  
}

ros::Subscriber<geometry_msgs::Pose> arduino_sub1("move_axis_absolute", &move_axis_callback);

void setup() {

  node_handle.initNode();
  node_handle.advertise(arduino_joint_publisher);
  node_handle.subscribe(arduino_sub1);

  pinMode(LED_PIN, OUTPUT);
  pinMode(LIMIT_SWITCH_PIN1, INPUT_PULLUP);
  pinMode(LIMIT_SWITCH_PIN2, INPUT_PULLUP);
  pinMode(LIMIT_SWITCH_PIN3, INPUT_PULLUP);

  // set the speed and acceleration rates for the stepper motor
  axis1.setMaxSpeed(500.0);
  axis1.setAcceleration(1000.0);
  
  axis2.setMaxSpeed(500.0);
  axis2.setAcceleration(1000.0);

  axis3.setMaxSpeed(500.0);
  axis3.setAcceleration(1000.0);

}
void loop() {

  if (begin_move==true){
      begin_move = false;

      while(axis1.isRunning() || axis2.isRunning() || axis3.isRunning() ){

            axis1.run();
            axis2.run();
            axis3.run();
            node_handle.spinOnce();
            
            if (stopFlag==true) //add stop command, axis limits, timeout here
            {
              axis1.stop();
              axis2.stop();
              axis3.stop();
              stopFlag = false;
              begin_move = false;
              }       
      }
 }

    if (begin_homing){
        begin_homing = false;

          while (digitalRead(LIMIT_SWITCH_PIN1)) {  // Make the Stepper move CCW until the switch is activated   
              axis1.moveTo(100000);  // Set the position to move to
              axis1.run();  // Start moving the stepper
              delay(5);
              node_handle.spinOnce();
          }
          axis1.setCurrentPosition(0);
          axis1.runToNewPosition(-3600);
          axis1.setCurrentPosition(0);

          while (digitalRead(LIMIT_SWITCH_PIN2)) {  // Make the Stepper move CCW until the switch is activated   
              axis2.moveTo(-100000);  // Set the position to move to
              axis2.run();  // Start moving the stepper
              delay(5);
              node_handle.spinOnce();
          }
          axis2.setCurrentPosition(0);
          axis2.runToNewPosition(2300);
          axis2.setCurrentPosition(0);

          while (digitalRead(LIMIT_SWITCH_PIN3)) {  // Make the Stepper move CCW until the switch is activated   
              axis3.moveTo(100000);  // Set the position to move to
              axis3.run();  // Start moving the stepper
              delay(5);
              node_handle.spinOnce();
          }
          axis3.setCurrentPosition(0);  
          axis3.runToNewPosition(-800);
          axis3.setCurrentPosition(0);
    }
    
  joints.position.x=axis1.currentPosition();
  joints.position.y=axis2.currentPosition();
  joints.position.z=axis3.currentPosition();
  arduino_joint_publisher.publish( &joints );
  node_handle.spinOnce();
  delay(1);
}
