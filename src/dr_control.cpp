#include "ros/ros.h"
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <serial/serial.h>

#define INDOOR false
#define OUTDOOR !(INDOOR)

#define CMD_MIN 1100
#define CMD_MAX 1940
#define CMD_MID 1520

serial::Serial ser;

bool ESTOP_flg = false;

double Robot_Wheel_base = 0.47; // [m]
double Robot_wheel_Radius = 0.2; // [m]

void writeControlcmd(bool _estop, double _left_vel, double _right_vel) {
  static int i = 0;
  std::string UART_MSG_LEFT = "000000";
  std::string UART_MSG_RIGHT = "000000";

  if(_estop == true) {
    UART_MSG_LEFT[0] = 'U';
    UART_MSG_LEFT[1] = 'S';
    UART_MSG_LEFT[2] = 0x01; // left motor
    UART_MSG_LEFT[3] = 0x03; // direction : forward (2: backward, 3: break, 4: torque off)
    UART_MSG_LEFT[4] = int(0); // velocity
    UART_MSG_LEFT[5] = 0x00; // checksum(now unavailable, set 0)

    ser.write(UART_MSG_LEFT);
  }
  else {
    UART_MSG_LEFT[0] = 'U';
    UART_MSG_LEFT[1] = 'S';
    UART_MSG_LEFT[2] = 0x00; // left motor
    UART_MSG_LEFT[3] = int(_left_vel*10); // direction : forward (2: backward, 3: break, 4: torque off)
    UART_MSG_LEFT[4] = int(_right_vel*10); // velocity
    UART_MSG_LEFT[5] = 0x00; // checksum(now unavailable, set 0)

    ser.write(UART_MSG_LEFT);
  }
}


void poseCallback(const geometry_msgs::Twist::ConstPtr& twist_msg) {
  int default_vel = 5;
  double rotate_speed = 3;

  double angular_vel = twist_msg->angular.z;
  double linear_vel = twist_msg->linear.x;

  double right_vel = 0;
  double left_vel = 0;

  double right_motor_speed = 0;
  double left_motor_speed = 0;
  
  double RperL_ratio = 0.0;

  if(linear_vel < 0) {
    ROS_INFO("move rear");
    writeControlcmd(false, 0.1, rotate_speed);
  }
  else if(linear_vel == 0. && angular_vel == 0.) {
    ROS_INFO("E-stop");
    writeControlcmd(true, 0.0, 0.0);
    writeControlcmd(true, 0.0, 0.0);
    writeControlcmd(true, 0.0, 0.0);
  }
  else if(linear_vel == 0. && angular_vel != 0.) { // rotate
    if(angular_vel < 0.) {
      ROS_INFO("right turn");
      writeControlcmd(false, 0.1, 2);
    }
    else {
      ROS_INFO("left turn");
      writeControlcmd(false, 2, 0.1);
    }
  }
  else {
    right_vel = (2*linear_vel + angular_vel*0.8*Robot_Wheel_base)/(2.0);
    left_vel = (2*linear_vel - angular_vel*0.8*Robot_Wheel_base)/(2.0);
    //right_vel = (2*linear_vel + angular_vel*Robot_Wheel_base)/(2.0);
    //left_vel = (2*linear_vel - angular_vel*Robot_Wheel_base)/(2.0);

    right_motor_speed = right_vel / Robot_wheel_Radius;
    left_motor_speed = left_vel / Robot_wheel_Radius;

    RperL_ratio = right_motor_speed / left_motor_speed;

    if(RperL_ratio > 1) { //left_turn 
        right_motor_speed = 2.0;
        left_motor_speed = 0.1;
    }
    else if(RperL_ratio < 0.1) { // right turn
        right_motor_speed = 0.1;
        left_motor_speed = 2.0;
    }

    ROS_INFO_STREAM("left motor:" << left_motor_speed << " right motor:" << right_motor_speed);
    writeControlcmd(false, right_motor_speed, left_motor_speed);
  }
}

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "dr_control");
  ros::NodeHandle nh;
  ros::Subscriber twist_sub = nh.subscribe("/cmd_vel", 1, poseCallback);
  ros::Rate loop_rate(10);
  
  ser.setPort("/dev/ttyUSB0");
  ser.setBaudrate(9600);
  serial::Timeout to = serial::Timeout::simpleTimeout(1000);
  ser.setTimeout(to);
  ser.open();

  ROS_INFO("BASE CONTROL PKG Started...");

  while(ros::ok()) {
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
