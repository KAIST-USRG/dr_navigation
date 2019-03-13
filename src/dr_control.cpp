#include "ros/ros.h"
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <serial/serial.h>
#include <string>

class DRControl {
private:
  ros::NodeHandle nh;
  ros::NodeHandle nh_;
  ros::Subscriber twist_sub_;

  serial::Serial dr_serial_;
  std::string port_;
  int baud_;
  int timeout_;

public:
  DRControl() {
    nh_ = ros::NodeHandle("~");
    nh_.param<std::string>("port", port_, "/dev/ttyACM0");
    nh_.param("baudrate", baud_, 115200);
    nh_.param("timeout", timeout_, 1000);

    twist_sub_ = nh.subscribe<geometry_msgs::Twist> ("cmd_vel", 10, &DRControl::twistCallback, this);

    dr_serial_.setPort(port_);
    dr_serial_.setBaudrate(baud_);
    serial::Timeout timeout = serial::Timeout::simpleTimeout(timeout_);
    dr_serial_.setTimeout(timeout);
  }

  void twistCallback(const geometry_msgs::Twist::ConstPtr& twist_msg) {
    std::string command_string;
    //command_string = "$CVW,200,0,\r\n";
    command_string = "$CVW," + std::to_string(int(twist_msg->linear.x)*1000)
                       + "," + std::to_string(int(twist_msg->angular.z)*1000) + "\r\n";
    ROS_INFO_STREAM(command_string);
    //dr_serial_.write(command_string);
  }
};

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "dr_control");
  DRControl dr_control;

  ros::spin();

  return 0;
}
