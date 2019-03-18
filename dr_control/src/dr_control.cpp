#include "ros/ros.h"
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <serial/serial.h>
#include <string>

class LowPassFilter {
private:
  double ts_;
  double tau_;
  double pre_output_;
  double output_;
  double input_;
  
public:
  LowPassFilter() {}
  LowPassFilter(double ts, double tau):ts_(ts), tau_(tau) {}

  double filter(double input) {
    input_ = input;
    output_ = ( tau_ * pre_output_ + ts_ * input_ ) /(tau_ + ts_);
    pre_output_ = output_;

    return output_;
  }

  void setTau(double tau) {
    tau_ = tau;
  }

  void setTs(double ts) {
    ts_ = ts;
  }
};

class DRControl {
private:
  ros::NodeHandle nh;
  ros::NodeHandle nh_;
  ros::Subscriber twist_sub_;

  serial::Serial dr_serial_;
  std::string port_;
  int baud_;
  int timeout_;

  bool reverse_x_direction_;

  LowPassFilter linear_lpf_;
  double linear_ts_;
  double linear_tau_;
  bool linear_LPF_on_;

  LowPassFilter angular_lpf_;
  double angular_ts_;
  double angular_tau_;
  bool angular_LPF_on_;

public:
  DRControl() {
    nh_ = ros::NodeHandle("~");
    nh_.param<std::string>("port", port_, "/dev/ttyUSB1");
    nh_.param("baudrate", baud_, 115200);
    nh_.param("timeout", timeout_, 1000);

    nh_.param("reverse_x_direction", reverse_x_direction_, false);

    nh_.param("linear_ts", linear_ts_, 0.1);
    nh_.param("linear_tau", linear_tau_, 0.9);
    nh_.param("linear_LPF_on", linear_LPF_on_, true);

    nh_.param("angular_ts", angular_ts_, 0.1);
    nh_.param("angular_tau", angular_tau_, 0.9);
    nh_.param("angular_LPF_on", angular_LPF_on_, true);

    linear_lpf_ = LowPassFilter(linear_ts_, linear_tau_);
    angular_lpf_ = LowPassFilter(angular_ts_, angular_tau_);

    twist_sub_ = nh.subscribe<geometry_msgs::Twist> ("cmd_vel", 10, &DRControl::twistCallback, this);

    dr_serial_.setPort(port_);
    dr_serial_.setBaudrate(baud_);
    serial::Timeout timeout = serial::Timeout::simpleTimeout(timeout_);
    dr_serial_.setTimeout(timeout);

    try {
      dr_serial_.open();
    }
    catch(serial::IOException ex) {
      ROS_ERROR("Serial Port Closed. Turn off the node.");
      exit(-1);
    }
  }

  void twistCallback(const geometry_msgs::Twist::ConstPtr& twist_msg) {
    std::string command_string;

    double linear_speed = linear_lpf_.filter(twist_msg->linear.x);
    double angular_speed = angular_lpf_.filter(twist_msg->angular.z);

    if(reverse_x_direction_) linear_speed *= -1;

    command_string = "$CVW," + std::to_string(int(linear_speed*1000))
                       + "," + std::to_string(int(angular_speed*1000)) + "\r\n";
    dr_serial_.write(command_string);

    ROS_INFO_STREAM("$CVW," + std::to_string(int(linear_speed*1000))
                       + "," + std::to_string(int(angular_speed*1000)) );
  }
};

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "dr_control");
  DRControl dr_control;

  ros::spin();

  return 0;
}
