/*
 * File: bluerov/src/controller.cpp
 * Author: Josh Villbrandt <josh@javconcepts.com>
 * Date: April 2015
 * Description: Converts ROS messages into mavlink messages.
 */

#include <ros/ros.h>
#include <ros/console.h>
#include <std_msgs/Bool.h>
#include <bluerov/Thruster.h>

class Controller
{
  public:
    Controller();
    void spin();

  private:
    void stopAllThrusters();
    void thrusterEnableCallback(const std_msgs::Bool::ConstPtr& msg);
    void thrusterCallback(const bluerov::Thruster::ConstPtr& thruster);

    ros::NodeHandle nh;
    ros::Subscriber thruster_enable_sub;
    ros::Subscriber thruster_sub;

    double imu_pub_rate, health_pub_rate, watchdog_timeout;
    int esc_pwm_min, esc_pwm_max, num_thrusters;
    bool enabled;
};

Controller::Controller() {
  // load parameters
  ros::NodeHandle nh_priv("~");
  nh_priv.param("imu_pub_rate",     imu_pub_rate,     20.0);  // Hz
  nh_priv.param("health_pub_rate",  health_pub_rate,  2.0);   // Hz
  nh_priv.param("watchdog_timeout", watchdog_timeout, 2.0);   // seconds
  nh_priv.param("esc_pwm_min",      esc_pwm_min,      1100);  // microseconds
  nh_priv.param("esc_pwm_max",      esc_pwm_max,      1900);  // microseconds
  nh_priv.param("num_thrusters",    num_thrusters,    8);     // amount

  // lets show em what we got
  ROS_INFO_STREAM("param imu_pub_rate: " << imu_pub_rate);
  ROS_INFO_STREAM("param health_pub_rate: " << health_pub_rate);
  ROS_INFO_STREAM("param watchdog_timeout: " << watchdog_timeout);
  ROS_INFO_STREAM("param esc_pwm_min: " << esc_pwm_min);
  ROS_INFO_STREAM("param esc_pwm_max: " << esc_pwm_max);
  ROS_INFO_STREAM("param num_thrusters: " << num_thrusters);

  // connects subs and pubs
  thruster_enable_sub = nh.subscribe<std_msgs::Bool>("thruster_enable", 1, &Controller::thrusterEnableCallback, this);
  thruster_sub = nh.subscribe<bluerov::Thruster>("thruster", 1, &Controller::thrusterCallback, this);

  // require user to explicitly enable thrusters
  enabled = false;

  // kill on motors on startup
  stopAllThrusters();
}

void Controller::spin() {
  while(ros::ok()) {
    ros::spinOnce();
  }
}

void Controller::stopAllThrusters() {
  for(int i = 0; i < num_thrusters; i++) {
    ROS_WARN("stopAllThrusters");
  }
}

void Controller::thrusterEnableCallback(const std_msgs::Bool::ConstPtr& msg) {
  // receive new enabled setting
  enabled = msg->data;

  // if disabled, immediately stop all motors
  if(!enabled) {
    stopAllThrusters();
  }
}

void Controller::thrusterCallback(const bluerov::Thruster::ConstPtr& thruster) {
  if(enabled) {
    ROS_WARN("thrusterCallback");
  }
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "controller");
  Controller controller;
  controller.spin();
  return 0;
}
