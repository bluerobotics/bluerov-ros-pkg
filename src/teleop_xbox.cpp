/*
 * File: bluerov/src/pilot.cpp
 * Author: Josh Villbrandt <josh@javconcepts.com>
 * Date: April 2015
 * Description: Sends thruster commands in microseconds to the BlueROV.
 */

#include <math.h>
#include <ros/ros.h>
#include <ros/console.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <dynamic_reconfigure/server.h>
#include <bluerov/teleop_xboxConfig.h>

class TeleopXbox {
  public:
    TeleopXbox();
    void spin();

  private:
    void configCallback(bluerov::teleop_xboxConfig &update, uint32_t level);
    void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
    double computeAxisValue(const sensor_msgs::Joy::ConstPtr& joy, int index, double expo);

    ros::NodeHandle nh;
    ros::Publisher vel_pub;
    ros::Subscriber joy_sub;

    dynamic_reconfigure::Server<bluerov::teleop_xboxConfig> server;
    bluerov::teleop_xboxConfig config;
};

TeleopXbox::TeleopXbox() {
  // connect dynamic reconfigure
  dynamic_reconfigure::Server<bluerov::teleop_xboxConfig>::CallbackType f;
  f = boost::bind(&TeleopXbox::configCallback, this, _1, _2);
  server.setCallback(f);

  // connects subs and pubs
  vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
  joy_sub = nh.subscribe<sensor_msgs::Joy>("joy", 1, &TeleopXbox::joyCallback, this);
}

void TeleopXbox::spin() {
  while(ros::ok()) {
    // enforce a crude max rate of 100Hz
    ros::Duration(0.01).sleep();

    // call all waiting callbacks
    ros::spinOnce();
  }
}

void TeleopXbox::configCallback(bluerov::teleop_xboxConfig &update, uint32_t level) {
  ROS_INFO("reconfigure request received");
  config = update;
}

void TeleopXbox::joyCallback(const sensor_msgs::Joy::ConstPtr& joy) {
  // send cmd_vel message
  geometry_msgs::Twist msg;
  msg.linear.x =  config.x_scaling  * computeAxisValue(joy, config.x_axis,  config.expo);
  msg.linear.y =  config.y_scaling  * computeAxisValue(joy, config.y_axis,  config.expo);
  msg.linear.z =  config.z_scaling  * computeAxisValue(joy, config.z_axis,  config.expo);
  msg.angular.x = config.wx_scaling * computeAxisValue(joy, config.wx_axis, config.expo);
  msg.angular.y = config.wy_scaling * computeAxisValue(joy, config.wy_axis, config.expo);
  msg.angular.z = config.wz_scaling * computeAxisValue(joy, config.wz_axis, config.expo);
  vel_pub.publish(msg);

  // send enable action
  //TODO
}

double TeleopXbox::computeAxisValue(const sensor_msgs::Joy::ConstPtr& joy, int index, double expo) {
  // TODO: validate index and expo inputs

  // grab axis value
  double value;
  if(index == 6) {
    // this is the trigger pair pseudo axis
    value = joy->axes[2] - joy->axes[5];
    if(value != 0.0) value = value / 2;
  }
  else if(index == 7) {
    // this is the bumper pair pseudo axis
    value = joy->buttons[5] - joy->buttons[4];
    if(value != 0.0) value = value / 2;
  }
  else {
    value = joy->axes[index];
  }

  // apply exponential scaling
  return expo * pow(value, 5) + (1.0 - expo) * value;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "teleop_xbox");
  TeleopXbox teleop_xbox;
  teleop_xbox.spin();
  return 0;
}
