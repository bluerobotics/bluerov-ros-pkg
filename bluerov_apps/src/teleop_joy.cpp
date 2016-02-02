/*
 * File: bluerov_apps/src/teleop_joy.cpp
 * Author: Josh Villbrandt <josh@javconcepts.com>
 * Date: February 2016
 * Description: Manual remote control of ROVs like the bluerov_apps.
 */

#include <math.h>
#include <ros/ros.h>
#include <ros/console.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Bool.h>
#include <dynamic_reconfigure/server.h>
#include <bluerov_apps/teleop_joyConfig.h>
#include <mavros_msgs/CommandLong.h>
#include <mavros_msgs/CommandCode.h>

class TeleopJoy {
  public:
    TeleopJoy();
    void spin();

  private:
    void setArming(bool armed);
    // void setMode(bool armed);
    // void triggerTakeoff();
    // void triggerLanding();
    void configCallback(bluerov_apps::teleop_joyConfig &update, uint32_t level);
    void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
    // double computeAxisValue(const sensor_msgs::Joy::ConstPtr& joy, int index, double expo);

    ros::NodeHandle nh;
    // ros::Publisher cmd_vel_pub;
    // ros::Publisher hazard_enable_pub;
    ros::Subscriber joy_sub;

    dynamic_reconfigure::Server<bluerov_apps::teleop_joyConfig> server;
    bluerov_apps::teleop_joyConfig config;

    bool initLT;
    bool initRT;
};

TeleopJoy::TeleopJoy() {
  // connect dynamic reconfigure
  dynamic_reconfigure::Server<bluerov_apps::teleop_joyConfig>::CallbackType f;
  f = boost::bind(&TeleopJoy::configCallback, this, _1, _2);
  server.setCallback(f);

  // connects subs and pubs
  // cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
  // hazard_enable_pub = nh.advertise<std_msgs::Bool>("hazard_enable", 1);
  joy_sub = nh.subscribe<sensor_msgs::Joy>("joy", 1, &TeleopJoy::joyCallback, this);

  // set initial values
  initLT = false;
  initRT = false;
}

void TeleopJoy::spin() {
  ros::Rate loop(config.pub_rate);

  while(ros::ok()) {
    // call all waiting callbacks
    ros::spinOnce();

    // enforce a max publish rate
    loop.sleep();
  }
}

void TeleopJoy::configCallback(bluerov_apps::teleop_joyConfig &update, uint32_t level) {
  ROS_INFO("reconfigure request received");
  config = update;
}

void TeleopJoy::setArming(bool enable)
{
  // generate request
  // mecanumbot::RobotHazardsEnable srv;
  // srv.request.enable = enable;

  // // send request
  // if(hazards_cli.call(srv)) {
  //   if(enable) {
  //     ROS_INFO("Hazards enabled");
  //   }
  //   else {
  //     ROS_INFO("Hazards disabled");
  //   }
  // }
  // else {
  //   ROS_ERROR("Failed to update hazards");
  // }
}

void TeleopJoy::joyCallback(const sensor_msgs::Joy::ConstPtr& joy) {
  // // send cmd_vel message
  // geometry_msgs::Twist msg;
  // msg.linear.x =  config.x_scaling  * computeAxisValue(joy, config.x_axis,  config.expo);
  // msg.linear.y =  config.y_scaling  * computeAxisValue(joy, config.y_axis,  config.expo);
  // msg.linear.z =  config.z_scaling  * computeAxisValue(joy, config.z_axis,  config.expo);
  // msg.angular.x = config.wx_scaling * computeAxisValue(joy, config.wx_axis, config.expo);
  // msg.angular.y = config.wy_scaling * computeAxisValue(joy, config.wy_axis, config.expo);
  // msg.angular.z = config.wz_scaling * computeAxisValue(joy, config.wz_axis, config.expo);
  // cmd_vel_pub.publish(msg);

  // // send hazards enable message
  // if(joy->buttons[config.disable_button] > 0) {
  //   std_msgs::Bool hmsg;
  //   hmsg.data = false;
  //   hazard_enable_pub.publish(hmsg);
  //   // ROS_INFO("Hazards disabled.");
  // }
  // else if(joy->buttons[config.enable_button] > 0) {
  //   std_msgs::Bool hmsg;
  //   hmsg.data = true;
  //   hazard_enable_pub.publish(hmsg);
  //   // ROS_INFO("Hazards enabled.");
  // }

  // remember buttons for the future
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "teleop_joy");
  TeleopJoy teleop_joy;
  teleop_joy.spin();
  return 0;
}
