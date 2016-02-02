/*
 * File: bluerov_apps/src/teleop_joy.cpp
 * Author: Josh Villbrandt <josh@javconcepts.com>
 * Date: February 2016
 * Description: Manual remote control of ROVs like the bluerov_apps.
 */

#include <vector>
#include <math.h>
#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/Joy.h>
#include <dynamic_reconfigure/server.h>
#include <bluerov_apps/teleop_joyConfig.h>
#include <mavros_msgs/CommandBool.h>

// #include <geometry_msgs/Twist.h>
// #include <mavros_msgs/CommandLong.h>
// #include <mavros_msgs/CommandCode.h>

class TeleopJoy {
  public:
    TeleopJoy();
    void spin();

  private:
    // functions
    bool risingEdge(const sensor_msgs::Joy::ConstPtr& joy, int index);
    void setArming(bool armed);
    // void setMode(bool armed);
    // void triggerTakeoff();
    // void triggerLanding();
    void configCallback(bluerov_apps::teleop_joyConfig &update, uint32_t level);
    void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
    // double computeAxisValue(const sensor_msgs::Joy::ConstPtr& joy, int index, double expo);

    // node handle
    ros::NodeHandle nh;

    // dynamic reconfigure
    dynamic_reconfigure::Server<bluerov_apps::teleop_joyConfig> server;
    bluerov_apps::teleop_joyConfig config;

    // pubs and subs
    // ros::Publisher cmd_vel_pub;
    // ros::Publisher hazard_enable_pub;
    ros::Subscriber joy_sub;
    ros::ServiceClient arm_client;

    // state
    std::vector<int> previous_buttons;
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
  arm_client = nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");

  // set initial values
  // initLT = false;
  // initRT = false;
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

bool TeleopJoy::risingEdge(const sensor_msgs::Joy::ConstPtr& joy, int index) {
  return (joy->buttons[index] == 1 && previous_buttons[index] == 0);
}

void TeleopJoy::setArming(bool arm) {
  // generate request
  mavros_msgs::CommandBool srv;
  srv.request.value = arm;

  // send request
  if(arm_client.call(srv)) {
    if(arm) {
      ROS_INFO("Armed");
    }
    else {
      ROS_INFO("Disarmed");
    }
  }
  else {
    ROS_ERROR("Failed to update arming");
  }
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

  // init previous_buttons
  if(previous_buttons.size() != joy->buttons.size()) {
    previous_buttons = std::vector<int>(joy->buttons);
  }

  // arm
  if(risingEdge(joy, config.arm_button)) {
    setArming(true);
  }

  // disarm
  if(risingEdge(joy, config.disarm_button)) {
    setArming(false);
  }

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

  // remember current button states for future comparison
  previous_buttons = std::vector<int>(joy->buttons);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "teleop_joy");
  TeleopJoy teleop_joy;
  teleop_joy.spin();
  return 0;
}
