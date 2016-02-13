/*
 * File: bluerov_apps/src/joystick_remapper.cpp
 * Author: Josh Villbrandt <josh@javconcepts.com>
 * Date: February 2016
 * Description: Manual remote control of ROVs like the bluerov_apps.
 */

#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/Joy.h>

class JoystickRemapper {
  public:
    JoystickRemapper();

  private:
    // functions
    void joyCallback(const sensor_msgs::Joy::ConstPtr& in);

    // node handle
    ros::NodeHandle nh;

    // pubs and subs
    ros::Subscriber joy_sub;
    ros::Publisher joy_pub;

    // config
    std::string user_warning;
    std::string joy_source;
    std::string joy_dest;
};

JoystickRemapper::JoystickRemapper() {
  // load parameters
  ros::NodeHandle nh_priv("~");
  nh_priv.param<std::string>("user_warning", user_warning, "");
  nh_priv.param<std::string>("joy_source", joy_source, "");
  nh_priv.param<std::string>("joy_dest", joy_dest, "");

  // lets show em what we got
  ROS_INFO("joy_source: %s", joy_source.c_str());
  ROS_INFO("joy_dest: %s", joy_dest.c_str());

  // announce warning if set
  if(user_warning.length() > 0) {
    ROS_WARN("%s", user_warning.c_str());
  }

  // connects subs and pubs
  joy_sub = nh.subscribe<sensor_msgs::Joy>(joy_source, 1, &JoystickRemapper::joyCallback, this);
  joy_pub = nh.advertise<sensor_msgs::Joy>(joy_dest, 1);
}

void JoystickRemapper::joyCallback(const sensor_msgs::Joy::ConstPtr& in) {
  // specifically support F310 to Xbox for now because d-pad buttons are
  // treated as axes on F310

  // init outgoing message
  sensor_msgs::Joy out;
  out.header = in->header;

  // translate axes
  // f310 axes (from): [left X, left Y, LT, right X, right Y, RT, pad L/R, pad U/D]
  // xbox axes (to):   [left X, left Y, LT, right X, right Y, RT]
  out.axes = std::vector<float>(in->axes);
  out.axes.pop_back();
  out.axes.pop_back();

  // translate buttons
  // f310 buttons (from): [A, B, X, Y LB, RB, BACK, START, POWER, left stick, right stick click]
  // xbox buttons (to):   [A, B, X, Y LB, RB, BACK, START, POWER, left stick, right stick click, pad L, pad R, pad U, pad D]
  out.buttons = std::vector<int>(in->buttons);
  out.buttons.push_back((in->axes[6] > 0.5) ? 1 : 0);
  out.buttons.push_back((in->axes[6] < -0.5) ? 1 : 0);
  out.buttons.push_back((in->axes[7] > 0.5) ? 1 : 0);
  out.buttons.push_back((in->axes[7] < -0.5) ? 1 : 0);

  joy_pub.publish(out);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "joystick_remapper");
  JoystickRemapper joystick_remapper;
  ros::spin();
  return 0;
}
