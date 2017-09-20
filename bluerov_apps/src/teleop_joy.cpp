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
#include <mavros_msgs/CommandLong.h>
#include <mavros_msgs/OverrideRCIn.h>

class TeleopJoy {
  public:
    TeleopJoy();
    void spin();

  private:
    // functions
    bool risingEdge(const sensor_msgs::Joy::ConstPtr& joy, int index);
    void setArming(bool armed);
    // void setMode(uint8_t mode);
    void cmdTakeoffLand(bool takeoff);
    double computeAxisValue(const sensor_msgs::Joy::ConstPtr& joy, int index, double expo);
    uint16_t mapToPpm(double in);
    void configCallback(bluerov_apps::teleop_joyConfig &update, uint32_t level);
    void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);

    // node handle
    ros::NodeHandle nh;

    // dynamic reconfigure
    dynamic_reconfigure::Server<bluerov_apps::teleop_joyConfig> server;
    bluerov_apps::teleop_joyConfig config;

    // pubs and subs
    ros::Subscriber joy_sub;
    ros::Publisher rc_override_pub;
    ros::ServiceClient cmd_client;

    // constants
    enum {COMPONENT_ARM_DISARM=400}; // https://pixhawk.ethz.ch/mavlink/
    enum {NAV_LAND_LOCAL=23, NAV_TAKEOFF_LOCAL=24}; // https://pixhawk.ethz.ch/mavlink/
    enum {MODE_STABILIZE=1000, MODE_ALT_HOLD=2000}; // ppm in uS; from ArduSub/radio.cpp
    enum {PPS_MIN=1000, PPS_MAX=2000}; // uS

    // state
    uint16_t mode;
    uint16_t camera_tilt;
    bool initLT;
    bool initRT;
    std::vector<int> previous_buttons;
};

TeleopJoy::TeleopJoy() {
  // connect dynamic reconfigure
  dynamic_reconfigure::Server<bluerov_apps::teleop_joyConfig>::CallbackType f;
  f = boost::bind(&TeleopJoy::configCallback, this, _1, _2);
  server.setCallback(f);

  // connects subs and pubs
  joy_sub = nh.subscribe<sensor_msgs::Joy>("joy", 1, &TeleopJoy::joyCallback, this);
  rc_override_pub = nh.advertise<mavros_msgs::OverrideRCIn>("/mavros/rc/override", 1);
  cmd_client = nh.serviceClient<mavros_msgs::CommandLong>("/mavros/cmd/command");

  // initialize state variables
  mode = MODE_STABILIZE;
  camera_tilt = 1500;
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

bool TeleopJoy::risingEdge(const sensor_msgs::Joy::ConstPtr& joy, int index) {
  return (joy->buttons[index] == 1 && previous_buttons[index] == 0);
}

void TeleopJoy::setArming(bool arm) {
  // Arm/disarm method following:
  // https://github.com/mavlink/qgroundcontrol/issues/590
  // https://pixhawk.ethz.ch/mavlink/#MAV_CMD_COMPONENT_ARM_DISARM

  // generate request
  mavros_msgs::CommandLong srv;
  srv.request.command = COMPONENT_ARM_DISARM;
  srv.request.param1 = (arm ? 1 : 0);
  srv.request.param2 = 21196; // force disarm (see GCS_Mavlink.cpp)

  // send request
  if(cmd_client.call(srv)) {
    ROS_INFO(arm ? "Armed" : "Disarmed");
  }
  else {
    ROS_ERROR("Failed to update arming");
  }
}

void TeleopJoy::cmdTakeoffLand(bool takeoff) {
  // https://pixhawk.ethz.ch/mavlink/#MAV_CMD_NAV_LAND_LOCAL

  // generate request
  mavros_msgs::CommandLong srv;
  srv.request.command = (takeoff ? NAV_TAKEOFF_LOCAL : NAV_LAND_LOCAL);
  srv.request.param1 = 0;
  srv.request.param2 = 0;
  srv.request.param3 = config.scend_rate;
  srv.request.param4 = 0;
  srv.request.param5 = 0;
  srv.request.param6 = 0;
  srv.request.param7 = (takeoff ? config.takeoff_depth : 0);

  // send request
  if(cmd_client.call(srv)) {
    ROS_INFO(takeoff ? "Taking off" : "Landing");
  }
  else {
    ROS_ERROR("Failed to request takeoff/land");
  }
}

uint16_t TeleopJoy::mapToPpm(double in) {
  // in should be -1 to 1
  // out should be 1000 to 2000 (microseconds)

  uint16_t out = 1000 + (in + 1.0) * 500;

  if(out > 2000) {
    return 2000;
  }
  else if(out < 1000) {
    return 1000;
  }
  else {
    return out;
  }
}

double TeleopJoy::computeAxisValue(const sensor_msgs::Joy::ConstPtr& joy, int index, double expo) {
  // return 0 if axis index is invalid
  if(index < 0 || index>= joy->axes.size()) {
    return 0.0;
  }

  // grab axis value
  double value;
  if(index == 6) {
    // the joystick driver initializes all values to 0.0, however, the triggers
    // physically spring back to 1.0 - let's account for this here
    double lt = joy->axes[2];
    double rt = joy->axes[5];
    if(lt < -0.01 || lt > 0.01) initLT = true;
    else if(!initLT) lt = 1.0;
    if(rt < -0.01 || rt > 0.01) initRT = true;
    else if(!initRT) rt = 1.0;

    // this is the trigger pair pseudo axis (LT-RT; pressing RT results in a positive number)
    value = (lt - rt) / 2.0;
  }
  else {
    value = joy->axes[index];
  }

  // apply exponential scaling
  return expo * pow(value, 5) + (1.0 - expo) * value;
}

void TeleopJoy::joyCallback(const sensor_msgs::Joy::ConstPtr& joy) {
  // init previous_buttons
  if(previous_buttons.size() != joy->buttons.size()) {
    previous_buttons = std::vector<int>(joy->buttons);
  }

  // arm, disarm
  if(risingEdge(joy, config.disarm_button)) {
    setArming(false);
  }
  else if(risingEdge(joy, config.arm_button)) {
    setArming(true);
  }

  // mode switching
  if(risingEdge(joy, config.stabilize_button)) {
    mode = MODE_STABILIZE;
  }
  else if(risingEdge(joy, config.alt_hold_button)) {
    mode = MODE_ALT_HOLD;
  }

  // takeoff and land
  if(risingEdge(joy, config.land_button)) {
    cmdTakeoffLand(false);
  }
  else if(risingEdge(joy, config.takeoff_button)) {
    cmdTakeoffLand(true);
  }

  // change camera_tilt
  if(risingEdge(joy, config.cam_tilt_reset)) {
    camera_tilt = 1500;
  }
  else if(risingEdge(joy, config.cam_tilt_up)) {
    camera_tilt = camera_tilt + config.cam_tilt_step;
    if(camera_tilt > PPS_MAX) {
      camera_tilt = PPS_MAX;
    }
  }
  else if(risingEdge(joy, config.cam_tilt_down)) {
    camera_tilt = camera_tilt - config.cam_tilt_step;
    if(camera_tilt < PPS_MIN) {
      camera_tilt = PPS_MIN;
    }
  }

  // remember current button states for future comparison
  previous_buttons = std::vector<int>(joy->buttons);

  // send rc override message
  mavros_msgs::OverrideRCIn msg;

  msg.channels[5] = mapToPpm(config.x_scaling  * computeAxisValue(joy, config.x_axis,  config.expo)); // forward  (x)
  msg.channels[6] = mapToPpm(config.y_scaling  * computeAxisValue(joy, config.y_axis,  config.expo)); // strafe   (y)
  msg.channels[2] = mapToPpm(config.z_scaling  * computeAxisValue(joy, config.z_axis,  config.expo)); // throttle (z)

  msg.channels[1] = mapToPpm(config.wx_scaling * computeAxisValue(joy, config.wx_axis, config.expo)); // roll     (wx)
  msg.channels[0] = mapToPpm(config.wy_scaling * computeAxisValue(joy, config.wy_axis, config.expo)); // pitch    (wy)
  msg.channels[3] = mapToPpm(config.wz_scaling * computeAxisValue(joy, config.wz_axis, config.expo)); // yaw      (wz)

  msg.channels[4] = mode; // mode
  msg.channels[7] = camera_tilt; // camera tilt

  rc_override_pub.publish(msg);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "teleop_joy");
  TeleopJoy teleop_joy;
  teleop_joy.spin();
  return 0;
}
