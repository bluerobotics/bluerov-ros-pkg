/*
 * File: bluerov/src/simple_pilot.cpp
 * Author: Josh Villbrandt <josh@javconcepts.com>
 * Date: April 2015
 * Description: Sends actuator commands to a mavlink controller.
 */

#include <vector>
#include <ros/ros.h>
#include <mavros/CommandLong.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Bool.h>
#include <dynamic_reconfigure/server.h>
#include <bluerov/simple_pilotConfig.h>

class Pilot {
  public:
    Pilot();
    void spin();

  private:
    ros::NodeHandle nh;
    // ros::Publisher mavlink_pub;
    ros::ServiceClient command_client;
    ros::Subscriber cmd_vel_sub;
    ros::Subscriber hazard_enable_sub;

    dynamic_reconfigure::Server<bluerov::simple_pilotConfig> server;
    bluerov::simple_pilotConfig config;

    bool hazards_enabled;

    void configCallback(bluerov::simple_pilotConfig &update, uint32_t level);
    void setServo(int index, float pulse_width);
    void velCallback(const geometry_msgs::Twist::ConstPtr& cmd_vel);
    void hazardCallback(const std_msgs::Bool::ConstPtr& msg);
};

Pilot::Pilot() {
  // connect dynamic reconfigure
  dynamic_reconfigure::Server<bluerov::simple_pilotConfig>::CallbackType f;
  f = boost::bind(&Pilot::configCallback, this, _1, _2);
  server.setCallback(f);

  // connects subs and pubs
  command_client = nh.serviceClient<mavros::CommandLong>("/mavros/cmd/command");
  cmd_vel_sub = nh.subscribe<geometry_msgs::Twist>("cmd_vel", 1, &Pilot::velCallback, this);
  hazard_enable_sub = nh.subscribe<std_msgs::Bool>("hazard_enable", 1, &Pilot::hazardCallback, this);

  // set initial values
  hazards_enabled = false;
}

void Pilot::spin() {
  // enforce a max spin rate so we don't kill the CPU
  ros::Rate loop(1000); // Hz

  while(ros::ok()) {
    // call all waiting callbacks
    ros::spinOnce();
    loop.sleep();
  }
}

void Pilot::configCallback(bluerov::simple_pilotConfig &update, uint32_t level) {
  ROS_INFO("reconfigure request received");
  config = update;
}

void Pilot::setServo(int index, float value) {
  // thruster values should be between 1100 and 1900 microseconds (us)
  // values less than 1500 us are backwards; values more than are forwards
  int pulse_width = (value + 1) * 400 + 1100;

  // send mavros command message
  // http://docs.ros.org/api/mavros/html/srv/CommandLong.html
  // CMD_DO_SET_SERVO (183): https://pixhawk.ethz.ch/mavlink/
  mavros::CommandLong srv;
  srv.request.command = 183; //mavros::CommandLongRequest::CMD_DO_SET_SERVO;
  srv.request.param1 = index + 1; // servos are 1-indexed here
  srv.request.param2 = pulse_width;
  bool result = command_client.call(srv);
  //ROS_INFO_STREAM("Pilot::setServo(" << index << ", " << value << ") = " << result);
}

void Pilot::velCallback(const geometry_msgs::Twist::ConstPtr& cmd_vel) {
  // only continue if hazards are enabled
  if(!hazards_enabled) return;

  // extract cmd_vel message
  float roll     = cmd_vel->angular.x;
  float pitch    = cmd_vel->angular.y;
  float yaw      = cmd_vel->angular.z;
  float forward  = cmd_vel->linear.x;
  float strafe   = cmd_vel->linear.y;
  float vertical = cmd_vel->linear.z;

  // build thruster commands (expected to be between -1 and 1)
  float thruster[6];
  thruster[0] =
    roll +
    -config.front_forward_decouple * forward +
    -config.front_strafe_decouple * strafe +
    -config.front_pitch_bias * pitch +
    config.front_vertical_bias * vertical +
    config.buoyancy_control; // Vertical Left (VL)
  thruster[1] =
    config.front_forward_decouple * forward +
    pitch +
    vertical +
    config.buoyancy_control; // Vertical Back (VB)
  thruster[2] =
    -roll +
    -config.front_forward_decouple * forward +
    config.front_strafe_decouple * strafe +
    -config.front_pitch_bias * pitch +
    config.front_vertical_bias * vertical +
    config.buoyancy_control;  // Vertical Right (VR)
  thruster[3] = -yaw + forward; // Forward Left (FL)
  thruster[4] = strafe; // LATeral (LAT)
  thruster[5] = yaw + forward; // Forward Right (FR)

  // send thruster positions
  for(int i = 0; i < 6; i++) {
   setServo(i, thruster[i]);
  }
}

void Pilot::hazardCallback(const std_msgs::Bool::ConstPtr& msg) {
  // save message data
  hazards_enabled = msg->data;
  if(hazards_enabled) ROS_INFO("Enabled thrusters.");
  else ROS_INFO("Disabled thrusters.");

  // zero thruster speeds
  if(!hazards_enabled) {
    for(int i = 0; i < 6; i++) {
     setServo(i, 0);
    }
  }
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "pilot");
  Pilot pilot;
  pilot.spin();
  return 0;
}
