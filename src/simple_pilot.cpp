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

class Pilot {
  public:
    Pilot();
    void spin();

  private:
    ros::NodeHandle nh;
    // ros::Publisher mavlink_pub;
    ros::ServiceClient command_client;
    ros::Subscriber cmd_vel_sub;

    void setServo(int index, float pulse_width);
    void velCallback(const geometry_msgs::Twist::ConstPtr& update);
};

Pilot::Pilot() {
  // connects subs and pubs
  command_client = nh.serviceClient<mavros::CommandLong>("/mavros/cmd/command");
  // mavlink_pub = nh.advertise<mavros::Mavlink>("/mavlink/to", 1);
  cmd_vel_sub = nh.subscribe<geometry_msgs::Twist>("cmd_vel", 1, &Pilot::velCallback, this);
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

void Pilot::setServo(int index, float value) {
  // thruster values should be between 1100 and 1900 microseconds (us)
  // values less than 1500 us are backwards; values more than are forwards
  int pulse_width = (value + 1) * 400 + 1100;

  // send mavros command message
  // http://docs.ros.org/api/mavros/html/srv/CommandLong.html
  // CMD_DO_SET_SERVO: https://pixhawk.ethz.ch/mavlink/
  mavros::CommandLong srv;
  srv.request.command = mavros::CommandLongRequest::CMD_DO_SET_SERVO;
  srv.request.param1 = index + 1; // servos are 1-indexed here
  srv.request.param2 = pulse_width;
  bool result = command_client.call(srv);
}

void Pilot::velCallback(const geometry_msgs::Twist::ConstPtr& cmd_vel) {
  // extract cmd_vel message
  float roll     = cmd_vel->angular.x;
  float pitch    = cmd_vel->angular.y;
  float yaw      = cmd_vel->angular.z;
  float forward  = cmd_vel->linear.x;
  float strafe   = cmd_vel->linear.y;
  float vertical = cmd_vel->linear.z;

  // build thruster commands (expected to be between -1 and 1)
  float thruster[6];
  thruster[0] = roll + 0.5*strafe + 0.5*pitch + 0.5*vertical; // Vertical Left (VL)
  thruster[1] = -pitch + vertical; // Vertical Back (VB)
  thruster[2] = -roll - 0.5*strafe + 0.5*pitch + 0.5*vertical; // Vertical Right (VR)
  thruster[3] = yaw + forward; // Forward Left (FL)
  thruster[4] = strafe; // LATeral (LAT)
  thruster[5] = -yaw + forward; // Forward Right (FR)

  // send thruster positions
  for(int i = 0; i < 6; i++) {
   setServo(i, thruster[i]);
  }
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "pilot");
  Pilot pilot;
  pilot.spin();
  return 0;
}
