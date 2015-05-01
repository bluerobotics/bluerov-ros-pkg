/*
 * File: bluerov/src/pilot.cpp
 * Author: Josh Villbrandt <josh@javconcepts.com>
 * Date: April 2015
 * Description: Sends thruster commands in microseconds to the BlueROV.
 */

#include <math.h>
#include <ros/ros.h>
#include <bluerov/Thruster.h>
#include <geometry_msgs/Twist.h>

class Pilot {
  public:
    Pilot();
    void spin();

  private:
    ros::NodeHandle nh;
    ros::Publisher thruster_pub;
    ros::Subscriber cmd_vel_sub;

    void velCallback(const geometry_msgs::Twist::ConstPtr& update);
};

Pilot::Pilot() {}

void Pilot::spin() {
  while(ros::ok()) {
    // call all waiting callbacks
    ros::spinOnce();
  }
}

void Pilot::velCallback(const geometry_msgs::Twist::ConstPtr& cmd_vel) {
  // init thruster message
  bluerov::Thruster thruster_msg;
  thruster_msg.header.stamp = ros::Time::now();
  thruster_msg.header.frame_id = "base_link";

  // extract cmd_vel message
  float roll     = cmd_vel->angular.x;
  float pitch    = cmd_vel->angular.y;
  float yaw      = cmd_vel->angular.z;
  float forward  = cmd_vel->linear.x;
  float strafe   = cmd_vel->linear.y;
  float vertical = cmd_vel->linear.z;

  // thruster values should be between 1100 and 1900 microseconds (us)
  // values less than 1500 us are backwards; values more than are forwards
  static const int16_t base = 1500;

  // build thruster commands
  int16_t thruster[6];
  thruster[0] = base + roll + 0.5*strafe + 0.5*pitch + 0.5*vertical; // Vertical Left (VL)
  thruster[1] = base - pitch + vertical; // Vertical Back (VB)
  thruster[2] = base - roll - 0.5*strafe + 0.5*pitch + 0.5*vertical; // Vertical Right (VR)
  thruster[3] = base + yaw + forward; // Forward Left (FL)
  thruster[4] = base + strafe; // LATeral (LAT)
  thruster[5] = base - yaw + forward; // Forward Right (FR)

  // apply commands to message
  for(int i = 0; i < 6; i++) {
    thruster_msg.commands.data.push_back(thruster[i]);
  }

  // publish message
  thruster_pub.publish(thruster_msg);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "pilot");
  Pilot pilot;
  pilot.spin();
  return 0;
}
