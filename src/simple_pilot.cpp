/*
 * File: bluerov/src/simple_pilot.cpp
 * Author: Josh Villbrandt <josh@javconcepts.com>
 * Date: April 2015
 * Description: Sends actuator commands to a mavlink controller.
 */

#include <math.h>
#include <ros/ros.h>
#include <mavros/ActuatorControl.h>
#include <geometry_msgs/Twist.h>

class Pilot {
  public:
    Pilot();
    void spin();

  private:
    ros::NodeHandle nh;
    ros::Publisher actuator_pub;
    ros::Subscriber cmd_vel_sub;

    void velCallback(const geometry_msgs::Twist::ConstPtr& update);
};

Pilot::Pilot() {
  // connects subs and pubs
  actuator_pub = nh.advertise<mavros::ActuatorControl>("/mavros/actuator_control", 1);
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

void Pilot::velCallback(const geometry_msgs::Twist::ConstPtr& cmd_vel) {
  ROS_INFO("velCallback");

  // init thruster message
  mavros::ActuatorControl actuator_msg;
  actuator_msg.header.stamp = ros::Time::now();
  actuator_msg.header.frame_id = "base_link";
  actuator_msg.group_mix = 0;

  // extract cmd_vel message
  float roll     = cmd_vel->angular.x;
  float pitch    = cmd_vel->angular.y;
  float yaw      = cmd_vel->angular.z;
  float forward  = cmd_vel->linear.x;
  float strafe   = cmd_vel->linear.y;
  float vertical = cmd_vel->linear.z;

  // build thruster commands for the BlueROV R1 vehicle
  actuator_msg.controls[0] = roll + 0.5*strafe + 0.5*pitch + 0.5*vertical; // Vertical Left (VL)
  actuator_msg.controls[1] = pitch + vertical; // Vertical Back (VB)
  actuator_msg.controls[2] = roll - 0.5*strafe + 0.5*pitch + 0.5*vertical; // Vertical Right (VR)
  actuator_msg.controls[3] = yaw + forward; // Forward Left (FL)
  actuator_msg.controls[4] = strafe; // LATeral (LAT)
  actuator_msg.controls[5] = yaw + forward; // Forward Right (FR)
  actuator_msg.controls[6] = 0.0;
  actuator_msg.controls[7] = 0.0;

  // publish message
  actuator_pub.publish(actuator_msg);
  ROS_INFO("velCallback complete");
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "pilot");
  Pilot pilot;
  pilot.spin();
  return 0;
}
