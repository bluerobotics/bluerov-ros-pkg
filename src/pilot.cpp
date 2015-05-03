/*
 * File: bluerov/src/pilot.cpp
 * Author: Josh Villbrandt <josh@javconcepts.com>
 * Date: April 2015
 * Description: Sends thruster commands in microseconds to the BlueROV.
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

    int n_thrusters;
    double pub_rate;
    ros::Time last_actuator_pub;
    ros::Duration min_actuator_pub_period;
    geometry_msgs::Twist::ConstPtr cmd_vel;

    void velCallback(const geometry_msgs::Twist::ConstPtr& update);
    void sendThrusterMessage();
};

Pilot::Pilot() {
  // load parameters
  ros::NodeHandle nh_priv("~");
  nh_priv.param("n_thrusters", n_thrusters, 8); // amount
  nh_priv.param("pub_rate", pub_rate, 10.0); // Hz

  // lets show em what we got
  ROS_INFO_STREAM("param n_thrusters: " << n_thrusters);
  ROS_INFO_STREAM("param pub_rate: " << pub_rate);

  // connects subs and pubs
  actuator_pub = nh.advertise<mavros::ActuatorControl>("actuator_control", 1);
  cmd_vel_sub = nh.subscribe<geometry_msgs::Twist>("cmd_vel", 1, &Pilot::velCallback, this);

  // setup
  last_actuator_pub = ros::Time::now();
  min_actuator_pub_period = ros::Duration(1.0 / pub_rate);
}

void Pilot::spin() {
  // enforce a max spin rate so we don't kill the CPU
  ros::Rate loop(1000); // Hz

  while(ros::ok()) {
    // force actuator publish for IMU in absence other control input
    if(ros::Time::now() > (last_actuator_pub + min_actuator_pub_period)) {
      sendThrusterMessage();
    }

    // call all waiting callbacks
    ros::spinOnce();
    loop.sleep();
  }
}

void Pilot::velCallback(const geometry_msgs::Twist::ConstPtr& update) {
  cmd_vel = update;
  sendThrusterMessage();
}

// double Pilot::thrustCommandT100(double thrustInNewtons) {
//   double pwm;

//   // deadzone screws up our polynomial estimate, so use two equations instead
//   // data from T100 spec page; see "extra/thruster equations.xlsx"
//   if(thrust >= 0.0) {
//     // todo
//   }
//   else {
//     pwm = 56.67 * pow(thrustInNewtons, 2) + 298.4 * thrustInNewtons + 1453.8;
//   }

//   return pwm;
// }

void Pilot::sendThrusterMessage() {
  // init message
  mavros::ActuatorControl actuator_msg;
  actuator_msg.header.stamp = ros::Time::now();
  actuator_msg.header.frame_id = "base_link";
  actuator_msg.group_mix = 0;

  // set motor commands
  for (int i = 0; i < n_thrusters; i++)
  {
    actuator_msg.controls[0] = 0.2;
  }

  // publish message
  actuator_pub.publish(actuator_msg);
  last_actuator_pub = ros::Time::now();
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "pilot");
  Pilot pilot;
  pilot.spin();
  return 0;
}
