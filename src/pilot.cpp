/*
 * File: bluerov/src/pilot.cpp
 * Author: Josh Villbrandt <josh@javconcepts.com>
 * Date: April 2015
 * Description: Sends thruster commands in microseconds to the BlueROV.
 */

#include <ros/ros.h>
#include <bluerov/Thruster.h>

class Pilot {
  public:
    Pilot();
    void spin();

  private:
    ros::NodeHandle nh;
    ros::Publisher thruster_pub;

    int n_thrusters;
    double pub_rate;
    ros::Time last_pub_time;
    ros::Duration pub_period;

    void sendThrusterMessage();
};

Pilot::Pilot() {
  // load parameters
  ros::NodeHandle nh_priv("~");
  nh_priv.param("n_thrusters", n_thrusters, 1); // amount
  nh_priv.param("pub_rate", pub_rate, 10.0); // Hz

  // lets show em what we got
  ROS_INFO_STREAM("param n_thrusters: " << n_thrusters);
  ROS_INFO_STREAM("param pub_rate: " << pub_rate);

  // connects subs and pubs
  thruster_pub = nh.advertise<bluerov::Thruster>("thruster", 1);

  // setup
  last_pub_time = ros::Time::now();
  pub_period = ros::Duration(1.0 / pub_rate);
}

void Pilot::spin() {
  while(ros::ok()) {
    // time to publish?
    if(ros::Time::now() > (last_pub_time + pub_period)) {
      sendThrusterMessage();
      last_pub_time = ros::Time::now();
    }

    // call all waiting callbacks
    ros::spinOnce();
  }
}

void Pilot::sendThrusterMessage() {
  // create message
  bluerov::Thruster thruster_msg;
  thruster_msg.header.stamp = ros::Time::now();
  thruster_msg.header.frame_id = "base_link";

  // set motor commands
  for (int i = 0; i < n_thrusters; i++)
  {
    // thruster values should be between 1100 and 1900 microseconds (us)
    // values less than 1500 us are backwards; values more than are forwards
    thruster_msg.commands.data.push_back(1500);
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
