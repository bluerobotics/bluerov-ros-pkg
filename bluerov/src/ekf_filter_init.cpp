/*
 * File: bluerov/src/ekf_filter_init.cpp
 * Author: Josh Villbrandt <josh@javconcepts.com>
 * Date: February 2016
 * Description: This application publishes a fake odom message to trigger the filter
 *   initialization in robot_pose_ekf when only an IMU is used.
 */

#include <ros/ros.h>
#include <tf/tf.h>
// #include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
// #include <std_msgs/Float32MultiArray.h>

// #define PI 3.14159265

class OdometryPublisher
{
  public:
    OdometryPublisher();
    void spin();

  private:
    void publishMessage();

    ros::NodeHandle nh;
    // ros::Subscriber enc_sub;
    ros::Publisher odom_pub;
    // tf::TransformBroadcaster odom_broadcaster;

    // double x, y, th;
    // double scale_x, scale_y, scale_th;
    // bool calibration_mode;
};


OdometryPublisher::OdometryPublisher()
{
  // connects subs and pubs
  // enc_sub = nh.subscribe<std_msgs::Float32MultiArray>("encoders", 10, &OdometryPublisher::encoderCallback, this);
  odom_pub = nh.advertise<nav_msgs::Odometry>("fake_odom_init", 5);
}

void OdometryPublisher::spin()
{
  while(ros::ok())
  {
    publishMessage();

    // call all waiting callbacks
    ros::Duration(1.0).sleep();
    ros::spinOnce();
  }
}

void OdometryPublisher::publishMessage()
{
  // set the header
  nav_msgs::Odometry odom;
  odom.header.stamp = ros::Time::now();
  odom.header.frame_id = "fcu";
  odom.child_frame_id = "base_link";

  // set the position
  odom.pose.pose.position.x = 0.0;
  odom.pose.pose.position.y = 0.0;
  odom.pose.pose.position.z = 0.0;
  odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);

  // set the velocity
  odom.twist.twist.linear.x = 0.0;
  odom.twist.twist.linear.y = 0.0;
  odom.twist.twist.angular.z = 0.0;

  // publish the message
  ROS_INFO("publishing fake_odom_init");
  odom_pub.publish(odom);
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "odometry_publisher");
  OdometryPublisher odometry_publisher;
  odometry_publisher.spin();
}
