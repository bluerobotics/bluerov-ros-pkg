/*
 * File: bluerov/src/navio_controller.cpp
 * Author: J. Neilan <jimbolysses@gmail.com>
 * Date: October 2015
 * Description: Sends thruster commands in microseconds to the Navio+ RPi
 *	sheild via the Navio+ API.
 */

#include "navio_controller.h"

NavioController::NavioController()
{
  p_interface 	= new NavioInterface();
  _cmdVelSub 	= _nodeHandle.subscribe<geometry_msgs::Twist>( "cmd_vel", 1, &NavioController::VelCallback, this );

  imuPub 	= _nodeHandle.advertise<sensor_msgs::Imu>( "imu_raw", 1 );
  gpsPub 	= _nodeHandle.advertise<sensor_msgs::NavSatFix>( "gps_status", 1 );

  //adcPub	= _nodeHandle.publish<sensor_ma -- create new message for adc --
  ahrsPub	= _nodeHandle.advertise<sensor_msgs::Imu>( "imu_fused", 1 );

  //baroPub	= _nodeHandle.publish< -- create new message for barometer --

}

NavioController::~NavioController()
{

}

void NavioController::InitNavioInterface()
{
  p_interface->Initialize();
  // 50Hz
  p_interface->SetFrequency( 50 );
}

void NavioController::Spin()
{
  // 200Hz
  ros::Rate loopRate( 200 );

  while( ros::ok() )
  {
    ros::spinOnce();
    loopRate.sleep();
  }
}

void NavioController::SetServo( int index, float value )
{
  int step = 3; // output 1 on navio, servo1 is channel 3, etc...

  // thruster values should be between 1100 and 1900 microseconds (us)
  // values less than 1500 us are backwards; values more than are forwards
  int pulseWidth = (value + 1) * 400 + 1100;

  p_interface->SendPWM( index + step, pulseWidth );
}

void NavioController::VelCallback (const geometry_msgs::Twist::ConstPtr &cmdVelIn )
{
  // Get velocity commands
  float roll     = cmdVelIn->angular.x;
  float pitch    = cmdVelIn->angular.y;

  float yaw      = cmdVelIn->angular.z;
  float forward  = cmdVelIn->linear.x;

  float strafe   = cmdVelIn->linear.y;
  float vertical = cmdVelIn->linear.z;

  // Build thruster commands (expected to be between -1 and 1)
  float thruster[5];

  // TODO - work out logic for 4 vectored and 1 vert thruster arrangement --

  thruster[0] = forward;

  //ROS_INFO( "%f", thruster[0] );

  for( size_t i = 0; i < 1; ++i )
  {
    SetServo( i, thruster[i] );
  }
}

void * NavioController::PublishNavioData( void *controllerIn )
{
  NavioController *controller = (navioController*)controllerIn;

  sensor_msgs::Imu 		imuRawMessage;
  sensor_msgs::Imu 		imuaFusedMessage;
  sensor_msgs::NavSatFix	gpsMessage;

  std::vector<float>		data;

  // A few things horrid about this black. Imu raw used the float[9] of the 
  // ROS Imu message type. No raw IMU message exists, so we need to create one.
  // For now, using existing messages for testing.
  while( true )
  {
    // IMU 
    data = controller->GetImu();

    imuRawMessage.orientation.x 	= -1;
    imuRawMessage.linear_acceleration.x = -1;
    imuRawMessage.angular_velocity.x 	= -1

    imuRawMessage.angular_velocity_covariance[0] = data[0];
    imuRawMessage.angular_velocity_covariance[1] = data[1];
    imuRawMessage.angular_velocity_covariance[2] = data[2];

    imuRawMessage.angular_velocity_covariance[3] = data[3];
    imuRawMessage.angular_velocity_covariance[4] = data[4];
    imuRawMessage.angular_velocity_covariance[5] = data[5];

    imuRawMessage.angular_velocity_covariance[6] = data[6];
    imuRawMessage.angular_velocity_covariance[7] = data[7];
    imuRawMessage.angular_velocity_covariance[8] = data[8];

    imuPub.publish( imuRawMessage );

    // GPS
    //data = controller-GetGPS();

    // IMU fused
    //data = controller->GetAHRS();
  }

  return nullptr;
}

//------- Main -------------------------------
int main( int argc, char **argv )
{
  ros::init( argc, argv, "navio_controller" );

  ROS_INFO( "Navio+ Controller Online" );

  NavioController controller;
  controller.Spin();

  return 0;
}
