/*
 * File: bluerov/src/navio_controller.cpp
 * Author: J. Neilan <jimbolysses@gmail.com>
 * Date: October 2015
 * Description: Sends thruster commands in microseconds to the Navio+ RPi
 *	sheild via the Navio+ API.
 */

#include <ros/ros.h>
#include <ros/console.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/Bool.h>
#include "navio_interface.cpp"

class NavioController
{
 public:
    NavioController();
    virtual ~NavioController();

    // Attributes
    NavioInterface 			*p_interface;

    // Methods
    void Spin();
    void InitNavioInterface();

 private:
    // Attributes
    ros::NodeHandle 			_nodeHandle;
    ros::Subscriber			_cmdVelSub;

    ros::Publisher			_imuPub;
    ros::Publisher			_gpsPub;

    ros::Publisher			_adcPub;
    ros::Publisher			_ahrsPub;

    ros::Publisher			_baroPub;

    // Methods
    void VelCallback( const geometry_msgs::Twist::ConstPtr &cmdVel );
    void SetServo( int index, float value );
};

NavioController::NavioController()
{
  p_interface 	= new NavioInterface();
  _cmdVelSub 	= _nodeHandle.subscribe<geometry_msgs::Twist>( "cmd_vel", 1, &NavioController::VelCallback, this );

  _imuPub 	= _nodeHandle.publish<sensor_msgs::Imu>( "imu_raw", 1 );
  _gpsPub 	= _nodeHandle.publish<sensor_msgs::NavSatFix>( "gps_status", 1 );

  //_adcPub	= _nodeHandle.publish<sensor_ma -- create new message for adc --
  _ahrsPub	= _nodehandle.publish<sensor_msgs::Imu>( "imu_fused", 1 );

  //_baroPub	= _nodeHandle.publish< -- create new message for barometer --

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
  ros::Rate loopRate( 1000 );

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

int main( int argc, char **argv )
{
  ros::init( argc, argv, "navio_controller" );

  ROS_INFO( "Navio+ Controller Online" );

  NavioController controller;
  controller.Spin();

  return 0;
}
