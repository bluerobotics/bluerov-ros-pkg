#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <iostream>
#include <math.h> 
#include <mavros_msgs/CommandLong.h>
#include <mavros_msgs/OverrideRCIn.h>
#include <mavros_msgs/SetMode.h>
#include <string>

using namespace std;

class Teleop
{
public:
  Teleop();
  void spin();

private:
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
  void setArming(bool arm);
  void setMode(string mode);
  
  ros::NodeHandle nh;
  ros::Subscriber joy_sub_;
  ros::Publisher rc_override_pub;
  ros::ServiceClient cmd_client;
  ros::ServiceClient set_mode;

  int RCmsg[8];
  
  enum {COMPONENT_ARM_DISARM=400};

  bool flag_lr;
  bool flag_ud;

  int map(float x);
  int map(float x, int inmin, int inmax, int outmin, int outmax);
};

Teleop::Teleop()
{
  joy_sub_ = nh.subscribe<sensor_msgs::Joy>("joy", 10, &Teleop::joyCallback, this);
  rc_override_pub = nh.advertise<mavros_msgs::OverrideRCIn>("/mavros/rc/override", 1);
  cmd_client = nh.serviceClient<mavros_msgs::CommandLong>("/mavros/cmd/command");
  set_mode = nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
  
  RCmsg[0] = 1500;
  RCmsg[1] = 1500;
  RCmsg[2] = 1500;
  RCmsg[3] = 1500;
  RCmsg[4] = 1500;
  RCmsg[5] = 1500;
  RCmsg[6] = 1500;
  RCmsg[7] = 1500;

  flag_lr = false;
  flag_ud = false;
}

void Teleop::spin() {
  ros::Rate loop(10);

  while(ros::ok()) {
    mavros_msgs::OverrideRCIn msg;
    msg.channels[0] = RCmsg[0];
    msg.channels[1] = RCmsg[1];
    msg.channels[2] = RCmsg[2];
    msg.channels[3] = RCmsg[3];
    msg.channels[4] = RCmsg[4];
    msg.channels[5] = RCmsg[5];
    msg.channels[6] = RCmsg[6];
    msg.channels[7] = RCmsg[7];
    rc_override_pub.publish(msg);
  
    // call all waiting callbacks
    ros::spinOnce();

    // enforce a max publish rate
    loop.sleep();
  }
}

int Teleop::map(float x)
{
  float inmin = -1;
  float inmax = 1;
  float outmin = 1000;
  float outmax = 2000;
  
  return round(float((x - inmin) * (outmax - outmin) / (inmax - inmin) + outmin));
}

int Teleop::map(float x, int inmin, int inmax, int outmin, int outmax)
{
    return round(float((x - inmin) * (outmax - outmin) / (inmax - inmin) + outmin));
}

void Teleop::setMode(string mode)
{
    mavros_msgs::SetMode srv_setMode;
    srv_setMode.request.base_mode = 0;
    srv_setMode.request.custom_mode = mode;
    if(set_mode.call(srv_setMode)){
        ROS_INFO("Flight Mode %s", mode.c_str());
    }else{
        ROS_ERROR("Failed SetMode");
    }    
}

void Teleop::setArming(bool arm) 
{
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

void Teleop::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
 if(joy->buttons[6]) //L
 {
      setArming(false);
 }
 
 if(joy->buttons[7]) //R
 {
    setArming(true);
 }
 
 if(joy->buttons[2]) //X
 {
    if(flag_lr)
        flag_lr = false;
    else
        flag_lr = true;
 }
 
  if(joy->buttons[3]) //Y
 {
    if(flag_ud)
        flag_ud = false;
    else
        flag_ud = true;
 }
 
 if(joy->buttons[0]) //A
    setMode("MANUAL");
    
 if(joy->buttons[1]) //B
    setMode("STABILIZE");
 
 /*
 cout << "L_LR " << map(joy->axes[0])  
      << "\nL_UD " << map(joy->axes[1]) 
      << "\nR_LR " << map(joy->axes[3]) 
      << "\nR_UD " << map(joy->axes[4])
      << "\nLT " << map(joy->axes[2]) 
      << "\nRT " << map(joy->axes[5])
      << "\nA " << joy->buttons[0]
      << "\nB " << joy->buttons[1]
      << "\nX " << joy->buttons[2] 
      << "\nY " << joy->buttons[3] 
      << "\nLTB " << joy->buttons[4] 
      << "\nRTB " << joy->buttons[5] 
      << "\nL " << joy->buttons[6] 
      << "\nR " << joy->buttons[7] 
      << "\nXB " << joy->buttons[8] 
      << "\nLJB " << joy->buttons[9] 
      << "\nRJB " << joy->buttons[10] << endl;
  */
  
  if(flag_ud)
  {
    RCmsg[0] = map(joy->axes[1]); //set pitch to left joy u/d
  }else{
    RCmsg[7] = map(joy->axes[1]); //set camera to left joy u/d
  }
  
  if(flag_lr)
  {
    RCmsg[1] = map(joy->axes[0]); //set roll to left joy l/r
  }else{
    RCmsg[5] = map(joy->axes[0]); //set lateral to left joy l/r
  }
  
  RCmsg[2] = map(joy->axes[4]); //Throttle/Up-Down = R_UD
  RCmsg[3] = map(joy->axes[3]); //Yaw = R_LR
  RCmsg[4] = round((map(joy->axes[2]) + map(joy->axes[5]*-1))/2); //Forward = LTB,RTB
  
  for(int i=0; i<8; i++)
  {
    cout << RCmsg[i] << endl;
  }
  cout << endl;
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "teleop");
  Teleop teleop;

  teleop.spin();
  return 0;
}
