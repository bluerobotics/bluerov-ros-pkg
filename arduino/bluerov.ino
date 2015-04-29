/*
 * File: bluerov.ino
 * Author: Rustom Jehangir <rusty@bluerobotics.com>, Josh Villbrandt <josh@javconcepts.com>
 * Date: April 2015
 * Description: Sends thruster commands to ESCs; sends IMU data to RasPi.
 */

#include "Arduino.h"

// APM and sensors
#include <Utils.h>
#include "MPU6000.h"
#include "HMC5883.h"
#include "DCM.h"
#include "APM.h"

// ROS
#include <ros.h>
#include <std_msgs/Bool.h>
#include <bluerov/Thruster.h>
#include <bluerov/Health.h>
#include <sensor_msgs/Imu.h>

// constants
char frame_id[] = "/imu";
#define SENSOR_UPDATE_PERIOD    50    // 20 Hz
#define IMU_PUB_PERIOD          50    // 20 Hz
#define HEALTH_PUB_PERIOD       500   // 2 Hz
#define WATCHDOG_PERIOD         500   // 2 Hz
#define ESC_PWM_MIN             1100  // microseconds
#define ESC_PWM_MAX             1900  // microseconds
#define NUM_PWM_CHANNELS        8     // amount

// state variables
bool thruster_output_enabled = false;
unsigned long last_sensor_update = 0;
unsigned long last_thruster_msg = 0;
unsigned long last_imu_msg = 0;
unsigned long last_health_msg = 0;
ros::NodeHandle  nh;
ros::Publisher imu_pub("encoders", &imu_msg);
ros::Publisher health_pub("robot_health", &health_msg);

// thruster enable listener
void enableCallback(const std_msgs::Bool& enable_msg) {
  thruster_output_enabled = enable_msg;
  last_thruster_msg = millis();
}
ros::Subscriber<std_msgs::Bool> thruster_enable_sub("thruster_enable", &enableCallback);

// thruster command listener
void thrusterCallback(const bluerov::Thruster& thruster_msg) {
  if(thruster_output_enabled) {
    for(unsigned int i = 0; i < thruster_msg.commands.size(); i++) {
      APM::outputPWM(i, constrain(thruster_msg.commands[i], ESC_PWM_MIN, ESC_PWM_MAX));
    }
  }
  last_thruster_msg = millis();
}
ros::Subscriber<bluerov::Thruster> thruster_sub("thruster", &thrusterCallback);

// update APM sensors
void updateSensors() {
  if(MPU6000::newdata) {
    // calculate elapsed time
    float dt = (micros()-last_sensor_update)/1000000.0f;
    last_sensor_update = micros();

    // read new data
    MPU6000::read();
    HMC5883::read();

    // update 3D heading calc using previous DCM data
    // TODO: save declination to EPROM and update through ROS msg
    HMC5883::calculate(DCM::roll, DCM::pitch);
    HMC5883::applyDeclination(12.4);

    // update DCM
    DCM::updateMeasurements(
      MPU6000::gyroY,
      MPU6000::gyroX,
      -MPU6000::gyroZ,
      -MPU6000::accelY,
      -MPU6000::accelX,
      MPU6000::accelZ,
      dt);
    DCM::normalize();
    DCM::driftCorrection(HMC5883::heading);
    DCM::accelerationCorrection();
    DCM::convertDCMtoEuler();
  }
}

// immediately stop all thrusters
void stopAllThrusters() {

}

// publish IMU message
void publishImuMsg() {
  bluerov::Health imu_msg;
  imu_msg.header.frame_id = frame_id;
  imu_msg.header.stamp = nh.now();
  imu_pub.publish(&imu_msg);
}

// publish IMU message
void publishHealthMsg() {
  bluerov::Health health_msg;
  health_msg.header.frame_id = frame_id;
  health_msg.header.stamp = nh.now();
  health_msg.cpu_used = 0.0; //utils.cpuUsage(HEALTH_PUB_PERIOD);
  health_msg.mem_used = 0.0; //utils.memUsage();
  health_msg.v_batt = 0.0;
  health_msg.i_batt = 0.0;
  health_msg.t_internal = 0.0; //bmp.readTemperature();
  health_msg.p_internal = 0.0; //bmp.readPressure();
  health_msg.sw_1 = false;
  health_msg.sw_2 = false;
  health_pub.publish(&health_msg);
}

void setup() {
  // set barometer CS pin high so it doesn't hog the bus
  pinMode(40, OUTPUT);
  digitalWrite(40, HIGH);

  // init APM
  APM::init();
  MPU6000::init();
  HMC5883::init();
  DCM::init();

  // set compass offsets
  if(false) HMC5883::calibrateOffsets();
  HMC5883::set_offset(108, 6, 96);

  // init ROS
  nh.initNode();
  nh.subscribe(thruster_enable_sub);
  nh.subscribe(thruster_sub);
  nh.advertise(imu_pub);
  while(!nh.connected()) nh.spinOnce();
}

void loop() {
  unsigned long cycle_start = millis();

  // stop all thrusters if thruster output is disabled
  if(!thruster_output_enabled) {
    stopAllThrusters();
  }

  // update sensors
  if((cycle_start - last_sensor_update) > SENSOR_UPDATE_PERIOD) {
    updateSensors();
    last_sensor_update = cycle_start;
  }

  // publish IMU message
  if((cycle_start - last_imu_msg) > IMU_PUB_PERIOD) {
    publishImuMsg();
    last_imu_msg = cycle_start;
  }

  // publish Health message
  if((cycle_start - last_health_msg) > HEALTH_PUB_PERIOD) {
    publishHealthMsg();
    last_health_msg = cycle_start;
  }

  // thruster safety watchdog
  if((cycle_start - last_thruster_msg) > WATCHDOG_PERIOD) {
    thruster_output_enabled = false;
  }

  // send / receive ROS messages
  nh.spinOnce();
}
