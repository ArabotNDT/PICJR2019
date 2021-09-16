// Copyright 1996-2021 Cyberbotics Ltd.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef ROS_ARABOT_HPP
#define ROS_ARABOT_HPP

#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/InertialUnit.hpp>
#include <webots/Accelerometer.hpp>
#include <webots/Camera.hpp>

#include <webots_ros/get_float.h>
#include <webots_ros/get_int.h>
#include <webots_ros/set_bool.h>
#include <webots_ros/set_float.h>
#include <webots_ros/set_int.h>

#include <std_msgs/Float64.h>

#include <Ros.hpp>

using namespace webots;

class RosArabot : public Ros {
public:
  RosArabot();
  virtual ~RosArabot();

protected:
  virtual void setupRobot();
  virtual void setRosDevices(const char **hiddenDevices, int numberHiddenDevices);
  virtual void launchRos(int argc, char **argv);
  virtual int step(int duration);  

  bool setLeftWheelVelocityCallback(webots_ros::set_float::Request &req, webots_ros::set_float::Response &res);
  bool setRightWheelVelocityCallback(webots_ros::set_float::Request &req, webots_ros::set_float::Response &res);
  void publishCameraImage();
private:
  const double WHEEL_RADIUS_IN_MM = 35.0; 

  Motor          *mWheelMotor[2];
  PositionSensor *mWheelEncoder[2];
  DistanceSensor *mUltrasonicSensor[3]; //Ultrasonic Distance Sensor - HC-SR04 provides 2cm to 400cm with a ranging accuracy that can reach up to 3mm
  InertialUnit   *mGyro;
  Accelerometer  *mAccel;
  Camera         *mCamera;

  ros::ServiceServer mWheelVelocityServer[2];
  ros::Publisher     mWheelEncoderPublisher[2];
  ros::Publisher     mUltrasonicSensorPublisher[3];
  ros::Publisher     mCameraImagePublisher;
};

#endif  // ROS_ARABOT_HPP

