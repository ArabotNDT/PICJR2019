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

#include "ros_arabot.hpp"

using namespace webots;

extern "C" {
int wb_robot_init();
}

using namespace std;

RosArabot::RosArabot() : Ros() {
}

RosArabot::~RosArabot() {
  mWheelVelocityServer[0].shutdown();
  mWheelVelocityServer[1].shutdown();
  mWheelEncoderPublisher[0].shutdown();
  mWheelEncoderPublisher[1].shutdown();
  mUltrasonicSensorPublisher[0].shutdown();
  mUltrasonicSensorPublisher[1].shutdown();
  mUltrasonicSensorPublisher[2].shutdown();
}

void RosArabot::setupRobot() {
  wb_robot_init();

  mRobot = new Robot();

  mWheelMotor[0] = mRobot->getMotor("motor_left");
  mWheelMotor[1] = mRobot->getMotor("motor_right");
  
  for(int i=0; i<2; i++) {
    mWheelMotor[i]->setPosition(INFINITY);
    mWheelMotor[i]->setVelocity(0.0);
  }
  
  mWheelEncoder[0] = mRobot->getPositionSensor("encolder_left");
  mWheelEncoder[1] = mRobot->getPositionSensor("encolder_right");
  
  mWheelEncoder[0]->enable(mRobot->getBasicTimeStep());
  mWheelEncoder[1]->enable(mRobot->getBasicTimeStep());
  
  for(int i=0; i<3 ; i++) {
    mUltrasonicSensor[i] = mRobot->getDistanceSensor("ultrassom" + to_string(i));
    mUltrasonicSensor[i]->enable(mRobot->getBasicTimeStep());
  }
  
  mGyro = mRobot->getInertialUnit("gyroscopio");
  mGyro->enable(mRobot->getBasicTimeStep());
  
  mAccel = mRobot->getAccelerometer("accelerometer");
  mAccel->enable(mRobot->getBasicTimeStep());
  
  mCamera = mRobot->getCamera("cam");
  mCamera->enable(mRobot->getBasicTimeStep());  
}

void RosArabot::launchRos(int argc, char **argv) {
  Ros::launchRos(argc, argv);

  // add services
  mWheelVelocityServer[0] = 
    nodeHandle()->advertiseService(name() + "/arabot/set_left_wheel_velocity", &RosArabot::setLeftWheelVelocityCallback, this);
  mWheelVelocityServer[1] = 
    nodeHandle()->advertiseService(name() + "/arabot/set_right_wheel_velocity", &RosArabot::setRightWheelVelocityCallback, this);

  // add topics
  mWheelEncoderPublisher[0] =
    nodeHandle()->advertise<webots_ros::Float64Stamped>(name() + "/arabot/get_right_wheel_encoder", 1);
  mWheelEncoderPublisher[1] =
    nodeHandle()->advertise<webots_ros::Float64Stamped>(name() + "/arabot/get_left_wheel_encoder", 1);
}

void RosArabot::setRosDevices(const char **hiddenDevices, int numberHiddenDevices) {

}

bool RosArabot::setLeftWheelVelocityCallback(webots_ros::set_float::Request &req, webots_ros::set_float::Response &res) {
  mWheelMotor[0]->setVelocity(req.value);
  return (res.success = true);
}

bool RosArabot::setRightWheelVelocityCallback(webots_ros::set_float::Request &req, webots_ros::set_float::Response &res) {
  mWheelMotor[1]->setVelocity(req.value);
  return (res.success = true);
}

int RosArabot::step(int duration) {
  // publish topics
  webots_ros::Float64Stamped value;
  value.header.stamp = ros::Time::now();
  value.header.frame_id = name() + "/arabot";

  // publish wheel odometry in meters
  for(int i=0; i<2; i++) {
    value.data = (mWheelEncoder[i]->getValue() * WHEEL_RADIUS_IN_MM) / 1000.0; 
    mWheelEncoderPublisher[i].publish(value);
  }

  // publish ultrasonic sensor distance in meters
  for(int i=0; i<3 ; i++) {
    value.data = mUltrasonicSensor[i]->getValue() / 1000.0;
    mUltrasonicSensorPublisher[i].publish(value);
  }

  return mRobot->step(duration);
}

