// Copyright 1996-2020 Cyberbotics Ltd.
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

#include <signal.h>
#include "ros/ros.h"

// include files to use standard message types in topic
// Webots only use basic messages type defined in ROS library
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <webots_ros/set_float.h>
#include <webots_ros/Float64Stamped.h>

static int controllerCount;
static std::vector<std::string> controllerList;
static double obstacle_distance[3] = {1000.0, 1000.0, 1000.0};   
static double traveled_distance[2] = {0.0, 0.0};

void quit(int sig) {
  ROS_INFO("User stopped the 'collision_avoidance' node.");
  ros::shutdown();
  exit(0);
}

// catch names of the controllers availables on ROS network
void controllerNameCallback(const std_msgs::String::ConstPtr &name) {
  controllerCount++;
  controllerList.push_back(name->data);
  ROS_INFO("Controller #%d: %s.", controllerCount, controllerList.back().c_str());
}

std::string getControllerName(ros::NodeHandle &n) {
  std::string controllerName;

// subscribe to the topic model_name to get the list of availables controllers
  ros::Subscriber nameSub = n.subscribe("model_name", 100, controllerNameCallback);
  while (controllerCount == 0 || controllerCount < nameSub.getNumPublishers()) {
    ros::spinOnce();
    ros::Duration(0.5).sleep();
  }
  ros::spinOnce();

  // if there is more than one controller available, let the user choose
  if (controllerCount == 1)
    controllerName = controllerList[0];
  else {
    int wantedController = 0;
    do {
      std::cout << "Choose the # of the controller you want to use:\n";
      std::cin >> wantedController;
    } while ( !(1 <= wantedController && wantedController <= controllerCount) );
    controllerName = controllerList[wantedController - 1];
  }
  // leave topic once it's not necessary anymore
  nameSub.shutdown();
  return controllerName;
}

void getWheelEncoderDistanceCallback0(const std_msgs::Float64::ConstPtr &value) {
  traveled_distance[0] = value->data;
}

void getWheelEncoderDistanceCallback1(const std_msgs::Float64::ConstPtr &value) {
  traveled_distance[1] = value->data;
}

void getUltrasonicSensorDistanceCallback0(const std_msgs::Float64::ConstPtr &value) {
  obstacle_distance[0] = value->data;
}

void getUltrasonicSensorDistanceCallback1(const std_msgs::Float64::ConstPtr &value) {
  obstacle_distance[1] = value->data;
}

void getUltrasonicSensorDistanceCallback2(const std_msgs::Float64::ConstPtr &value) {
  obstacle_distance[2] = value->data;
}

int main(int argc, char **argv) {

  ros::init(argc, argv, "collision_avoidance", ros::init_options::AnonymousName);

  signal(SIGINT, quit);

  ros::NodeHandle n;

  std::string name;// = getControllerName(n);

  ros::ServiceClient setLeftWheelVelocityService = n.serviceClient<webots_ros::set_float>(name + "/arabot/set_left_wheel_velocity");
  ros::ServiceClient setRightWheelVelocityService = n.serviceClient<webots_ros::set_float>(name + "/arabot/set_right_wheel_velocity");

  webots_ros::set_float setLeftWheelVelocityMessage;
  webots_ros::set_float setRightWheelVelocityMessage;

  ros::Subscriber getWheelEncoderDistanceSubscriber0 = n.subscribe(name + "/arabot/get_left_wheel_encoder", 1, getWheelEncoderDistanceCallback0);
  ros::Subscriber getWheelEncoderDistanceSubscriber1 = n.subscribe(name + "/arabot/get_right_wheel_encoder", 1, getWheelEncoderDistanceCallback1);

  ros::Subscriber getUltrasonicSensorDistanceSubscriber0 = n.subscribe(name + "/arabot/get_left_ultrasonic_sensor", 1, getUltrasonicSensorDistanceCallback0);
  ros::Subscriber getUltrasonicSensorDistanceSubscriber1 = n.subscribe(name + "/arabot/get_middle_ultrasonic_sensor", 1, getUltrasonicSensorDistanceCallback1);
  ros::Subscriber getUltrasonicSensorDistanceSubscriber2 = n.subscribe(name + "/arabot/get_right_ultrasonic_sensor", 1, getUltrasonicSensorDistanceCallback2);

  bool desviar_obstaculo = false;      
  while (ros::ok()) {

    ros::spinOnce();

    std::cout<<"Distância Percorrida Roda Esquerda: " << traveled_distance[0]  <<" [m]"<<std::endl;
    std::cout<<"Distância Percorrida Roda Direita: "  << traveled_distance[1]  <<" [m]"<<std::endl;
    std::cout<<" -----------------------------------------"<<std::endl;


    if ((obstacle_distance[0] < 800) || (obstacle_distance[1] < 800) || (obstacle_distance[2] < 800)) {
      setLeftWheelVelocityMessage.request.value = 2.0;
      setRightWheelVelocityMessage.request.value = -2.0;

      setLeftWheelVelocityService.call(setLeftWheelVelocityMessage);
      setRightWheelVelocityService.call(setRightWheelVelocityMessage);
    }
    else {
      setLeftWheelVelocityMessage.request.value = 2.0;
      setRightWheelVelocityMessage.request.value = 2.0;

      setLeftWheelVelocityService.call(setLeftWheelVelocityMessage);
      setRightWheelVelocityService.call(setRightWheelVelocityMessage);
    }

    /*  
    cout<<"Angle X : "<<gyro->getRollPitchYaw()[0]<<std::endl;
    cout<<"Angle Y : "<<gyro->getRollPitchYaw()[1]<<std::endl;
    cout<<"Angle Z : "<<gyro->getRollPitchYaw()[2]<<std::endl;
    cout<<" -----------------------------------------"<<endl;
    
    cout<<"Acceleration X : "<<ac->getValues()[0]<<std::endl;
    cout<<"Acceleration Y : "<<ac->getValues()[1]<<std::endl;
    cout<<"Acceleration Z : "<<ac->getValues()[2]<<std::endl;
    cout<<" -----------------------------------------"<<endl;
    */
  };

  return 0;
}
