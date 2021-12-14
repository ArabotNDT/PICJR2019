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

static unsigned int controllerCount;
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
  while (controllerCount == 0 || (controllerCount < nameSub.getNumPublishers()) ) {
    ros::spinOnce();
    ros::Duration(0.5).sleep();
  }
  ros::spinOnce();

  // if there is more than one controller available, let the user choose
  if (controllerCount == 1)
    controllerName = controllerList[0];
  else {
    unsigned int wantedController = 0;
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

void getLeftWheelEncoderDistanceCallback(const std_msgs::Float64::ConstPtr &value) {
  traveled_distance[0] = value->data;
}

void getRightWheelEncoderDistanceCallback(const std_msgs::Float64::ConstPtr &value) {
  traveled_distance[1] = value->data;
}

void getLeftUltrasonicSensorDistanceCallback(const std_msgs::Float64::ConstPtr &value) {
  obstacle_distance[0] = value->data;
}

void getMiddleUltrasonicSensorDistanceCallback(const std_msgs::Float64::ConstPtr &value) {
  obstacle_distance[1] = value->data;
}

void getRightUltrasonicSensorDistanceCallback(const std_msgs::Float64::ConstPtr &value) {
  obstacle_distance[2] = value->data;
}

int main(int argc, char **argv) {

  ros::init(argc, argv, "collision_avoidance", ros::init_options::AnonymousName);

  signal(SIGINT, quit);

  ros::NodeHandle n;

  std::string name;// = getControllerName(n);

  ros::Publisher setLeftWheelVelocityPublisher = n.advertise<std_msgs::Float64>(name + "/arabot/set_left_wheel_velocity",1);
  ros::Publisher setRightWheelVelocityPublisher = n.advertise<std_msgs::Float64>(name + "/arabot/set_right_wheel_velocity",1);

  ros::Subscriber getLeftWheelEncoderDistanceSubscriber = n.subscribe(name + "/arabot/get_left_wheel_encoder", 1, getLeftWheelEncoderDistanceCallback);
  ros::Subscriber getRightWheelEncoderDistanceSubscriber = n.subscribe(name + "/arabot/get_right_wheel_encoder", 1, getRightWheelEncoderDistanceCallback);

  ros::Subscriber getLeftUltrasonicSensorDistanceSubscriber = n.subscribe(name + "/arabot/get_left_ultrasonic_sensor", 1, getLeftUltrasonicSensorDistanceCallback);
  ros::Subscriber getMiddleUltrasonicSensorDistanceSubscriber = n.subscribe(name + "/arabot/get_middle_ultrasonic_sensor", 1, getMiddleUltrasonicSensorDistanceCallback);
  ros::Subscriber getRightUltrasonicSensorDistanceSubscriber = n.subscribe(name + "/arabot/get_right_ultrasonic_sensor", 1, getRightUltrasonicSensorDistanceCallback);

  bool desviar_obstaculo = false;      
  while (ros::ok()) {

    ros::spinOnce();

    std::cout<<"Distância Percorrida Roda Esquerda: " << traveled_distance[0]  <<" [m]"<<std::endl;
    std::cout<<"Distância Percorrida Roda Direita: "  << traveled_distance[1]  <<" [m]"<<std::endl;
    std::cout<<" -----------------------------------------"<<std::endl;


    std_msgs::Float64 leftWheelVelocityMessage;
    std_msgs::Float64 rightWheelVelocityMessage;
    if ((obstacle_distance[0] < 800) || (obstacle_distance[1] < 800) || (obstacle_distance[2] < 800)) {
      leftWheelVelocityMessage.data = 2.0;
      rightWheelVelocityMessage.data = -2.0;
    }
    else {
      leftWheelVelocityMessage.data = 2.0;
      rightWheelVelocityMessage.data = 2.0;
    }
    setLeftWheelVelocityPublisher.publish(leftWheelVelocityMessage);
    setRightWheelVelocityPublisher.publish(rightWheelVelocityMessage);

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
