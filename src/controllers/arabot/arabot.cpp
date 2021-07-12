#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/InertialUnit.hpp>
#include <webots/Accelerometer.hpp>
#include <webots/Camera.hpp>

using namespace webots;


#define TIME_STEP 13
#define MAX_SPEED 6.28

#include <iostream>
#include <string>
using namespace std;



int main(int argc, char **argv) {
  Robot *robot = new Robot();

  Motor *leftMotor = robot->getMotor("motor_left");
  Motor *rightMotor = robot->getMotor("motor_right");
  
  leftMotor->setPosition(INFINITY);
  rightMotor->setPosition(INFINITY);
  
  leftMotor->setVelocity(0.0);
  rightMotor->setVelocity(0.0);
  
  PositionSensor *leftPS = robot->getPositionSensor("encolder_left");
  PositionSensor *rightPS = robot->getPositionSensor("encolder_right");
  
  leftPS->enable(TIME_STEP);
  rightPS->enable(TIME_STEP);
  
  DistanceSensor *ultrassom[3];
  for(int i=0; i<3 ; i++)
  {
    ultrassom[i] = robot->getDistanceSensor("ultrassom" + to_string(i));
    ultrassom[i]->enable(TIME_STEP);
  }
  
  InertialUnit *gyro;
  gyro=robot->getInertialUnit("gyroscopio");
  gyro-> enable(TIME_STEP);
  
  Accelerometer *ac;
  ac=robot->getAccelerometer("accelerometer");
  ac-> enable(TIME_STEP);
  
  Camera *cm;
  cm=robot->getCamera("cam");
  cm-> enable(TIME_STEP);
  
  
  bool desviar_obstaculo = 0; 
  
    
  // Main loop:
  while (robot->step(TIME_STEP) != -1) {
  
     double raio = 35; // raio da roda em mm
    
     float rps_value_left = leftPS->getValue();
     float distance_left =  (rps_value_left * raio) / 1000;
     
     cout<<"Distância Percorrida Roda Esquerda: "<<distance_left<<" [m]"<<endl;
     

     float rps_value_right = rightPS->getValue();
     float distance_right =  (rps_value_right * raio) / 1000;
     
     cout<<"Distância Percorrida Roda Direita: "<<distance_right<<" [m]"<<endl;
     cout<<" -----------------------------------------"<<endl;

     // Desviar Objetos
     if(desviar_obstaculo > 0)
      {
        desviar_obstaculo = desviar_obstaculo - 1;
        leftMotor->setVelocity(2.0);
        rightMotor->setVelocity(-2.0);
      
      }
      else
      {
        double distancia_ultrassom[3];   
        for(int i=0; i<3 ; i++)
        {
           distancia_ultrassom[i] = ultrassom[i]->getValue();
        }
      
        
        if((distancia_ultrassom[0] < 800.0)||(distancia_ultrassom[1] < 800.0)||(distancia_ultrassom[2] < 800.0))
        {
          desviar_obstaculo = 100;
        }
        
        else
        {
          leftMotor->setVelocity(2.0);
          rightMotor->setVelocity(2.0);
        }
        
      }
      
    cout<<"Angle X : "<<gyro->getRollPitchYaw()[0]<<std::endl;
    cout<<"Angle Y : "<<gyro->getRollPitchYaw()[1]<<std::endl;
    cout<<"Angle Z : "<<gyro->getRollPitchYaw()[2]<<std::endl;
    cout<<" -----------------------------------------"<<endl;
    
    cout<<"Acceleration X : "<<ac->getValues()[0]<<std::endl;
    cout<<"Acceleration Y : "<<ac->getValues()[1]<<std::endl;
    cout<<"Acceleration Z : "<<ac->getValues()[2]<<std::endl;
    cout<<" -----------------------------------------"<<endl;
    
  };

  delete robot;
  return 0;
}