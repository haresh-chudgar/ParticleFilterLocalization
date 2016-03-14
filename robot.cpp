/*
 * Copyright 2016 <copyright holder> <email>
 * 
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * 
 *     http://www.apache.org/licenses/LICENSE-2.0
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 * 
 */

#include "robot.h"
#include <random>
#include <ctime>
#include <stdio.h>
#include <math.h>

Robot::Robot(double angularNoise, double angularNoise_WheelImbalance, double radialNoise, double tangentialNoise, double senseNoise)
:angularNoise(angularNoise), angularNoise_WheelImbalance(angularNoise_WheelImbalance), radialNoise(radialNoise), tangentialNoise(tangentialNoise), senseNoise(senseNoise)
{
  obstacleNoise = 0.25 / (sqrt(2 * M_PI) * senseNoise);
}

Robot::Robot(double angularNoise, double angularNoise_WheelImbalance, double radialNoise, double tangentialNoise, double senseNoise, GVector::vector3d<double> location)
:angularNoise(angularNoise), angularNoise_WheelImbalance(angularNoise_WheelImbalance), radialNoise(radialNoise), tangentialNoise(tangentialNoise), senseNoise(senseNoise)
{
  setLocation(location);
}

Robot::Robot(const Robot& other)
{
  myLocation = other.myLocation;
  angularNoise = other.angularNoise;
  radialNoise = other.radialNoise;
  tangentialNoise = other.tangentialNoise;
  senseNoise = other.senseNoise;
  weight = other.weight;
}

Robot::Robot(const Robot* other)
{
  myLocation.x = other->myLocation.x;
  myLocation.y = other->myLocation.y;
  myLocation.z = other->myLocation.z;
  
  angularNoise = other->angularNoise;
  radialNoise = other->radialNoise;
  tangentialNoise = other->tangentialNoise;
  senseNoise = other->senseNoise;
  
  weight = other->weight;
}

Robot::~Robot()
{
}

void Robot::setLocation(GVector::vector3d<double> newLoc) {
  
  //TODO: Check for correctness
  /*
  if newLoc < 0 or newLoc >= self.world_size
    raise ValueError, 'X coordinate out of bound'
  if new_y < 0 or new_y >= self.world_size:
    raise ValueError, 'Y coordinate out of bound'
  if new_orientation < 0 or new_orientation >= 2 * math.pi:
    raise ValueError, 'Orientation must be in [0..2pi]'
  */
  myLocation = newLoc;
}

void Robot::move(GVector::vector3d<double> delta) {
  
  double radialError = Gaussian(0.0, radialNoise * delta.x);
  double tangentialError = Gaussian(0.0, tangentialNoise * delta.y);
  double temp1 = Gaussian(0.0, angularNoise * delta.z);
  double radialTranslation = pow(pow(delta.x,2) + pow(delta.y,2), 0.5);
  //double temp2 = Gaussian(0.0, angularNoise_WheelImbalance * radialTranslation);
  double angularError = temp1;// + temp2;
  delta.x += radialError;
  delta.y += tangentialError;
  
  double changeX = delta.x * cos(myLocation.z + delta.z) - delta.y * sin(myLocation.z + delta.z);
  double changeY = delta.x * sin(myLocation.z + delta.z) + delta.y * cos(myLocation.z + delta.z);
  myLocation.x += changeX;
  myLocation.y += changeY;
  myLocation.z += delta.z + angularError;
  
  if(myLocation.z < 0) 
  {
    myLocation.z += (M_PI * 2);\
    printf("Angle: %f\n", myLocation.z);
    printf("********************************************** <0 *****************************************\n");
  } else if(myLocation.z > (M_PI * 2)) {
    myLocation.z -= (M_PI * 2);
    printf("Angle: %f\n", myLocation.z);
    printf("********************************************** >360 *****************************************\n");
  }
}

long double Robot::getWeight()
{
  return weight;
}

void Robot::setWeight(long double weight)
{
  this->weight = weight;
}

double Robot::ComputeObservationProb(Map map, double startAngle, double increment, double noOfScans, double *measurements) {
  
  double _distanceToWalls[768];
  map.FindNearestWalls(startAngle, increment, noOfScans, myLocation, _distanceToWalls, measurements);
  long double obsProb = 1;
  for(int scan = 0; scan < noOfScans; scan++) {
    double observedDistance = measurements[scan];
    double distance = _distanceToWalls[scan];
    
    if(distance <= 0.019) {
      continue;
    }
    //Sensor noise
    long double prob = GaussianProbability(observedDistance, distance, senseNoise);
    
    //Obstacle
    if(observedDistance < distance) {
      prob += obstacleNoise * exp(-obstacleNoise*observedDistance);
    }
    
    //Random
    prob += 0.1;

    //TODO: Max range
#if 0
    printf("%f,%f,%f\n",observedDistance, distance, prob);
#endif
    obsProb *= prob;
  }
  //obsProb += noOfScans * log(M_SQRT1_2 * M_2_SQRTPI / 2);
  //printf("Observed Probability: %Le\n",obsProb);
  setWeight(obsProb);
  return obsProb;
}

double Robot::Gaussian(double mu, double sigma) {
 
  std::default_random_engine generator(std::random_device{}());
  std::normal_distribution<double> distribution(mu, sigma);
  
  return distribution(generator);
}

long double Robot::GaussianProbability(double x, double mu, double sigma) {
  
  return exp(-pow(x - mu, 2) / (2 * senseNoise * senseNoise)) / (sqrt(2.0 * M_PI) * sigma);
  
  double exponent = -pow(x - mu, 2);
  exponent /= (pow(sigma,2) * 2.0);
  return exp(exponent) / sqrt(2.0 * M_PI * pow(sigma, 2));
}

GVector::vector3d<double> Robot::getLocation() {
  return myLocation;
}










