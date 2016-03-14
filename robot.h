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

#ifndef ROBOT_H
#define ROBOT_H

#include "gvector.h"
#include "map.h"

class Robot
{
public:
  Robot(double angularNoise, double angularNoise_WheelImbalance, double radialNoise, double tangentialNoise, double senseNoise);
  Robot(double angularNoise, double angularNoise_WheelImbalance, double radialNoise, double tangentialNoise, double senseNoise, GVector::vector3d<double> location);
  Robot(const Robot& other);
  Robot(const Robot* other);
  ~Robot();

  //Robot& operator=(const Robot& other);
  
  
  void setLocation(GVector::vector3d<double> newLoc);
  void move(GVector::vector3d<double> delta);
  double ComputeObservationProb(Map map, double startAngle, double increment, double noOfScans, double *measurements);
  GVector::vector3d<double> getLocation();

  void setWeight(long double weight);
  long double getWeight();
  
private:
  long double GaussianProbability(double x, double mu, double sigma);
  double Gaussian(double mu, double sigma);
  
  GVector::vector3d<double> myLocation;
  double angularNoise, angularNoise_WheelImbalance, radialNoise, tangentialNoise, senseNoise, obstacleNoise;
  
  long double weight;
};

#endif // ROBOT_H
