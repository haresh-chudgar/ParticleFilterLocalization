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

#ifndef PARTICLEFILTER_H
#define PARTICLEFILTER_H

#include <vector>
#include <iostream>
#include <sstream>
#include <fstream>
#include "robot.h"
#include "map.h"
#include "gvector.h"

class ParticleFilter
{
public:
  ParticleFilter(unsigned int noOfParticles, Map map);
  ~ParticleFilter();
  
  bool Initialize(GVector::vector3d<double> location);
  bool MotionUpdate(GVector::vector3d<double> odometry);
  bool SensorUpdate(double startAngle, double increment, double noOfScans, double *observedDistances);
  
  void WriteToFile();
  void CloseFile();
  
  void SensorUpdate_Thread(int);
private:
  
  double angularNoise, angularNoise_WheelImbalance, radialNoise, tangentialNoise, senseNoise;
  Map worldMap;
  unsigned int noOfParticles;
  std::vector<Robot*> particles;
  long double* weights;
  FILE* outputFile;
  
  unsigned int iteration;
  
  int samplingPeriod;
  int sampleIter;
  double* observedDistances_ForThread;
  double startAngle, increment, noOfScans;
};

#endif // PARTICLEFILTER_H










