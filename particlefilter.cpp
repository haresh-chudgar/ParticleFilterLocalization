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

#include "particlefilter.h"
#include <ctime>
#include <stdio.h>
#include <float.h>
#include <thread>

#define LOW_VARIANCE_RESAMPLING 0
#define CIRCULAR_RESAMPLING 1
#define THREADED 1

ParticleFilter::ParticleFilter(unsigned int noOfParticles, Map map):
noOfParticles(noOfParticles), worldMap(map)
{
  weights = new long double[noOfParticles];
    
  angularNoise = 1.0;
  angularNoise_WheelImbalance = 0.1;
  radialNoise = 0.2;
  tangentialNoise = 0.2;
  senseNoise = 0.5;
  
  samplingPeriod = 20;
  sampleIter = 0;
  
  iteration = 0;
  setbuf(stdout, NULL);
}

ParticleFilter::~ParticleFilter()
{
  delete[] weights;

}

bool ParticleFilter::Initialize(GVector::vector3d<double> location) {
  for(int i = 0; i < noOfParticles; i++) {
    particles.push_back(new Robot(angularNoise, angularNoise_WheelImbalance, radialNoise, tangentialNoise, senseNoise, location));
  }
}

bool ParticleFilter::MotionUpdate(GVector::vector3d<double> odometry) {
  for(std::vector<Robot*>::iterator it = particles.begin(); it != particles.end(); ++it){
    (*it)->move(odometry);
  }
}

void ParticleFilter::SensorUpdate_Thread(int threadId) {
  int index = threadId;
  //printf("SensorUpdate_Thread Start %d\n", index);
  weights[index] = particles.at(index)->ComputeObservationProb(worldMap, startAngle, increment, noOfScans, observedDistances_ForThread);
  //printf("SensorUpdate_Thread End %d, %Le\n", index, weights[index]);
}

#define PRINT_WEIGHTS 0
bool ParticleFilter::SensorUpdate(double startAngle, double increment, double noOfScans, double *observedDistances) {
  
  sampleIter = sampleIter + 1;
  if(sampleIter % samplingPeriod != 0) {
    return true;
  }
  sampleIter = 0;
  
  int iLoop = 0;
  std::vector<std::thread> threads;
  
  long double sumOfWeights = 0;
  observedDistances_ForThread = observedDistances;
  this->startAngle = startAngle;
  this->increment = increment;
  this->noOfScans = noOfScans;
  
  for(std::vector<Robot*>::iterator it = particles.begin(); it != particles.end(); ++it){
    //printf("Thread %d\n", iLoop);
#if THREADED
    threads.push_back(std::thread(&ParticleFilter::SensorUpdate_Thread, this, iLoop));
#else
    weights[iLoop] = particles.at(iLoop)->ComputeObservationProb(worldMap, startAngle, increment, noOfScans, observedDistances_ForThread);
#endif
    
    ++iLoop;
#if 0
    printf("%Le\n", weights[iLoop]);
#endif
  }
#if THREADED  
  for(std::vector<std::thread>::iterator it = threads.begin(); it != threads.end(); ++it){
    (*it).join();
  }
#endif
  //printf("All threads done!\n");
  
  sumOfWeights = 0;
  for(int i = 0; i < noOfParticles; i++) {
    sumOfWeights += weights[i];
    //printf("%d, %Le %Le\n", i, weights[i], sumOfWeights);
  }
  
  long double maxWeight = 0;
  for(int i = 0; i < noOfParticles; i++) {
    weights[i] /= sumOfWeights;
    particles[i]->setWeight(weights[i]);
    if(maxWeight < weights[i])
      maxWeight = weights[i];
#if PRINT_WEIGHTS
    printf("%d, %Le\n", i, weights[i]);
#endif
  }

  //Resample
#if LOW_VARIANCE_RESAMPLING
  std::vector<Robot*> tempParticles;
  srand(std::time(NULL));
  int r = int( ((double)rand() / RAND_MAX) * noOfParticles);
  
  for(int i = 0; i < noOfParticles; i++) {
#if PRINT_WEIGHTS
   printf("%d %Le\n", r, weights[r]);
#endif
    tempParticles.push_back(new Robot(particles[r]));
    r = (int)(r + i * maxWeight / noOfParticles) % noOfParticles;
  }
  for(int i = 0; i < noOfParticles; i++) {
    Robot* r = particles.back();
    delete r;
    particles.pop_back();
    weights[i] = (1 / noOfParticles);
  }
  particles = tempParticles;

#endif
  
#if CIRCULAR_RESAMPLING
  std::vector<Robot*> tempParticles;
  srand(std::time(NULL));
  unsigned int index = (unsigned int)(((double)rand() / RAND_MAX) * noOfParticles);
  double beta = 0.0;
  
  for(int i = 0; i < noOfParticles; i++) {
    srand(std::time(NULL));
    beta += ((double)rand() / RAND_MAX) * 2.0 * maxWeight;
    while (beta > weights[index]) {
      beta -= weights[index];
      index = (index + 1) % noOfParticles;
    }
#if PRINT_WEIGHTS
    printf("%d %Le\n", index, weights[index]);
#endif
    if(i == 0)
      printf("Angle: %f\n", particles[index]->getLocation().z);
    tempParticles.push_back(new Robot(particles[index]));
  }
  /*
  // sort using a custom function object
  struct {
      bool operator()(Robot* a, Robot* b)
      {   
	  return a->getWeight() > b->getWeight();
      }   
  } compareParticles;
  std:sort(particles.begin(), particles.end(), compareParticles);
  
  for(int iLoop = 0; iLoop < noOfParticles / 2; iLoop++) {
    tempParticles.push_back(new Robot(particles[iLoop]));
  }
  */
  for(int i = 0; i < noOfParticles; i++) {
    Robot* r = particles.back();
    delete r;
    particles.pop_back();
    weights[i] = (1 / noOfParticles);
  }
  particles = tempParticles;
#endif

  return true;
}

void ParticleFilter::WriteToFile()
{
  printf("Writing to file for iteration: %u\n", iteration);
  outputFile = fopen("particleFilteroutput_v0_Full_CyclicResampling.csv", "a");
  
  for(std::vector<Robot*>::iterator it = particles.begin(); it != particles.end(); ++it){
    GVector::vector3d<double> location = (*it)->getLocation();
    fprintf(outputFile, "%f, %f, %f\n", location.x, location.y, location.z);
  }
  fprintf(outputFile, "\n");
  fflush(outputFile);
  
  fclose(outputFile);
  
  printf("Iteration %u end\n", iteration);
  iteration += 1;
}

void ParticleFilter::CloseFile() {
  
}














