#include <iostream>
#include <sstream>
#include <fstream>
#include <string>
#include "robot.h"
#include "map.h"
#include "gvector.h"
#include "particlefilter.h"
#include <sys/time.h>
#include <ctime>

using namespace std;

typedef long long int64; typedef unsigned long long uint64;

uint64 GetTimeMs64()
{
#ifdef _WIN32
 /* Windows */
 FILETIME ft;
 LARGE_INTEGER li;

 /* Get the amount of 100 nano seconds intervals elapsed since January 1, 1601 (UTC) and copy it
  * to a LARGE_INTEGER structure. */
 GetSystemTimeAsFileTime(&ft);
 li.LowPart = ft.dwLowDateTime;
 li.HighPart = ft.dwHighDateTime;

 uint64 ret = li.QuadPart;
 ret -= 116444736000000000LL; /* Convert from file time to UNIX epoch time. */
 ret /= 10000; /* From 100 nano seconds (10^-7) to 1 millisecond (10^-3) intervals */

 return ret;
#else
 /* Linux */
 struct timeval tv;

 gettimeofday(&tv, NULL);

 uint64 ret = tv.tv_usec;
 /* Convert from micro seconds (10^-6) to milliseconds (10^-3) */
 ret /= 1000;

 /* Adds the seconds (10^0) after converting them to milliseconds (10^-3) */
 ret += (tv.tv_sec * 1000);

 return ret;
#endif
}

int main(int argc, char **argv) {
  
  vector<vector<GVector::vector2d<double>>> lineSegments;
  std::ifstream mapFile ("../map.txt");
  std::string line;
  while(std::getline(mapFile,line))
  {
    std::stringstream  lineStream(line);
    std::string        cell;
    std::string::size_type sz;
    GVector::vector2d<double> point;

    //lineSegments.push_back();
    vector<GVector::vector2d<double>> lineSegment;

    getline(lineStream,cell,',');
    point.x = stod(cell, &sz);
    getline(lineStream,cell,',');
    point.y = stod(cell, &sz);
    lineSegment.push_back(point);

    getline(lineStream,cell,',');
    point.x = stod(cell, &sz);
    getline(lineStream,cell,',');
    point.y = stod(cell, &sz);
    lineSegment.push_back(point);

    lineSegments.push_back(lineSegment);
  }
  
  Map worldMap(lineSegments);
  ParticleFilter filter(300, worldMap);
  double *observedDistances = NULL;
  
  //Testing map
  //double* scannedDistances = new double[768];
  //worldMap.FindNearestWalls(-2.356194, 0.006136, 768, GVector::vector3d<double>(56.256073 + 0.017315, 53.107269-0.005103, 1.583401-0.024967), scannedDistances);
  //delete[] scannedDistances;
  
  int loopCount = 0;
  std::ifstream logFile ("../robot-data.log");
  
  double prevOdometryX = 0, prevOdometryY = 0, prevOdometryAngle = 0;
  
  while(std::getline(logFile,line))
  {
    ++loopCount;
    if(loopCount == 122 || loopCount == 123) {
      printf("what is the error? /n");
    }
    std::stringstream  lineStream(line);
    std::string        cell;
    std::string::size_type sz;

    getline(lineStream,cell,' ');
    if((cell.c_str())[0] == 'I') {

      uint64_t startTime = GetTimeMs64();
      GVector::vector3d<double> initPoint;
      getline(lineStream,cell,' ');
      initPoint.x = stod(cell, &sz);
      getline(lineStream,cell,' ');
      initPoint.y = stod(cell, &sz);
      getline(lineStream,cell,' ');
      initPoint.z = stod(cell, &sz);
      
      printf("Set Initial location as: %f. %f, %f\n", initPoint.x, initPoint.y, initPoint.z);
      filter.Initialize(initPoint);
      
      uint64_t endTime = GetTimeMs64();
      printf("Init Time: %lu ms\n", endTime - startTime);
      
    } 
    else if(cell.c_str()[0]  == 'L') {
      
      uint64_t startTime = GetTimeMs64();
      
      getline(lineStream,cell,' ');
      double timestamp = stod(cell, &sz);
      
      getline(lineStream,cell,' ');
      int noOfScans = stoi(cell, &sz);
      
      getline(lineStream,cell,' ');
      double startAngle = stod(cell, &sz);
      
      getline(lineStream,cell,' ');
      double endAngle = stod(cell, &sz);
      
      getline(lineStream,cell,' ');
      double increment = stod(cell, &sz);
      
      if(observedDistances == NULL)
	observedDistances = new double[noOfScans];
      
      for(int scan = 0; scan < noOfScans; scan++) {
	getline(lineStream,cell,' ');
	observedDistances[scan] = stod(cell, &sz);
      }
      
      filter.SensorUpdate(startAngle, increment, noOfScans, observedDistances);
      filter.WriteToFile();
      
      uint64_t endTime = GetTimeMs64();
      printf("Sensor Update Time: %lf s\n", (endTime - startTime) / 1000.0);
      
    } 
    else if(cell.c_str()[0] == 'O') {
      
      uint64_t startTime = GetTimeMs64();
      
      getline(lineStream,cell,' ');
      double timestamp = stod(cell, &sz);
      
      double oX, oY, oAngle;
      GVector::vector3d<double> odometry;
      getline(lineStream,cell,' ');
      oX = stod(cell, &sz);
      getline(lineStream,cell,' ');
      oY = stod(cell, &sz);
      getline(lineStream,cell,' ');
      oAngle = stod(cell, &sz);
      //printf("Odometry: %f, %f, %f\n", oX, oY, oAngle);
      if(oX == 0 && oY == 0 && oAngle == 0) {
	prevOdometryX = oX;
	prevOdometryY = oY;
	prevOdometryAngle = oAngle;
	continue;
      } else if(oX == prevOdometryX && oY == prevOdometryY && oAngle == prevOdometryAngle) {
	continue;
      }
      odometry.x = oX - prevOdometryX;
      odometry.y = oY - prevOdometryY;
      odometry.z = oAngle - prevOdometryAngle;
      //printf("Change: %f, %f, %f\n", odometry.x, odometry.y, odometry.z);
      prevOdometryX = oX;
      prevOdometryY = oY;
      prevOdometryAngle = oAngle;
      
      filter.MotionUpdate(odometry);
      //filter.WriteToFile();

      uint64_t endTime = GetTimeMs64();
      printf("Predict Time: %lu ms\n", endTime - startTime);

    }
    
  }
  
}












































