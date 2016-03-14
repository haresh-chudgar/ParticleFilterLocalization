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

#include "map.h"
#include <math.h>
#include <iostream>
#include <stdio.h>

Map::Map(std::vector<std::vector<GVector::vector2d<double>>> lineSegments)
{
  for (std::vector<std::vector<GVector::vector2d<double>>>::iterator it = lineSegments.begin() ; it != lineSegments.end(); ++it) {
    std::vector<GVector::vector2d<double>> ls = *it;
    LineSegment line;
    line.point1 = ls.at(0);
    line.point2 = ls.at(1);
    if(line.point1.x == line.point2.x) {
      line.isHorizontal = true;
      line.isVertical = false;
    }
    else if(line.point1.y == line.point2.y) {
      line.isVertical = true;
      line.isHorizontal = false;
    } else {
      line.isVertical = false;
      line.isHorizontal = false;
    }
    map.push_back(line);
  }
  
}

Map::Map(const Map& other)
{
  map = other.map;

}

Map::~Map()
{

}

double Map::FindDistanceToWall(GVector::vector3d<double> robotLocation, LineSegment &lineSegment) {
  /*
  if(robotLocation.z <= M_PI_2) {
    if((lineSegment.point1.x < robotLocation.x && lineSegment.point2.x < robotLocation.x) || 
       (lineSegment.point1.y < robotLocation.y && lineSegment.point2.y < robotLocation.y))
      return -1;
  }
  else if(robotLocation.z <= M_PI) {
    if((lineSegment.point1.x > robotLocation.x && lineSegment.point2.x > robotLocation.x) || 
       (lineSegment.point1.y < robotLocation.y && lineSegment.point2.y < robotLocation.y))
      return -1;
  } else if(robotLocation.z <= 3 * M_PI_2) {
    if((lineSegment.point1.x > robotLocation.x && lineSegment.point2.x > robotLocation.x) || 
       (lineSegment.point1.y > robotLocation.y && lineSegment.point2.y > robotLocation.y))
      return -1;
  } else {
    if((lineSegment.point1.x < robotLocation.x && lineSegment.point2.x > robotLocation.x) || 
       (lineSegment.point1.y < robotLocation.y && lineSegment.point2.y > robotLocation.y))
      return -1;
  }
  */
  GVector::vector2d<double> iPoint;
  //Robot's scan line
  double m1 = tan(robotLocation.z);
  double c1 = robotLocation.y - robotLocation.x * m1;
  
  if(lineSegment.isHorizontal) {
    iPoint.x = lineSegment.point1.x;
    iPoint.y = iPoint.x * m1 + c1;
  } else if(lineSegment.isVertical) {
    iPoint.y = lineSegment.point1.y;
    iPoint.x = (iPoint.y - c1) / m1;
  } else {
    GVector::vector2d<double> diff = lineSegment.point2 - lineSegment.point1;
    
    double m2 = diff.y / diff.x;
    double c2 = lineSegment.point2.y - lineSegment.point2.x * m2;
  
    iPoint.x = (c1 - c2) / (m2 - m1);
    iPoint.y = (m2 * c1 - m1 * c2) / (m2 - m1);
  }
  
  /*Check if point is within line segment
  Line segment represented as vector AB
  C is the point
  0 <= dot(AB,AC) <= dot(AB,AB)*/
  //B - A gives vecor AB
  GVector::vector2d<double> AB = lineSegment.point2 - lineSegment.point1;
  //C - A gives vector AC
  GVector::vector2d<double> AC = iPoint - lineSegment.point1;
  //
  double dotABAB = AB.dot(AB);
  double dotABAC = AB.dot(AC);
  if(dotABAC < 0 || dotABAC > dotABAB) {
    return -1;
  }
  
  //Check if point is part of ray
  
  if(robotLocation.z <= M_PI_2) {
    if(iPoint.x < robotLocation.x || iPoint.y < robotLocation.y)
      return -1;
  }
  else if(robotLocation.z <= M_PI) {
    if(iPoint.x > robotLocation.x || iPoint.y < robotLocation.y)
      return -1;
  } else if(robotLocation.z <= 3 * M_PI_2) {
    if(iPoint.x > robotLocation.x || iPoint.y > robotLocation.y)
      return -1;
  } else {
    if(iPoint.x < robotLocation.x || iPoint.y > robotLocation.y)
      return -1;
  }
  
  return sqrt(pow(iPoint.y - robotLocation.y, 2) + pow(iPoint.x - robotLocation.x, 2));
}

void Map::FindNearestWalls(double startAngle, double increment, double noOfScans, GVector::vector3d<double> robotLocation, double* scannedDistances, double* observedDistances) {
  
  robotLocation.z += startAngle;
  for(int i = 0;i < noOfScans; i++) {
    
    if(observedDistances[i] <= 0.019) {
      scannedDistances[i] = 0;
      continue;
    }
    if(robotLocation.z < 0)
      robotLocation.z += (M_PI * 2);
    else if(robotLocation.z > (M_PI * 2))
	robotLocation.z -= (M_PI * 2);
    
    double minDistance = -1;
    for (std::vector<LineSegment>::iterator it = map.begin() ; it != map.end(); ++it) {
      double distanceToWall = FindDistanceToWall(robotLocation, *it);
      if(distanceToWall != -1 && (minDistance == -1 || distanceToWall < minDistance)) {
	minDistance = distanceToWall;
      }
    }
    scannedDistances[i] = minDistance;
    //printf("%f\n", minDistance);
    robotLocation.z += increment;
  }
}





