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

#ifndef MAP_H
#define MAP_H

#include <vector>
#include "gvector.h"

typedef struct _LineSegment {
  GVector::vector2d<double> point1;
  GVector::vector2d<double> point2;
  
  bool isVertical;
  bool isHorizontal;
} LineSegment;

class Map
{
public:
  Map(std::vector<std::vector<GVector::vector2d<double>>> lineSegments);
  Map(const Map& other);
  ~Map();
  
  void FindNearestWalls(double startAngle, double increment, double noOfScans, GVector::vector3d<double> robotLocation, double* scannedDistances, double* observedDistances);
private:
  double FindDistanceToWall(GVector::vector3d<double> robotLocation, LineSegment &lineSegment);
  
  //vector of line segments
  //each line segment is vector of two points
  std::vector<LineSegment> map;
};

#endif // MAP_H
