#ifndef UTILS_H
#define UTILS_H

#include <fstream>
#include <math.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include <string>

using namespace std;

#define INF 1e10

const double TRACK_DISTANCE = 6945.554; //m
const double ROAD_WIDTH = 12.0; //m
const double SPEED_LIMIT = 20.0; //m/s
const double AT = 0.02; //s
const double CYCLES = 2;
const double POINTS = 50;
const double SAFETY_DISTANCE = 40.0; //m
const double GUARD_DISTANCE = 20.0; //m

const double PARAM_MAX_SPEED_MPH = 49;
const double PARAM_MAX_SPEED = 22; // m.s-1
const double PARAM_MAX_ACCEL = 10; // m.s-2
const double PARAM_MAX_JERK  = 10; // m.s-3 average jerk over 1 second



enum class LANE { LEFT, CENTER, RIGHT };
enum class STATE { START, KEEP_LANE, CHANGE_LEFT, CHANGE_RIGHT };

const double PARAM_FOV = 70.0; // Field Of View
const double MAX_S = 6945.554;
//extern double MAX_S;


//template <typename _Ty>
//ostream& operator << (ostream& ostr, const vector<_Ty>& v);


#endif // UTILS_H
