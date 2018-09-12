#ifndef MAP_H
#define MAP_H

#include <string>
#include <vector>
#include <fstream>
#include <iostream>
#include <sstream>
#include <math.h>

#include "spline.h"

using namespace std;
using namespace tk; // spline

class Map {

  protected:
    /* define wp spline trajectory
    spline wp_spline_x;
    spline wp_spline_y;
    spline wp_spline_dx;
    spline wp_spline_dy;
    */
    double calcSpline(vector<double>  xx, vector<double>  yy, double s);

    // members to save waypoints from highway waypoints file
    vector<double> map_waypoints_x;
    vector<double> map_waypoints_y;
    vector<double> map_waypoints_s;
    vector<double> map_waypoints_dx;
    vector<double> map_waypoints_dy;


  public:
    Map(){};
    Map(string map_file_);
    ~Map() {};

    vector<double> getXY(double s, double d);

};

#endif // ROAD_H
