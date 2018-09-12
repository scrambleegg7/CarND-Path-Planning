#ifndef VEHICLE_H
#define VEHICLE_H

#include <vector>
#include "utils.h"


using namespace std;

class Vehicle {

  /* ["sensor_fusion"] A 2d vector of cars and then that car's [
  id: car's unique ID,
  x: car's x position in map coordinates,
  y: car's y position in map coordinates,
  v: car's speed,
  s: car's s position in frenet coordinates,
  d: car's d position in frenet coordinates. */

  protected:

    int id;
    int car_id;
    int event_id;
    double x;
    double y;
    double v;
    double s;
    double d;
    double yaw;

    vector<double> previous_s;
    vector<double> previous_d;

    vector<double> previous_path_x;
    vector<double> previous_path_y;
    

  public:

    Vehicle();
    Vehicle(int id, double x, double y, double v, double s, double d);
    ~Vehicle(){};

    void setEventId(int id) { event_id = id;}
    int getEventId() {return event_id;}

    int get_id();
    double get_x();
    double get_y();
    double get_v();
    double get_s();
    double get_d();
    double get_yaw();
    LANE lane();
    int lane_id();
    

    void update_vehicle_values(double x, double y, double v, double s, double d, double yaw);

    void set_previous_s(vector<double> previous_s);
    void set_previous_d(vector<double> previous_d);

    vector<double> prev_s();
    vector<double> prev_d();

    void set_previous_x(vector<double> px) {previous_path_x = px;}
    void set_previous_y(vector<double> py) {previous_path_y = py;}

    vector<double> prev_x() {return previous_path_x;}
    vector<double> prev_y() {return previous_path_y;}


    friend ostream& operator << (ostream& ostr, Vehicle &v);


};

#endif // VEHICLE_H
