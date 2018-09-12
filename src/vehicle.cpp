#include "vehicle.h"

using namespace std;

Vehicle::Vehicle(){
  this->id = -1;
}

Vehicle::Vehicle(int id, double x, double y, double v, double s, double d){
  this->id = id;
  this->x = x;
  this->y = y;
  this->v = v;
  this->s = s;
  this->d = d;
}

/* GETTERS*/
int Vehicle::get_id(){
  return this->id;
}

double Vehicle::get_x(){
  return this->x;
}

double Vehicle::get_y(){
  return this->y;
}

double Vehicle::get_v(){
  return this->v;
}

double Vehicle::get_s(){
  return this->s;
}

double Vehicle::get_d(){
  return this->d;
}

double Vehicle::get_yaw(){
  return this->yaw;
}

LANE Vehicle::lane(){
  LANE lane;
  if (this->d < 4.0) {
    lane = LANE::LEFT;
  }
  else if ((this->d >= 4.0) && (this->d < 8.0)) {
    lane = LANE::CENTER;
  }
  else {
    lane = LANE::RIGHT;
  }
  return lane;
}

int Vehicle::lane_id(){
  int lane;
  if (this->d < 4.0) {
    lane = 0;
  }
  else if ((this->d >= 4.0) && (this->d < 8.0)) {
    lane = 1;
  }
  else {
    lane = 2;
  }
  return lane;
}


void Vehicle::update_vehicle_values(double x, double y, double v, double s, double d, double yaw){
  this->x = x;
  this->y = y;
  this->v = v;
  this->s = s;
  this->d = d;
  this->yaw = yaw;
}

void Vehicle::set_previous_s(vector<double> previous_s){
  this->previous_s = previous_s;
}

void Vehicle::set_previous_d(vector<double> previous_d){
  this->previous_d = previous_d;
}

vector<double> Vehicle::prev_s(){
  return this->previous_s;
}

vector<double> Vehicle::prev_d(){
  return this->previous_d;
}



ostream& operator<<(ostream& ostr, Vehicle &vec) {
    //ostr << "{" << v.front();
    int event_id = vec.getEventId();
    int lane_id = vec.lane_id();
    int car_id = vec.get_id();
    double x = vec.get_x();
    double y = vec.get_y();
    double speed = vec.get_v();
    double s = vec.get_s();
    double yaw = vec.get_yaw();

    vector<double> v = { (double)event_id,(double)lane_id,(double)car_id,x,y,speed,s,yaw };

    ostr << v.front();
	  for (auto itr = ++v.begin(); itr != v.end(); itr++) {
        ostr << "," << *itr;
    }
    
    //vector<double> params = {x,y};  
    //ostr << params << endl;
    //ostr << params << std::endl;
    //ostr << "LANE id:" << v.lane_id() << "car id:" << v.get_id() << ",x:" << v.get_x() << ",y:" << v.get_y() << ",speed:" << v.get_v() << ",s:" << v.get_s() << ",yaw:" << v.get_yaw(); // << std::endl;   

    return ostr;
}

