#ifndef BEHAVIOUR_H
#define BEHAVIOUR_H

#include <string>
#include <vector>
#include <fstream>
#include <iostream>
#include <sstream>
#include <math.h>

#include "vehicle.h"

using namespace std;

class Behariour {

  private:

    int lane;
    double speed_diff;

  protected:


  public:
    Behariour(){};
    Behariour(string Behavior_file_);
    ~Behariour() {};

    void setup(vector< vector<double> >  s,  Vehicle c, double ref );

    int getlane() {return lane;}
    double get_speed_diff() {return speed_diff;}

};

#endif // 
