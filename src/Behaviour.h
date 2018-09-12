#ifndef BEHAVIOUR_H
#define BEHAVIOUR_H

#include <string>
#include <vector>
#include <fstream>
#include <iostream>
#include <sstream>
#include <math.h>


using namespace std;

class Behariour {

  protected:


  public:
    Behariour(){};
    Behariour(string Behavior_file_);
    ~Behariour() {};

    void setup(vector< vector<double> >  s );

};

#endif // 
