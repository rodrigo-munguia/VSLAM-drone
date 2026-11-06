#ifndef CONTROL_TYPES_H
#define CONTROL_TYPES_H

/*---------------------------------------------------
Rodrigo Munguía 2023.


-----------------------------------------------------
*/
#include <limits>
//#include "parameters.hpp"

using namespace std;
using namespace cv;



// commands structure
struct CMD
{
    string name;
    vector<double> args;     
};


struct ROBOT_STATE
{
    double x{0};
    double y{0};
    double z{0};
    double roll{0};
    double pitch{0};
    double yaw{0};
    double vx{0};
    double vy{0};
    double vz{0};
    double vyaw{0};
    // Loop-closure / explore radius from Robotstate.radius_uncertainty.
    // NaN = unavailable (odometry feedback cannot supply this field).
    double r_u{std::numeric_limits<double>::quiet_NaN()};
};







#endif
