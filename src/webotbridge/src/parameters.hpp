/*---------------------------------------------------
Rodrigo Munguía 2021.

System parameters
-----------------------------------------------------
*/
#include <string>
#include <vector>

#ifndef PARAMETERS_H
#define PARAMETERS_H

using namespace std;



struct parameters
{   
    
    double Sigma_roll;    
    double Sigma_pitch;
    double Sigma_yaw;

    double Sigma_Alt;
    double Sigma_Spd;
    
    vector<double> Base_location;
    double Sigma_rangetobase;

    // A8: CameraInfo published with standard images (matches localslam YAML when set).
    bool Publish_camera_info;
    int Camera_width;
    int Camera_height;
    double Camera_fc_u;
    double Camera_fc_v;
    double Camera_cc_u;
    double Camera_cc_v;
    double Camera_alpha_c;
    double Camera_k1;
    double Camera_k2;
    string Camera_optical_frame;
    

} ;   





#endif /*PARAMETERS_H*/