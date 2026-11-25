#ifndef LOCAL_SLAM_TYPES_H
#define LOCAL_SLAM_TYPES_H

/*---------------------------------------------------
Rodrigo Munguía 2020.

Get data from data base
-----------------------------------------------------
*/
#include <array>
#include <cstdint>
#include <string>
#include "opencv2/opencv.hpp"
#include <fstream>
#include <algorithm>
#include <sstream>
#include <armadillo>
//#include "parameters.hpp"

using namespace std;
using namespace cv;



// gps data
struct GPS
{
   long int time;
   double lat;
   double lon;
   double alt;
   int sat;     
};

// barometer (raw) data
struct BAR 
{
    long int time;
    double pressure;
    double temp;
};
// Altitude  data
struct ALT
{
    long int time;
    double altitude; //  (meters)
};


// Range (sonar) data
struct RANGE
{
    long int time;
    double range;
    double volt;
};

// Visual data
struct FRAME
{
    long int time;
    string image_file;
    cv::Mat image;
    double range;
    string range_type;
};

struct ATT
{
    long int time;
    float roll; //  
    float pitch;
    float yaw;
};

struct SPD
{
    long int time;
    float speedX; //  
    float speedY;
    float speedZ;
};

struct ODOD
{
    long int time;
    int TicksRight;
    int TicksLeft;
};

struct ODOV
{
    long int time;
    float linear_vel;
    float angular_vel;
    
};

struct RANGETOBASE
{
    long int time;
    double range; //  (meters)
};



// struct for storing data from sensors
struct DATA
{   
    string data_type;     
    long int time;
    GPS gps;
    BAR bar;
    RANGE range;
    FRAME frame;
    ALT alt;
    ATT att;
    SPD spd;
    ODOD odod;
    ODOV odov;
    RANGETOBASE rangetobase;
};

struct POINT3d
{
    double x;
    double y;
    double z;
};

struct LOCALSLAM_DATA
{
    vector<double> robot_state;
    arma::vec::fixed<3> t_c2r;  // camera to robot position vector
    arma::mat::fixed<3,3> Rr2c; // robot to camera rotation matrix 
    arma::vec::fixed<3> r_N; // position of the robot, expressed in the navigation frame 
    arma::mat::fixed<3,3> Rn2r; // navigation to robot rotation matrix
    vector<POINT3d> map_state;
    vector<POINT3d> map_anchors;
    vector<POINT3d> matched_img_feats;
    vector<POINT3d> unmatched_img_feats;
    vector<POINT3d> matched_img_anchors;
    vector<POINT3d> unmatched_img_anchors;
    double radius_uncertainty;

};

struct POS_UPDATE
{
    bool pos_update_available{false};
    arma::vec::fixed<3> delta_pos_update;
    string type;
    /** G6d: "ba" | "tag_loop" | "visual_loop" (empty allowed for legacy). */
    string source;
    // G3 identity captured at service admission; re-checked at EKF apply.
    std::array<uint8_t, 16> session_id{};
    uint64_t epoch_id{0};
    uint64_t keyframe_id{0};
    uint64_t correction_id{0};

    // G5/G6a: filled by EKF worker; service waits on cv (mutex_pos_update).
    bool result_ready{false};
    bool result_applied{false};
    arma::vec::fixed<3> applied_delta;
    std::string outcome;  // applied | gated_noop | reset_before_apply | cancelled | ...
    /** LC-CONSISTENCY-A3: Local generation watermark at apply outcome. */
    uint64_t result_last_generated_kf_id{0};
    /** G6a: set under mutex on service timeout while still pending — worker must not mutate. */
    bool cancel_requested{false};
};

    


#endif
