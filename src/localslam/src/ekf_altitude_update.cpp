#include "ekf.hpp"
/*
% Initial States ***************************************************
% x meaning
% index  0    1   2    3   4   5   6    7  8  9  10  11  12  
%       null phi theta psi w_x w_y w_z  x  y  z  v_x v_y v_z 
% Attitude states
% x(0) not used
% x(1:3)=   [phi theta psi] -> roll, pitch, and yaw of the robot
% x(4:6)=   [w_x w_y w_z ] ->  vel rotation in the body frame
% Position states
% x(7:9)= [x  y  z]  ->  Position in the navigation coordinate frame
% (navigation to camera)
% x(10:12)= [v_x v_y v_z]  -> Velocity in body frame.
*/


bool EKF::Altitude_update(ALT &alt)
{
  bool newly_latched = false;
  const double measured_alt = alt.altitude * PAR.Alt_h_2_d;

  if (!alt_home_ready_) {
    alt_at_home_ = measured_alt;
    alt_home_time_ms_ = alt.time;
    alt_home_ready_ = true;
    alt_home_latched_event_ = true;
    newly_latched = true;
  }

  const double z = measured_alt - alt_at_home_;

  const int x_len = static_cast<int>(x.size());
  arma::mat H;
  H.resize(1, x_len);
  H(0, 9) = 1;

  const double h = x(9);
  const double r = PAR.Sigma_alt_update;

  arma::mat H_P = H * P;
  arma::mat S = H_P * H.t() + r * r;
  arma::mat K = P * H.t() * arma::inv(S);

  P = P - K * H_P;
  x = x + K * (z - h);

  return newly_latched;
}

void EKF::reset_altitude_home()
{
  alt_home_ready_ = false;
  alt_at_home_ = 0.0;
  alt_home_time_ms_ = 0;
  alt_home_latched_event_ = false;
}

bool EKF::consume_altitude_home_latched_event()
{
  if (!alt_home_latched_event_) {
    return false;
  }
  alt_home_latched_event_ = false;
  return true;
}
