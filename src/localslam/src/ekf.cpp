#include "ekf.hpp"
#include "ls_pos_apply_policy.hpp"
#include "dynamic_diag.hpp"
#include "../../common/Transforms/Euler_to_Ra2b.hpp"
#include "../../common/Transforms/Ra2b_TO_Quat_a2b.hpp"
#include "../../common/Transforms/AngleWrap.hpp"


typedef std::vector<double> vec_t;


void EKF::system_init(DATA &dat)
{

    
       if(PAR.Attitude_update == true)
       {
          if (dat.data_type == "attitude")
          {
              //init_quat_system_state(dat);
              init_euler_system_statet(dat);
              initialized = true;
          }       
       }
       else
       {
             init_euler_system_statet(dat);
             initialized = true;
       }

    

}
//----------------------------------------------
void EKF::init_euler_system_statet(DATA &dat)
{
  
  x.resize(PAR.Robot_state_size);
  P.resize(PAR.Robot_state_size,PAR.Robot_state_size);
  x.zeros();
  P.zeros();
  Pc.zeros();
  Cl_radius_uncertainty = 0;
  if(PAR.CL_Compute_Cp_from_start == true)
    Compute_Pc_flag = true;
  else
    Compute_Pc_flag = false;

  yaw_at_home = dat.att.yaw;
  AngleWrap(yaw_at_home);
  reset_altitude_home();

  double axis_x = PAR.Robot_init_roll ;
  double axis_y = PAR.Robot_init_pitch ;
  double axis_z = PAR.Robot_init_yaw;
  //double axis_z = yaw_at_home;
  double x_init = PAR.Robot_init_x;
  double y_init = PAR.Robot_init_y;
  double z_init = PAR.Robot_init_z;

  x(0) = 0; 
  x(1) = axis_x;
  x(2) = axis_y;
  x(3) = axis_z;
  x(4) = 0;
  x(5) = 0;
  x(6) = 0;
  x(7) = x_init;
  x(8) = y_init;
  x(9) = z_init;
  x(10) = 0;
  x(11) = 0;
  x(12) = 0;

  //cout << "x_0 norm " << arma::norm(x) << endl;
  cout << "-> robot state vector initialized " << endl;

  //P = eye(PAR.Robot_state_size,PAR.Robot_state_size)*numeric_limits<double>::epsilon();
  double eps = numeric_limits<double>::epsilon()  ;
  arma::vec::fixed<13> p_ini = {0, .01, .01, .01, .01, .01, .01, eps, eps, eps, .0001, .0001, .0001};
  // E5e-A3a: assign intended initial variances (matches quat init path).
  // p_ini entries are variances (rad^2 / (m/s)^2 / …), length 13 ≡ Robot_state_size.
  P = arma::diagmat(p_ini);

  //-------------------------------------------------------------------------------  
  cam_parameters.distortions = &PAR.Mono_cam_distortions[0];
  cam_parameters.cc[0] = PAR.Mono_cam_cc_u;
  cam_parameters.cc[1] = PAR.Mono_cam_cc_v;
  cam_parameters.fc[0] = PAR.Mono_cam_fc_u;
  cam_parameters.fc[1] = PAR.Mono_cam_fc_v;
  cam_parameters.alpha_c = PAR.Mono_cam_alpha_c;
         
  t_c2r(0) = PAR.Mono_cam_2_robot_pos_x;
  t_c2r(1) = PAR.Mono_cam_2_robot_pos_y;
  t_c2r(2) = PAR.Mono_cam_2_robot_pos_z;        

  double cam_axis_x = PAR.Mono_cam_2_robot_axis_x;
  double cam_axis_y = PAR.Mono_cam_2_robot_axis_y;
  double cam_axis_z = PAR.Mono_cam_2_robot_axis_z;
  double Ra2b_c[9];
  Euler_to_Ra2b_colum_major(cam_axis_x, cam_axis_y, cam_axis_z, Ra2b_c);
  arma::mat Rr2c_t(Ra2b_c,3,3); // camera to robot rotation matrix 
  Rr2c = Rr2c_t;
  //-------------------------------------------------------------------------------- 

  FeatsDATA.clear();
  AnchorsDATA.clear();
  NewKF_available = false;
  NewRobotState_available = false;
  New_KF_cl = false;

 
 
}
//----------------------------------------------------------
void EKF::init_quat_system_state(DATA &dat)
{ 
  x.resize(PAR.Robot_state_size);
  P.resize(PAR.Robot_state_size,PAR.Robot_state_size);
  x.zeros();
  P.zeros();

  Pc.zeros();
  Cl_radius_uncertainty = 0;
  if(PAR.CL_Compute_Cp_from_start == true)
    Compute_Pc_flag = true;
  else
    Compute_Pc_flag = false;

  yaw_at_home = dat.att.yaw;
  AngleWrap(yaw_at_home);
  reset_altitude_home();

  double axis_x = PAR.Robot_init_roll ;
  double axis_y = PAR.Robot_init_pitch ;
  double axis_z = PAR.Robot_init_yaw;
  //double axis_z = yaw_at_home;
  double x_init = PAR.Robot_init_x;
  double y_init = PAR.Robot_init_y;
  double z_init = PAR.Robot_init_z;
  double Ra2b[9];

  Euler_to_Ra2b_colum_major(axis_x, axis_y, axis_z, Ra2b);    

  double q_int[4];
  Ra2b_TO_Quat_a2b(Ra2b,q_int);

  x(0) = q_int[0];
  x(1) = q_int[1];
  x(2) = q_int[2];
  x(3) = q_int[3];
  x(4) = numeric_limits<double>::epsilon();
  x(5) = numeric_limits<double>::epsilon();
  x(6) = numeric_limits<double>::epsilon();
  x(7) = x_init;
  x(8) = y_init;
  x(9) = z_init;
  x(10) = numeric_limits<double>::epsilon();
  x(11) = numeric_limits<double>::epsilon();
  x(12) = numeric_limits<double>::epsilon();

  //P = eye(PAR.Robot_state_size,PAR.Robot_state_size)*numeric_limits<double>::epsilon();
  double eps = numeric_limits<double>::epsilon()  ;
  arma::vec::fixed<13> p_ini = {eps, eps, eps, eps, eps, eps, eps, eps, eps, eps, eps, eps, eps};
  
  //P = eye(PAR.Robot_state_size,PAR.Robot_state_size)*p_ini;
  P = arma::diagmat(p_ini);
  
  //-------------------------------------------------------------------------------  
  cam_parameters.distortions = &PAR.Mono_cam_distortions[0];
  cam_parameters.cc[0] = PAR.Mono_cam_cc_u;
  cam_parameters.cc[1] = PAR.Mono_cam_cc_v;
  cam_parameters.fc[0] = PAR.Mono_cam_fc_u;
  cam_parameters.fc[1] = PAR.Mono_cam_fc_v;
  cam_parameters.alpha_c = PAR.Mono_cam_alpha_c;
         
  t_c2r(0) = PAR.Mono_cam_2_robot_pos_x;
  t_c2r(1) = PAR.Mono_cam_2_robot_pos_y;
  t_c2r(2) = PAR.Mono_cam_2_robot_pos_z;        

  double cam_axis_x = PAR.Mono_cam_2_robot_axis_x;
  double cam_axis_y = PAR.Mono_cam_2_robot_axis_y;
  double cam_axis_z = PAR.Mono_cam_2_robot_axis_z;
  double Ra2b_c[9];
  Euler_to_Ra2b_colum_major(cam_axis_x, cam_axis_y, cam_axis_z, Ra2b_c);
  arma::mat Rr2c_t(Ra2b_c,3,3); // camera to robot rotation matrix 
  Rr2c = Rr2c_t;


  //-------------------------------------------------------------------------------- 
  FeatsDATA.clear();
  AnchorsDATA.clear();
  NewKF_available = false;
  NewRobotState_available = false;
  New_KF_cl = false;

  cout << "lslam state initialized" << endl;
  cout << x << endl;

  

}
//----------------------------------------------
void EKF::ekf_step(DATA &dat)
{   
    
    static auto ct_i = std::chrono::high_resolution_clock::now(); // for computation time 
    
    static auto dt_i = std::chrono::high_resolution_clock::now(); // for compute delta_t and 
    static auto last_time = std::chrono::high_resolution_clock::now();
    static auto last_time_data = std::chrono::high_resolution_clock::now();
    
    static double last_timeDS;
    static bool init_t = false;
    
    static double total_comp_time = 0;
    static double total_exec_time = 0;

    static int n_o = 0;

        //----- ekf prediction --------------------------------------------------------------------
            
            dt_i = std::chrono::high_resolution_clock::now();            
            auto elapsed = std::chrono::duration_cast<std::chrono::nanoseconds>(dt_i - last_time);            
            double  delta_t = elapsed.count() * 1e-9;
            if(delta_t > PAR.Mono_cam_frame_period)delta_t = PAR.Mono_cam_frame_period;
            bool prediction_done = false;
            
            if(PAR.Data_origin == "dataset")                
                {
                 
                  delta_t = delta_t*PAR.x_vel_run_time;
                }
            
                
            if( delta_t > 1/(double)PAR.Max_freq_output || dat.data_type != "")
            {   
                
                

                bool receiving_data;
                if(dat.data_type != "")
                {
                  last_time_data = std::chrono::high_resolution_clock::now();  
                }
                
                auto ct_d = std::chrono::high_resolution_clock::now();
                auto elapsed_last_data = std::chrono::duration_cast<std::chrono::nanoseconds>(ct_d - last_time_data);
                double et_since_last_data = elapsed_last_data .count() * 1e-9; 
                if(et_since_last_data > .1)
                { 
                    // if no data has been received by a period o time, set flag
                    receiving_data = false;
                }
                else
                {
                    receiving_data = true;
                }
                
             // ct_i = std::chrono::high_resolution_clock::now();  
             // cout << delta_t << endl;
     
              if(receiving_data && PAR.Prediction_model == "noise_driven")
              {
                localslam::dynamic_diag::Row pred_b;
                if (localslam::dynamic_diag::enabled()) {
                  pred_b.mono_ms = localslam::dynamic_diag::mono_ms_now();
                  pred_b.stage = "PRED";
                  pred_b.phase = "before";
                  pred_b.pred_dt = delta_t;
                  pred_b.meas_time_ms = dat.time;
                  localslam::dynamic_diag::snapshot_state(pred_b, x, P);
                  localslam::dynamic_diag::push(pred_b);
                }
                prediction_euler_noise_driven(delta_t); // if data is been received, perform EKF prediction
                if (localslam::dynamic_diag::enabled()) {
                  localslam::dynamic_diag::Row pred_a = pred_b;
                  pred_a.mono_ms = localslam::dynamic_diag::mono_ms_now();
                  pred_a.phase = "after";
                  localslam::dynamic_diag::snapshot_state(pred_a, x, P);
                  localslam::dynamic_diag::fill_deltas(pred_a, pred_b);
                  localslam::dynamic_diag::push(pred_a);
                }
                total_exec_time =  total_exec_time + delta_t;
                //cout << "LocalSLAM -> " << total_exec_time << endl;
              }
              if(receiving_data == true && Compute_Pc_flag == true)
                {
                  prediction_compute_CL_cov(delta_t);
                } 
              
              prediction_done = true;
              last_time = dt_i;
            } 
            
            //-------------- end EKF prediction ------------------------------------------------------
           
            if (dat.data_type != "") // for any kind of measurement
            {                 
               // n_o++;            
                /*
                if(PAR.Data_origin == "dataset")
                {
                  // compute delta_t from dataset times
                  if (init_t == false)
                  {
                    delta_t = PAR.Mono_cam_frame_period;
                    last_timeDS = dat.time;
                    init_t = true;
                  }
                  else
                  {
                    delta_t = (dat.time - last_timeDS)/1000000;
                    last_timeDS = dat.time;                    
                  }                 

                }
                */                  
                /*
                ct_i = std::chrono::high_resolution_clock::now();    

                auto t_c = std::chrono::high_resolution_clock::now();
                auto elapsed = std::chrono::duration_cast<std::chrono::nanoseconds>(t_c - last_time);
                double  delta_t = elapsed.count() * 1e-9;
                if(delta_t > PAR.Mono_cam_frame_period)delta_t = PAR.Mono_cam_frame_period; // set the maximun delta_t as the camera frame rate period 
                last_time = t_c;
                // for data from dataset, compensate for the data frequency of the dataset component
                
                if(PAR.Data_origin == "dataset")                
                { 
                  delta_t = delta_t*PAR.x_vel_run_time;
                }  

                prediction(delta_t); // EKF prediction
                
                total_exec_time =  total_exec_time + delta_t;
                //cout << delta_t <<  endl;
                */
            } 

            if(dat.data_type == "odometry_vw" && PAR.Prediction_model == "odometry_vw_robot")
            {
                prediction_euler_odometry_vw_robot(dat.odov);
            }

            if(dat.data_type == "odometry_diff" && PAR.Prediction_model == "differential_robot")
            {
                prediction_euler_differential_robot(dat.odod);
            }


            if(dat.data_type == "frame")
            {
              
               ct_i = std::chrono::high_resolution_clock::now();  

               localslam::dynamic_diag::Row vis_b;
               if (localslam::dynamic_diag::enabled()) {
                 vis_b.mono_ms = localslam::dynamic_diag::mono_ms_now();
                 vis_b.stage = "VIS";
                 vis_b.phase = "before";
                 vis_b.meas_time_ms = dat.frame.time != 0 ? dat.frame.time : dat.time;
                 localslam::dynamic_diag::snapshot_state(vis_b, x, P);
                 localslam::dynamic_diag::push(vis_b);
               }
               if(PAR.Visual_update)Visual_update_e(&dat.frame);
               if (localslam::dynamic_diag::enabled()) {
                 localslam::dynamic_diag::Row vis_a = vis_b;
                 vis_a.mono_ms = localslam::dynamic_diag::mono_ms_now();
                 vis_a.phase = "after";
                 localslam::dynamic_diag::snapshot_state(vis_a, x, P);
                 localslam::dynamic_diag::fill_deltas(vis_a, vis_b);
                 localslam::dynamic_diag::push(vis_a);
               }

               auto ct_f = std::chrono::high_resolution_clock::now();
               auto elapsed = std::chrono::duration_cast<std::chrono::nanoseconds>(ct_f - ct_i);
               double comp_time_per_frame = elapsed.count() * 1e-9;
               
               total_comp_time = total_comp_time + comp_time_per_frame;
              // cout << "computation time per frame(s): " << comp_time_per_frame << endl;
              // cout << "total exec time(s): " << total_exec_time << " total comp time(s): " << total_comp_time << endl;
              // cout << n_o << endl;
              if(PAR.Stats){
                store.time_per_frame.first.push_back(total_exec_time);
                store.time_per_frame.second.push_back(comp_time_per_frame);                
                store.total_exec_time = total_exec_time;
                store.total_comp_time = total_comp_time;
                Store_info(total_exec_time);
              }

            }

            if(dat.data_type == "alt")
            {
              //cout << "Altitude:" << dat.alt.altitude << endl;
              localslam::dynamic_diag::Row alt_b;
              if (localslam::dynamic_diag::enabled() && PAR.Altitude_update) {
                alt_b.mono_ms = localslam::dynamic_diag::mono_ms_now();
                alt_b.stage = "ALT";
                alt_b.phase = "before";
                alt_b.meas_time_ms = dat.alt.time != 0 ? dat.alt.time : dat.time;
                localslam::dynamic_diag::snapshot_state(alt_b, x, P);
                localslam::dynamic_diag::push(alt_b);
              }
              if(PAR.Altitude_update)Altitude_update(dat.alt);
              if (localslam::dynamic_diag::enabled() && PAR.Altitude_update) {
                localslam::dynamic_diag::Row alt_a = alt_b;
                alt_a.mono_ms = localslam::dynamic_diag::mono_ms_now();
                alt_a.phase = "after";
                localslam::dynamic_diag::snapshot_state(alt_a, x, P);
                localslam::dynamic_diag::fill_deltas(alt_a, alt_b);
                localslam::dynamic_diag::push(alt_a);
              }

            }
            if(dat.data_type == "speed")
            {
              //cout << "Speed : " << dat.spd.speedX << " " << dat.spd.speedY << " " << dat.spd.speedX << endl;
              //cout << "position before update : " << x(7) << " " << x(8) << " " << x(9) << endl;
              if(PAR.Speed_update)Speed_Update_nav_euler(dat.spd);
              //cout << "position after update : " << x(7) << " " << x(8) << " " << x(9) << endl; 

            }
            if(dat.data_type == "attitude")
            {       
              
              //cout << "roll: " << dat.att.roll*(180/3.1416) <<  " pitch: " << dat.att.pitch*(180/3.1416)  << " yaw: " << dat.att.yaw*(180/3.1416) <<endl;
              //cout << "sum_dt " << sum_dt << endl; 
              //cout << "b: " << x(0) << " " << x(1)*(180/3.1416) << " " << x(2)*(180/3.1416) << " "<<  x(3)*(180/3.1416) << endl;
              localslam::dynamic_diag::Row att_b;
              if (localslam::dynamic_diag::enabled() && PAR.Attitude_update) {
                att_b.mono_ms = localslam::dynamic_diag::mono_ms_now();
                att_b.stage = "AHRS";
                att_b.phase = "before";
                att_b.meas_time_ms = dat.att.time != 0 ? dat.att.time : dat.time;
                localslam::dynamic_diag::snapshot_state(att_b, x, P);
                localslam::dynamic_diag::push(att_b);
              }
              if(PAR.Attitude_update)Attitude_euler_Update(dat.att);
              if (localslam::dynamic_diag::enabled() && PAR.Attitude_update) {
                localslam::dynamic_diag::Row att_a = att_b;
                att_a.mono_ms = localslam::dynamic_diag::mono_ms_now();
                att_a.phase = "after";
                localslam::dynamic_diag::snapshot_state(att_a, x, P);
                localslam::dynamic_diag::fill_deltas(att_a, att_b);
                localslam::dynamic_diag::push(att_a);
              }
              //cout << "a: " << x(0) << " " << x(1)*(180/3.1416) << " " << x(2)*(180/3.1416) << " "<<  x(3)*(180/3.1416) << endl;              

            }
            
            if(dat.data_type == "rangetobase")
            {
              localslam::dynamic_diag::Row rng_b;
              if (localslam::dynamic_diag::enabled() && PAR.Rangetobase_update) {
                rng_b.mono_ms = localslam::dynamic_diag::mono_ms_now();
                rng_b.stage = "RANGE";
                rng_b.phase = "before";
                rng_b.meas_time_ms = dat.rangetobase.time != 0 ? dat.rangetobase.time : dat.time;
                localslam::dynamic_diag::snapshot_state(rng_b, x, P);
                localslam::dynamic_diag::push(rng_b);
              }
              if(PAR.Rangetobase_update)Rangetobase_update(dat.rangetobase);
              if (localslam::dynamic_diag::enabled() && PAR.Rangetobase_update) {
                localslam::dynamic_diag::Row rng_a = rng_b;
                rng_a.mono_ms = localslam::dynamic_diag::mono_ms_now();
                rng_a.phase = "after";
                localslam::dynamic_diag::snapshot_state(rng_a, x, P);
                localslam::dynamic_diag::fill_deltas(rng_a, rng_b);
                localslam::dynamic_diag::push(rng_a);
              }
               //cout << "Range to base: " << dat.rangetobase.range << endl;

            }

            if(dat.data_type == "range")
            {
            
            //cout << "Range: " << dat.range.range << endl;

            }
            if(dat.data_type == "gps")
            {
            
            //cout << "Gps sats: " << dat.gps.sat << endl;

            }
            
            if(dat.data_type == "odometry_diff")
            {

              //cout << "Odometry Diff: " << dat.odod.TicksRight << " " << dat.odod.TicksLeft << endl;

            }

            if(dat.data_type == "odometry_vw")
            {

              //cout << "Odometry VW: " << dat.odov.linear_vel << " " << dat.odov.angular_vel << endl;
            }

            if (dat.data_type != "" || prediction_done == true ) // for any kind of measurement
            {   
               auto ct_t = std::chrono::high_resolution_clock::now();
               auto elapsed = std::chrono::duration_cast<std::chrono::nanoseconds>(ct_t - ct_i);
               double comp_time_per_step = elapsed.count() * 1e-9;            
               //total_comp_time = total_comp_time + comp_time_per_step;
               n_o++;
              
               NewRobotState_available = true;
            }


}

//----------------------------------------------------
EKF::PosDeltaApplyResult EKF::Update_pos_with_delta(
  arma::vec::fixed<3> delta_pos, string type)
{
  using localslam::PosUpdateGateParams;
  using localslam::plan_pos_update_apply;

  PosUpdateGateParams gates;
  gates.gs_xy = PAR.GS_xy_update;
  gates.gs_z = PAR.GS_z_update;
  gates.gs_xy_max = PAR.GS_xy_update_max_delta;
  gates.gs_z_max = PAR.GS_z_update_max_delta;
  gates.cl_xy = PAR.CL_xy_update;
  gates.cl_z = PAR.CL_z_update;
  gates.cl_xy_max = PAR.CL_xy_update_max_delta;
  gates.cl_z_max = PAR.CL_z_update_max_delta;

  const auto plan = plan_pos_update_apply(
    delta_pos[0], delta_pos[1], delta_pos[2], type, gates);

  PosDeltaApplyResult out;
  out.applied = plan.any_applied;
  out.applied_dx = plan.applied_dx;
  out.applied_dy = plan.applied_dy;
  out.applied_dz = plan.applied_dz;
  out.reset_pc = plan.reset_pc;

  if (!plan.any_applied) {
    return out;
  }

  if (plan.apply_xy) {
    x[7] = x[7] + plan.applied_dx;
    x[8] = x[8] + plan.applied_dy;
  }
  if (plan.apply_z) {
    x[9] = x[9] + plan.applied_dz;
  }

  for (size_t i = 0; i < FeatsDATA.size(); i++) {
    const int idx_i = FeatsDATA[i].idx_i_state;
    if (plan.apply_xy) {
      x[idx_i] = x[idx_i] + plan.applied_dx;
      x[idx_i + 1] = x[idx_i + 1] + plan.applied_dy;
    }
    if (plan.apply_z) {
      x[idx_i + 2] = x[idx_i + 2] + plan.applied_dz;
    }
  }
  for (size_t i = 0; i < AnchorsDATA.size(); i++) {
    if (plan.apply_xy) {
      AnchorsDATA[i].AnchorState[0] += plan.applied_dx;
      AnchorsDATA[i].AnchorState[1] += plan.applied_dy;
    }
    if (plan.apply_z) {
      AnchorsDATA[i].AnchorState[2] += plan.applied_dz;
    }
  }

  // close_loop: reset CL horizontal uncertainty only when a correction applied.
  if (plan.reset_pc) {
    Pc.zeros();
  }
  // Covariance P is intentionally unchanged (existing behavior).
  // Expose corrected pose on the next snapshot publish (same EKF cycle).
  NewRobotState_available = true;
  return out;
}
//--------------------------------------
void EKF::SetCompute_Pc_flag(bool state)
{
  Compute_Pc_flag = state;
}

//--------------------------------------
bool EKF::get_ekf_initialized_state()
{
    return initialized;
}
void EKF::set_ekf_initialized_statet(bool state)
{
    initialized = state;
    // A5/A4: on re-init request, drop any pending pre-reset snapshot so the
    // worker cannot publish stale robot/odom/path after clearing outputs.
    if (state == false) {
      NewRobotState_available = false;
      NewKF_available = false;
      New_KF_cl = false;
      // Clear altitude home so a post-reset first sample re-latches (instance
      // state; replaces the former process-static latch that survived reset).
      reset_altitude_home();
    }
}
void EKF::set_ekf_visual_update(bool state)
{
    PAR.Visual_update = state;
}

void EKF::set_camera_intrinsics(
  int img_cols,
  int img_rows,
  double fc_u,
  double fc_v,
  double cc_u,
  double cc_v,
  double alpha_c,
  const std::vector<double> & distortions)
{
  PAR.Mono_cam_img_cols = img_cols;
  PAR.Mono_cam_img_rows = img_rows;
  PAR.Mono_cam_fc_u = fc_u;
  PAR.Mono_cam_fc_v = fc_v;
  PAR.Mono_cam_cc_u = cc_u;
  PAR.Mono_cam_cc_v = cc_v;
  PAR.Mono_cam_alpha_c = alpha_c;
  PAR.Mono_cam_distortions = distortions;
  if (PAR.Mono_cam_distortions.size() < 5) {
    PAR.Mono_cam_distortions.resize(5, 0.0);
  }
  cam_parameters.distortions = &PAR.Mono_cam_distortions[0];
  cam_parameters.fc[0] = PAR.Mono_cam_fc_u;
  cam_parameters.fc[1] = PAR.Mono_cam_fc_v;
  cam_parameters.cc[0] = PAR.Mono_cam_cc_u;
  cam_parameters.cc[1] = PAR.Mono_cam_cc_v;
  cam_parameters.alpha_c = PAR.Mono_cam_alpha_c;
} 
//-----------------------------------------
// Rodrigo M. 2022
bool EKF::get_RobotState(arma::vec::fixed<13> &x_r,double &radius_uncertainty)
{
  arma::mat::fixed<13, 13> P_unused;
  return get_RobotStateSnapshot(x_r, P_unused, radius_uncertainty);
}

bool EKF::get_RobotStateSnapshot(
  arma::vec::fixed<13> & x_r,
  arma::mat::fixed<13, 13> & P_r,
  double & radius_uncertainty)
{
  if (NewRobotState_available == true) {
    x_r = x.subvec(0, 12);
    P_r = P(arma::span(0, 12), arma::span(0, 12));
    radius_uncertainty = Cl_radius_uncertainty;
    NewRobotState_available = false;
    return true;
  }
  return false;
}

void EKF::copy_robot_state_and_cov(
  arma::vec::fixed<13> & x_r,
  arma::mat::fixed<13, 13> & P_r) const
{
  x_r = x.subvec(0, 12);
  P_r = P(arma::span(0, 12), arma::span(0, 12));
}
//-----------------------------------------
// Rodrigo M. 2022
LOCALSLAM_DATA EKF::get_lslam_data()
{
  LOCALSLAM_DATA slam_state;
  
  // get robot state
  
  slam_state.robot_state = arma::conv_to<vec_t>::from(x.subvec(0,PAR.Robot_state_size-1)); // convert from armadillo vector to c++ vector 
  
  slam_state.Rr2c = Rr2c;
  slam_state.t_c2r = t_c2r;

  double phi = x(1);
  double theta = x(2);
  double psi = x(3);
  double Ra2b[9];
  Euler_to_Ra2b_colum_major(phi, theta, psi, Ra2b);
  arma::mat Rn2r(Ra2b,3,3);
  
  slam_state.Rn2r = Rn2r;
  slam_state.r_N = x.subvec(7,9); 

  // Clear output accumulators only (LOCALSLAM_DATA). Does not modify live
  // FeatsDATA / AnchorsDATA in the estimator.
  slam_state.map_state.clear();
  slam_state.map_anchors.clear();
  slam_state.matched_img_feats.clear();
  slam_state.unmatched_img_feats.clear();
  slam_state.unmatched_img_anchors.clear();
  slam_state.matched_img_anchors.clear();
  
  for(int i = 0; i < FeatsDATA.size(); i++)
  {
    POINT3d p;
    p.x =  x(FeatsDATA[i].idx_i_state);
    p.y =  x(FeatsDATA[i].idx_i_state + 1);
    p.z =  x(FeatsDATA[i].idx_i_state + 2);
    slam_state.map_state.push_back(p);
    
    if (FeatsDATA[i].predicted == true)
    {
      if(FeatsDATA[i].matched == true )
      {
        POINT3d pm;
        pm.x = FeatsDATA[i].Keypoint.pt.x;
        pm.y = FeatsDATA[i].Keypoint.pt.y;
        slam_state.matched_img_feats.push_back(pm);
      }
      else
      { 
        POINT3d pu;
        pu.x = FeatsDATA[i].PredictedPoint.x;
        pu.y = FeatsDATA[i].PredictedPoint.y;
        slam_state.unmatched_img_feats.push_back(pu);
      }
    }
  }
//-----------------------------------------------
 for(int i = 0; i < AnchorsDATA.size(); i++)
  {   
      POINT3d p;
      p.x =  AnchorsDATA[i].AnchorState[0];
      p.y =  AnchorsDATA[i].AnchorState[1];
      p.z =  AnchorsDATA[i].AnchorState[2];      
      slam_state.map_anchors.push_back(p);

      if (AnchorsDATA[i].predicted == true)
      {
        if(AnchorsDATA[i].matched == true )
        {
          POINT3d pm;
          pm.x = AnchorsDATA[i].Keypoint.pt.x;
          pm.y = AnchorsDATA[i].Keypoint.pt.y;
          slam_state.matched_img_anchors.push_back(pm);
        }
        else
        { 
          POINT3d pu;
          pu.x = AnchorsDATA[i].PredictedPoint.x;
          pu.y = AnchorsDATA[i].PredictedPoint.y;
          slam_state.unmatched_img_anchors.push_back(pu);
        }
      }
  }       

  slam_state.radius_uncertainty =  Cl_radius_uncertainty;
  
  //cout << slam_state.radius_uncertainty << endl;

  return slam_state;
}

//----------------------------------------------------------------
bool EKF::get_KeyFrame(KEYFRAME &KF)
{
  if(NewKF_available == true)
  {
    KF = KF_selected;
    NewKF_available = false;
    return true;
  }
  else
  {
    return false;
  }

}
//--------------------------------------------------------------
bool EKF::get_KeyFrameCL(KEYFRAME &KF)
{
  if(New_KF_cl == true)
  {
    KF = KF_cl;
    New_KF_cl = false;
    return true;
  }
  else
  {
    return false;
  }

}

void EKF::set_reference_identity(
  const slam_identity::UuidBytes & session_id, uint64_t epoch_id)
{
  ref_session_id_ = session_id;
  ref_epoch_id_ = epoch_id;
  next_keyframe_id_ = 1;
  // Drop any buffered KF selected under a previous identity.
  NewKF_available = false;
  New_KF_cl = false;
}

void EKF::stamp_keyframe_identity(KEYFRAME & kf, FRAME * frame, uint8_t stream)
{
  kf.session_id = ref_session_id_;
  kf.epoch_id = ref_epoch_id_;
  kf.keyframe_id = next_keyframe_id_++;
  kf.stream = stream;
  if (frame != nullptr && frame->time > 0) {
    kf.source_time_ms = frame->time;
    kf.source_stamp_valid = true;
  } else {
    kf.source_time_ms = 0;
    kf.source_stamp_valid = false;
  }
}

//-------------------------------------------------------------------
void EKF::Store_info(double time)
{
  int n_feats = FeatsDATA.size();
  int n_anchors = AnchorsDATA.size();
  store.n_feats_per_frame.first.push_back(time);
  store.n_feats_per_frame.second.push_back(n_feats);
  store.n_anchors_per_frame.first.push_back(time);
  store.n_anchors_per_frame.second.push_back(n_anchors);
}
