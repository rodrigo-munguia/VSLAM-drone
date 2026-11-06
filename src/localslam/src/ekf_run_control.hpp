#ifndef LOCALSLAM_EKF_RUN_CONTROL_HPP_
#define LOCALSLAM_EKF_RUN_CONTROL_HPP_

namespace localslam
{

/**
 * Enable / disable / reset semantics shared by SimpleServ and std_srvs.
 * Does not touch the observation buffer (matches legacy 'i' / 'r' / 's').
 */
struct EKFRunControl
{
  bool steps_running{false};
  bool reinit_requested{false};

  void set_running(bool on) { steps_running = on; }

  void request_reinit() { reinit_requested = true; }

  /** Worker consumes reinit flag (equivalent to clearing re_init_sys). */
  bool consume_reinit()
  {
    if (!reinit_requested) {
      return false;
    }
    reinit_requested = false;
    return true;
  }
};

}  // namespace localslam

#endif  // LOCALSLAM_EKF_RUN_CONTROL_HPP_
