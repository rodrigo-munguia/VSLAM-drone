#ifndef LOCALSLAM_SENSOR_INPUT_PLAN_HPP_
#define LOCALSLAM_SENSOR_INPUT_PLAN_HPP_

#include <stdexcept>
#include <string>

namespace localslam
{

/** Which ROS subscriptions to open for a given Sensor_input_mode (A1). */
struct SensorInputPlan
{
  bool use_standard_migrated{false};

  // Migrated modalities (exactly one of standard/legacy active)
  bool sub_image_std{false};
  bool sub_imu_std{false};
  bool sub_body_velocity_std{false};
  bool sub_downward_range_std{false};
  /** Standard altitude modality slot (mavros OR FluidPressure — exclusive). */
  bool sub_altitude_std{false};
  bool sub_pressure_std{false};

  bool sub_frame_legacy{false};
  bool sub_att_legacy{false};
  bool sub_spd_legacy{false};
  bool sub_range_legacy{false};
  bool sub_alt_legacy{false};

  // Always retained
  bool sub_rangetobase{true};
  bool sub_gps_legacy{true};
  bool sub_odo_d_legacy{true};
  bool sub_odo_v_legacy{true};
};

inline SensorInputPlan make_sensor_input_plan(const std::string & mode)
{
  SensorInputPlan p;
  if (mode == "standard") {
    p.use_standard_migrated = true;
    p.sub_image_std = true;
    p.sub_imu_std = true;
    p.sub_body_velocity_std = true;
    p.sub_downward_range_std = true;
    p.sub_altitude_std = true;
    // Legacy copies of migrated modalities must stay off (no dual fusion).
    p.sub_frame_legacy = false;
    p.sub_att_legacy = false;
    p.sub_spd_legacy = false;
    p.sub_range_legacy = false;
    p.sub_alt_legacy = false;
    p.sub_rangetobase = true;
    return p;
  }
  if (mode == "legacy") {
    p.use_standard_migrated = false;
    p.sub_frame_legacy = true;
    p.sub_att_legacy = true;
    p.sub_spd_legacy = true;
    p.sub_range_legacy = true;
    p.sub_alt_legacy = true;
    p.sub_rangetobase = true;
    return p;
  }
  throw std::runtime_error(
    "Unsupported Sensor_input_mode='" + mode + "'. Use 'legacy' or 'standard'.");
}

/**
 * Validates Altitude_input_mode (A9).
 * derived_altitude → mavros_msgs/Altitude; fluid_pressure → FluidPressure.
 * Exclusive: never both.
 */
inline void validate_altitude_input_mode(const std::string & mode)
{
  if (mode == "derived_altitude" || mode == "fluid_pressure") {
    return;
  }
  throw std::runtime_error(
    "Unsupported Altitude_input_mode='" + mode +
    "'. Use 'derived_altitude' or 'fluid_pressure'.");
}

/**
 * Apply exclusive altitude-source selection onto a plan that already has
 * standard altitude modality enabled (sub_altitude_std from Sensor_input_mode).
 */
inline void apply_altitude_input_mode(
  SensorInputPlan & plan, const std::string & altitude_mode)
{
  validate_altitude_input_mode(altitude_mode);
  if (!plan.use_standard_migrated) {
    // Legacy Alt_topic remains the only altitude source in legacy mode.
    plan.sub_pressure_std = false;
    if (altitude_mode == "fluid_pressure") {
      throw std::runtime_error(
        "Altitude_input_mode=fluid_pressure requires Sensor_input_mode=standard");
    }
    return;
  }
  if (altitude_mode == "derived_altitude") {
    plan.sub_altitude_std = true;
    plan.sub_pressure_std = false;
  } else {
    plan.sub_altitude_std = false;
    plan.sub_pressure_std = true;
  }
}

}  // namespace localslam

#endif  // LOCALSLAM_SENSOR_INPUT_PLAN_HPP_
