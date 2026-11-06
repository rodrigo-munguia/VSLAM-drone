#include "localslam_component.hpp"
#include "sensor_conversions.hpp"
#include "range_frame_association.hpp"
#include "camera_calibration.hpp"
#include "pressure_altitude.hpp"

#include "rclcpp/rclcpp.hpp"
#include "localslam_types.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include <cmath>
#include <mutex>
#include <algorithm>
#if __has_include(<cv_bridge/cv_bridge.hpp>)
#include <cv_bridge/cv_bridge.hpp>
#else
#include <cv_bridge/cv_bridge.h>
#endif

using namespace std;

namespace localslam
{

void EKFslam::enqueue_observation(const DATA & dat)
{
  DATA local = dat;
  mutex_dat.lock();
  ManageBuffer(local);
  mutex_dat.unlock();
}

void EKFslam::Alt_callback(const interfaces::msg::Alt & msg)
{
  DATA dat;
  dat.data_type = "alt";
  dat.alt.altitude = msg.altitude;
  dat.alt.time = msg.time;
  dat.time = msg.time;
  enqueue_observation(dat);
}

void EKFslam::Att_callback(const interfaces::msg::Att & msg)
{
  DATA dat;
  dat.data_type = "attitude";
  dat.att.roll = msg.roll;
  dat.att.pitch = msg.pitch;
  dat.att.yaw = msg.yaw;
  dat.att.time = msg.time;
  dat.time = msg.time;
  enqueue_observation(dat);
}

void EKFslam::Gps_callback(const interfaces::msg::Gps & msg)
{
  DATA dat;
  dat.data_type = "gps";
  dat.gps.lat = msg.lat;
  dat.gps.lon = msg.lon;
  dat.gps.alt = msg.alt;
  dat.gps.sat = msg.sat;
  dat.gps.time = msg.time;
  dat.time = msg.time;
  enqueue_observation(dat);
}

void EKFslam::Range_callback(const interfaces::msg::Range & msg)
{
  DATA dat;
  dat.range.time = msg.time;
  dat.data_type = "range";
  dat.range.range = msg.range;
  dat.range.volt = msg.volt;
  dat.time = msg.time;
  enqueue_observation(dat);
}

void EKFslam::Frame_callback(const interfaces::msg::Frame & msg)
{
  // A8: legacy Frame path still respects camera_info gating (no YAML merge).
  if (PAR.Calibration_source == "camera_info" && !visual_calibration_ready()) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 2000,
      "Ignoring legacy Frame_topic until CameraInfo calibration is ready");
    return;
  }

  DATA dat;
  dat.frame.time = msg.time;
  dat.data_type = "frame";

  cv_bridge::CvImagePtr cv_ptr;
  cv_ptr = cv_bridge::toCvCopy(msg.img, sensor_msgs::image_encodings::MONO8);
  dat.frame.image = cv_ptr->image;
  dat.frame.range = -1;
  dat.time = msg.time;
  enqueue_observation(dat);
}

void EKFslam::Speed_callback(const interfaces::msg::Spd & msg)
{
  DATA dat;
  dat.spd.time = msg.time;
  dat.data_type = "speed";
  dat.spd.speedX = msg.speed_x;
  dat.spd.speedY = msg.speed_y;
  dat.spd.speedZ = msg.speed_z;
  dat.time = msg.time;
  enqueue_observation(dat);
}

void EKFslam::OdoDiff_callback(const interfaces::msg::Ododiff &msg)
{
  DATA dat;
  dat.odod.time = msg.time;
  dat.data_type = "odometry_diff";
  dat.odod.TicksLeft = msg.ticks_left;
  dat.odod.TicksRight = msg.ticks_right;
  dat.time = msg.time;
  enqueue_observation(dat);
}

void EKFslam::OdoVW_callback(const interfaces::msg::Odovw &msg)
{
  DATA dat;
  dat.odov.time = msg.time;
  dat.data_type = "odometry_vw";
  dat.odov.angular_vel = msg.angular_vel;
  dat.odov.linear_vel = msg.linear_vel;
  dat.time = msg.time;
  enqueue_observation(dat);
}

void EKFslam::Rangetobase_callback(const interfaces::msg::Rangetobase &msg)
{
  DATA dat;
  dat.rangetobase.time = msg.time;
  dat.data_type = "rangetobase";
  dat.rangetobase.range = msg.range;
  dat.time = msg.time;
  enqueue_observation(dat);
}

//------------------------------------------------------------------
// Standard ROS inputs (A1) — convert then enqueue same DATA types
//------------------------------------------------------------------

void EKFslam::Image_std_callback(const sensor_msgs::msg::Image & msg)
{
  handle_visual_image(msg);
}

bool EKFslam::visual_calibration_ready() const
{
  if (PAR.Calibration_source == "yaml") {
    return true;
  }
  return calibration_ready_;
}

void EKFslam::apply_intrinsics_to_parameters(const camera_cal::Intrinsics & intr)
{
  PAR.Mono_cam_img_cols = intr.width;
  PAR.Mono_cam_img_rows = intr.height;
  PAR.Mono_cam_fc_u = intr.fc_u;
  PAR.Mono_cam_fc_v = intr.fc_v;
  PAR.Mono_cam_cc_u = intr.cc_u;
  PAR.Mono_cam_cc_v = intr.cc_v;
  PAR.Mono_cam_alpha_c = intr.alpha_c;
  PAR.Mono_cam_distortions = intr.distortions;
  if (PAR.Mono_cam_distortions.size() < 5) {
    PAR.Mono_cam_distortions.resize(5, 0.0);
  }
}

void EKFslam::handle_visual_image(const sensor_msgs::msg::Image & msg)
{
  if (PAR.Calibration_source == "camera_info" && !visual_calibration_ready()) {
    std::lock_guard<std::mutex> lock(mutex_cal_);
    const int bound = std::max(1, PAR.Camera_info_waiting_image_bound);
    waiting_images_.push_back(msg);
    while (static_cast<int>(waiting_images_.size()) > bound) {
      waiting_images_.pop_front();
      ++waiting_images_dropped_;
    }
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 2000,
      "Waiting for CameraInfo; holding %zu image(s) (dropped_total=%zu)",
      waiting_images_.size(), waiting_images_dropped_);
    return;
  }

  if (PAR.Calibration_source == "camera_info") {
    std::lock_guard<std::mutex> lock(mutex_cal_);
    if (!msg.header.frame_id.empty() && !applied_intrinsics_.optical_frame.empty() &&
      msg.header.frame_id != applied_intrinsics_.optical_frame)
    {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "Rejecting image frame_id='%s' (CameraInfo optical frame='%s')",
        msg.header.frame_id.c_str(), applied_intrinsics_.optical_frame.c_str());
      return;
    }
    if (!PAR.Camera_info_expected_frame_id.empty() &&
      msg.header.frame_id != PAR.Camera_info_expected_frame_id)
    {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "Rejecting image frame_id='%s' (expected '%s')",
        msg.header.frame_id.c_str(), PAR.Camera_info_expected_frame_id.c_str());
      return;
    }
    if (static_cast<int>(msg.width) != applied_intrinsics_.width ||
      static_cast<int>(msg.height) != applied_intrinsics_.height)
    {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "Rejecting image %ux%u (calibration %dx%d)",
        msg.width, msg.height,
        applied_intrinsics_.width, applied_intrinsics_.height);
      return;
    }
  } else if (msg.width > 0 && msg.height > 0 &&
    (static_cast<int>(msg.width) != PAR.Mono_cam_img_cols ||
     static_cast<int>(msg.height) != PAR.Mono_cam_img_rows))
  {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000,
      "Image size %ux%u differs from YAML Mono_cam_img %dx%d",
      msg.width, msg.height, PAR.Mono_cam_img_cols, PAR.Mono_cam_img_rows);
  }

  DATA dat;
  dat.data_type = "frame";
  dat.frame.time = conversions::stamp_to_ms(msg.header.stamp);
  dat.time = dat.frame.time;
  dat.frame.range = -1;
  dat.frame.range_type = "";

  try {
    cv_bridge::CvImageConstPtr cv_ptr;
    if (msg.encoding == sensor_msgs::image_encodings::MONO8 ||
        msg.encoding == "8UC1")
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
    } else {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
    }
    dat.frame.image = cv_ptr->image;
  } catch (const cv_bridge::Exception & e) {
    RCLCPP_WARN(this->get_logger(), "Image conversion failed: %s", e.what());
    return;
  }
  enqueue_observation(dat);
}

void EKFslam::flush_waiting_images()
{
  std::deque<sensor_msgs::msg::Image> pending;
  {
    std::lock_guard<std::mutex> lock(mutex_cal_);
    pending.swap(waiting_images_);
  }
  for (const auto & img : pending) {
    handle_visual_image(img);
  }
}

void EKFslam::CameraInfo_callback(const sensor_msgs::msg::CameraInfo & msg)
{
  if (PAR.Calibration_source != "camera_info") {
    return;
  }

  const auto mapped = camera_cal::map_camera_info(msg, PAR.Camera_info_image_path);
  if (!mapped.ok) {
    RCLCPP_ERROR_THROTTLE(
      this->get_logger(), *this->get_clock(), 2000,
      "Rejecting CameraInfo: %s", mapped.error.c_str());
    return;
  }
  if (!PAR.Camera_info_expected_frame_id.empty() &&
    msg.header.frame_id != PAR.Camera_info_expected_frame_id)
  {
    RCLCPP_ERROR_THROTTLE(
      this->get_logger(), *this->get_clock(), 2000,
      "Rejecting CameraInfo frame_id='%s' (expected '%s')",
      msg.header.frame_id.c_str(), PAR.Camera_info_expected_frame_id.c_str());
    return;
  }

  bool should_flush = false;
  {
    std::lock_guard<std::mutex> lock(mutex_cal_);
    if (calibration_ready_ &&
      camera_cal::intrinsics_equal(applied_intrinsics_, mapped.intrinsics))
    {
      return;
    }

    if (calibration_ready_ && !allow_new_geometry_after_reset_) {
      if (!camera_cal::intrinsics_equal(applied_intrinsics_, mapped.intrinsics)) {
        RCLCPP_ERROR_THROTTLE(
          this->get_logger(), *this->get_clock(), 2000,
          "Ignoring changed CameraInfo while estimator is active; "
          "call ~/reset before applying new geometry");
      }
      return;
    }

    applied_intrinsics_ = mapped.intrinsics;
    apply_intrinsics_to_parameters(applied_intrinsics_);
    calibration_ready_ = true;
    camera_cal_pending_apply_ = true;
    allow_new_geometry_after_reset_ = false;
    should_flush = true;
    RCLCPP_INFO(
      this->get_logger(),
      "CameraInfo applied (%s): %dx%d fc=(%.3f,%.3f) cc=(%.3f,%.3f) alpha=%.6f "
      "D=[%.6f,%.6f,0,0,0] frame='%s'",
      PAR.Camera_info_image_path.c_str(),
      applied_intrinsics_.width, applied_intrinsics_.height,
      applied_intrinsics_.fc_u, applied_intrinsics_.fc_v,
      applied_intrinsics_.cc_u, applied_intrinsics_.cc_v,
      applied_intrinsics_.alpha_c,
      applied_intrinsics_.distortions[0], applied_intrinsics_.distortions[1],
      applied_intrinsics_.optical_frame.c_str());
  }
  if (should_flush) {
    flush_waiting_images();
  }
}

void EKFslam::Imu_std_callback(const sensor_msgs::msg::Imu & msg)
{
  conversions::AttObs att;
  if (!conversions::imu_to_internal_att(msg, att)) {
    return;
  }
  DATA dat;
  dat.data_type = "attitude";
  dat.att.roll = att.roll;
  dat.att.pitch = att.pitch;
  dat.att.yaw = att.yaw;
  dat.att.time = att.time;
  dat.time = att.time;
  enqueue_observation(dat);
}

void EKFslam::BodyVelocity_std_callback(
  const geometry_msgs::msg::TwistWithCovarianceStamped & msg)
{
  conversions::SpdObs spd;
  if (!conversions::body_twist_to_internal_spd(msg, spd)) {
    return;
  }
  DATA dat;
  dat.data_type = "speed";
  dat.spd.speedX = spd.speedX;
  dat.spd.speedY = spd.speedY;
  dat.spd.speedZ = spd.speedZ;
  dat.spd.time = spd.time;
  dat.time = spd.time;
  enqueue_observation(dat);
}

void EKFslam::DownwardRange_std_callback(const sensor_msgs::msg::Range & msg)
{
  // Match legacy Range_callback: enqueue finite samples; getData applies the
  // (.4, 6) m gate used for landmark depth association.
  if (!std::isfinite(msg.range)) {
    return;
  }
  if (msg.min_range < msg.max_range) {
    if (msg.range < msg.min_range || msg.range > msg.max_range) {
      // Still enqueue so getData can mark null, matching out-of-gate handling.
    }
  }
  DATA dat;
  dat.data_type = "range";
  dat.range.range = msg.range;
  dat.range.volt = 0.0;
  dat.range.time = conversions::stamp_to_ms(msg.header.stamp);
  dat.time = dat.range.time;
  enqueue_observation(dat);
}

void EKFslam::Altitude_std_callback(const mavros_msgs::msg::Altitude & msg)
{
  // Exclusive with FluidPressure (subscription plan). Same enqueue path.
  conversions::AltObs alt;
  if (!conversions::mavros_relative_to_internal_alt(msg, alt)) {
    RCLCPP_DEBUG_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000,
      "Rejecting non-finite mavros Altitude.relative");
    return;
  }
  DATA dat;
  dat.data_type = "alt";
  dat.alt.altitude = alt.altitude;
  dat.alt.time = alt.time;
  dat.time = alt.time;
  enqueue_observation(dat);
}

void EKFslam::FluidPressure_std_callback(
  const sensor_msgs::msg::FluidPressure & msg)
{
  // Convert outside the EKF update. variance (Pa²) is ignored by design.
  const auto converted = pressure_converter_.convert(msg);
  if (!converted.ok) {
    // Bound/discard rejected samples (do not accumulate indefinitely).
    std::lock_guard<std::mutex> lock(mutex_pressure_wait_);
    const int bound = std::max(1, PAR.Pressure_waiting_bound);
    waiting_pressures_.push_back(msg);
    while (static_cast<int>(waiting_pressures_.size()) > bound) {
      waiting_pressures_.pop_front();
      ++waiting_pressures_dropped_;
    }
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 2000,
      "Rejecting FluidPressure: %s (held_reject=%zu dropped_total=%zu)",
      converted.error.c_str(), waiting_pressures_.size(),
      waiting_pressures_dropped_);
    return;
  }
  {
    // Valid conversion: drop any previously held rejects.
    std::lock_guard<std::mutex> lock(mutex_pressure_wait_);
    waiting_pressures_.clear();
  }
  if (converted.latched_new_reference) {
    RCLCPP_INFO(
      this->get_logger(),
      "Pressure reference latched at P_ref=%.3f Pa (ISA h=%.3f m); "
      "relative altitude starts at 0 (EKF alt latch remains separate; "
      "first relative≈0 avoids a second offset). "
      "FluidPressure.variance ignored.",
      converted.pressure_pa, converted.h_isa_m);
  }

  DATA dat;
  dat.data_type = "alt";
  dat.alt.altitude = converted.alt.altitude;
  dat.alt.time = converted.alt.time;
  dat.time = converted.alt.time;
  enqueue_observation(dat);
}

//--------------------------------------------------

void EKFslam::ManageBuffer(DATA &dat)
{
  string data_type = dat.data_type;

  for (int i = 0; i < static_cast<int>(DataBuffer.size()); i++) {
    if (DataBuffer[i].data_type == data_type) {
      DataBuffer.erase(DataBuffer.begin() + i);
      i--;
    }
  }
  DataBuffer.push_back(dat);
}

//----------------------------------------------------

DATA EKFslam::getData()
{
  // Association state remains process-static (legacy getData behavior).
  static RangeFrameAssociationState assoc;

  mutex_dat.lock();

  DATA dat;

  for (int i = 0; i < static_cast<int>(DataBuffer.size()); i++) {
    if (DataBuffer[i].data_type == "speed") {
      dat = DataBuffer[i];
      DataBuffer.erase(DataBuffer.begin() + i);
      break;
    }
    if (DataBuffer[i].data_type == "odometry_diff") {
      dat = DataBuffer[i];
      DataBuffer.erase(DataBuffer.begin() + i);
      break;
    }
    if (DataBuffer[i].data_type == "odometry_vw") {
      dat = DataBuffer[i];
      DataBuffer.erase(DataBuffer.begin() + i);
      break;
    }
    if (DataBuffer[i].data_type == "attitude") {
      dat = DataBuffer[i];
      DataBuffer.erase(DataBuffer.begin() + i);
      break;
    }
    if (DataBuffer[i].data_type == "alt") {
      dat = DataBuffer[i];
      DataBuffer.erase(DataBuffer.begin() + i);
      note_altitude_for_fallback(assoc, dat.alt.altitude);
      break;
    }
    if (DataBuffer[i].data_type == "rangetobase") {
      dat = DataBuffer[i];
      DataBuffer.erase(DataBuffer.begin() + i);
      break;
    }
    if (DataBuffer[i].data_type == "range") {
      dat = DataBuffer[i];
      DataBuffer.erase(DataBuffer.begin() + i);

      if (note_downward_range(assoc, dat.range.range, dat.range.time)) {
        break;
      } else {
        dat.data_type = "null";
        break;
      }
    }
    if (DataBuffer[i].data_type == "frame") {
      dat = DataBuffer[i];
      DataBuffer.erase(DataBuffer.begin() + i);

      dat.frame.range_type = associate_depth_to_frame(
        assoc, dat.frame.range, dat.frame.time);
      break;
    }
  }
  mutex_dat.unlock();

  return dat;
}

}  // namespace localslam
