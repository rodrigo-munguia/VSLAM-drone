#ifndef PLOT2_PLOT_FRAME_CONVERSIONS_HPP_
#define PLOT2_PLOT_FRAME_CONVERSIONS_HPP_

/**
 * Plot2 display-frame helpers (A9 / public odom parent).
 *
 * GT_topic (webotbridge) publishes Webots GPS already in the *public*
 * odom parent, epoch-aligned via EstimatorReference (same conversion as
 * /ground_truth/path; see webotbridge/gt_frame_conversions.hpp). Plot2 must
 * not apply D, attitude transforms, or a second origin subtraction.
 *
 * Estimated pose/path/landmarks on standard topics are already in the A2
 * *public* odom parent: p_pub = D * p_int.
 *
 * legacy_gt_position_to_public() remains for tests of the historical
 * internal-matched topic encoding (x, -y, -z) → public via D.
 *
 * Vertical view projection (public +up → ImGui pixels):
 *   Screen Y increases downward. Legacy Plot2 used screen_y += z when the
 *   stored Z was positive-down. Public streams are positive-up, so vertical
 *   views must use screen_y -= z_pub. This is a screen projection only —
 *   stored ROS coordinates are not modified.
 *
 * Top XY view (from above):
 *   screen_x = center_x + x_pub * s   →  +X right
 *   screen_y = center_y - y_pub * s   →  +Y up
 *   Viewing direction = -Z (look from +Z toward -Z). Screen basis
 *   right×up = +X×+Y = +Z, look = -(right×up) = -Z. Proper (det +1).
 *   An arbitrary Webots orbit can still map different world axes to its
 *   screen-right; that is camera choice, not a Plot2 data flip.
 */

namespace plot2
{
namespace frames
{

inline constexpr double D_axis[3] = {1.0, -1.0, -1.0};

/** Map legacy GT_topic position into the public odom parent (+up). */
inline void legacy_gt_position_to_public(
  double x_gt, double y_gt, double z_gt,
  double & x_pub, double & y_pub, double & z_pub)
{
  x_pub = D_axis[0] * x_gt;
  y_pub = D_axis[1] * y_gt;
  z_pub = D_axis[2] * z_gt;
}

/**
 * Independent physical check (no shared helper on the expected side):
 * Webots GPS z-up position (after optional yaw/home alignment encoded in
 * GT_topic x/y), with GT_topic = (x, -y_aligned, -z_up).
 * Public +up displacement must restore +y_aligned and +z_up.
 */
inline void expected_public_from_webots_z_up_via_legacy_gt(
  double x, double y_aligned, double z_up,
  double & x_pub, double & y_pub, double & z_pub)
{
  const double gt_x = x;
  const double gt_y = -y_aligned;
  const double gt_z = -z_up;
  legacy_gt_position_to_public(gt_x, gt_y, gt_z, x_pub, y_pub, z_pub);
}

/**
 * Public X–Z → screen. +X right, +Z up (screen_y decreases with +Z).
 * center_x/center_y are the pixel origin for world (0,*,0).
 */
inline void public_xz_to_screen(
  double x_pub, double z_pub,
  double center_x, double center_y, double scale,
  double & screen_x, double & screen_y)
{
  screen_x = center_x + x_pub * scale;
  screen_y = center_y - z_pub * scale;
}

/** Inverse of public_xz_to_screen (y world unused). */
inline void screen_to_public_xz(
  double screen_x, double screen_y,
  double center_x, double center_y, double scale,
  double & x_pub, double & z_pub)
{
  x_pub = (screen_x - center_x) / scale;
  z_pub = (center_y - screen_y) / scale;
}

/**
 * Public Z–Y → screen. +Y right, +Z up.
 * Horizontal axis is public Y (unchanged); vertical is public Z with +up.
 */
inline void public_zy_to_screen(
  double y_pub, double z_pub,
  double center_x, double center_y, double scale,
  double & screen_x, double & screen_y)
{
  screen_x = center_x + y_pub * scale;
  screen_y = center_y - z_pub * scale;
}

/** Inverse of public_zy_to_screen (x world unused). */
inline void screen_to_public_zy(
  double screen_x, double screen_y,
  double center_x, double center_y, double scale,
  double & y_pub, double & z_pub)
{
  y_pub = (screen_x - center_x) / scale;
  z_pub = (center_y - screen_y) / scale;
}

/**
 * Top XY from above (public odom parent):
 *   screen_x = center_x + x_pub * scale
 *   screen_y = center_y - y_pub * scale
 * Pixel Y increases downward. Positive scale keeps +X right, +Y up.
 */
inline void public_xy_to_screen(
  double x_pub, double y_pub,
  double center_x, double center_y, double scale,
  double & screen_x, double & screen_y)
{
  screen_x = center_x + x_pub * scale;
  screen_y = center_y - y_pub * scale;
}

/** Inverse of public_xy_to_screen (z world unused). */
inline void screen_to_public_xy(
  double screen_x, double screen_y,
  double center_x, double center_y, double scale,
  double & x_pub, double & y_pub)
{
  x_pub = (screen_x - center_x) / scale;
  y_pub = (center_y - screen_y) / scale;
}

/**
 * Screen-up basis for the top view in public XY (unit vectors).
 * right = (+1, 0), up = (0, +1) ⇒ right×up = +Z ⇒ look = -Z (from above).
 */
inline void top_view_screen_basis_public_xy(
  double right_xy[2], double up_xy[2])
{
  right_xy[0] = 1.0;
  right_xy[1] = 0.0;
  up_xy[0] = 0.0;
  up_xy[1] = 1.0;
}

}  // namespace frames
}  // namespace plot2

#endif  // PLOT2_PLOT_FRAME_CONVERSIONS_HPP_
