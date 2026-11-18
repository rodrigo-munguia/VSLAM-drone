#pragma once
/**
 * CTRL-R4d: convert ControlQuad mission waypoint args to absolute GPS ENU.
 *
 * Legacy Gps_callback stores relative pose with Y negated:
 *   r.x = gps_x - spawn_x
 *   r.y = -(gps_y - spawn_y)
 *   r.z = gps_z - spawn_z
 * so mission (x,y,z) matches that frame. Driver uses absolute GPS ENU:
 *   gps = spawn + (x, -y, z)
 * Do not apply an extra publish-time Y negation on the unified path.
 */

inline void mission_args_to_gps_enu(
  double spawn_x, double spawn_y, double spawn_z,
  double mission_x, double mission_y, double mission_z,
  double & gps_x, double & gps_y, double & gps_z)
{
  gps_x = spawn_x + mission_x;
  gps_y = spawn_y - mission_y;
  gps_z = spawn_z + mission_z;
}
