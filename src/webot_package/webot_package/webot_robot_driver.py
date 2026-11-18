# Copyright 1996-2023 Cyberbotics Ltd.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""ROS2 Mavic 2 Pro driver."""

import math
import rclpy
import os
from geometry_msgs.msg import Twist, Vector3
from interfaces.srv import SimpleServ
from interfaces.msg import Robotstate
import time

from webot_package.jerk_limited_trajectory import JerkLimitedTrajectory
from webot_package.traj_profile import (
    feasibility_xyz as traj_feasibility_xyz,
    resolve_effective_limits,
    segment_feasibility,
)
from webot_package.traj_request_manager import (
    STATUS_ACCEPTED,
    STATUS_CANCELLED,
    STATUS_REJECTED,
    STATUS_RUNNING,
    STATUS_SETTLED,
    StatusSnapshot,
    TrajRequestManager,
)
from webot_package.unified_position_control import (
    HOVER_OMEGA,
    TRAJ_A_MAX,
    TRAJ_J_MAX,
    TRAJ_STILL_SPEED,
    TRAJ_V_MAX,
    TRAJ_V_MAX_XY,
    TRAJ_V_MAX_Z,
    V_MAX_XY,
    V_MAX_Z,
    measured_speed,
    unified_step,
)
from interfaces.srv import UnifiedTrajectory
from interfaces.msg import UnifiedTrajectoryStatus


#K_VERTICAL_THRUST = 68.5    # with this thrust, the drone lifts.
K_VERTICAL_THRUST = 0.0
K_VERTICAL_P = 3.0          # Climb tracking P; hold uses a lower gain below.
K_VERTICAL_D = 1.5          # Vertical-rate damping [thrust/(m/s)]; stronger in hold.
K_VERTICAL_D_CLIMB = 0.45   # Lighter damping while climbing (preserve rate).
# CTRL-S1: true attitude setpoints (rad), not mixer disturbances.
# Signs match legacy leveler: u = Kp*(meas - des) + Kd*rate  (des=0 → old law).
# First ±5° step with Kp=50/Kd=1 overshot and flipped — retuned for damping.
K_ROLL_P = 12.0
K_PITCH_P = 10.0
K_ROLL_D = 8.0
K_PITCH_D = 6.0
K_YAW_P = 2.0
K_ATT_LIMIT = math.radians(15.0)   # |phi_d|, |theta_d| sat [rad]
K_ATT_SLEW = math.radians(25.0)    # max setpoint rate [rad/s]
K_ATT_MIXER_LIMIT = 25.0           # |roll_input|/|pitch_input| sat
K_ATT_SAFE = math.radians(50.0)    # if |meas| exceeds, force level setpoint
# CTRL-S2: horizontal accel → attitude (ENU m/s² → rad).
G_ACCEL = 9.81
K_ACCEL_XY_MAX = 2.5               # |a_xy| sat [m/s²] (~g tan 14°)
K_TILT_MAX = math.radians(12.0)    # combined tilt limit for accel path
# CTRL-S3: body-horizontal velocity P → accel (then S2/S1).
K_VEL_P = 2.5                      # Kv [1/s]; a_B = Kv (v_d - v); trims P shortfall
K_DIAG_TIMEOUT = 0.5               # attitude/accel diagnostic ownership [s]
LIFT_HEIGHT = 1


def attitude_pd(measured, desired, rate, kp, kd):
    """Mixer-compatible PD: u = kp*(meas - des) + kd*rate.

    With desired=0 this matches the pre-S1 Webots leveling term.
    Nonfinite desired → treat as 0 (safe level).
    """
    if not math.isfinite(desired):
        desired = 0.0
    if not math.isfinite(measured):
        measured = 0.0
    if not math.isfinite(rate):
        rate = 0.0
    return kp * (measured - desired) + kd * rate


def enu_accel_to_body(a_e, a_n, yaw):
    """ENU horizontal accel → body xy (same yaw map as GPS velocity)."""
    c = math.cos(yaw)
    s = math.sin(yaw)
    a_bx = c * a_e + s * a_n
    a_by = -s * a_e + c * a_n
    return a_bx, a_by


def body_accel_to_enu(a_bx, a_by, yaw):
    """Body horizontal accel → ENU (inverse of enu_accel_to_body)."""
    c = math.cos(yaw)
    s = math.sin(yaw)
    a_e = c * a_bx - s * a_by
    a_n = s * a_bx + c * a_by
    return a_e, a_n


def velocity_to_accel(v_dx, v_dy, v_x, v_y, kv=None, accel_max=None):
    """Body-horizontal P velocity loop: a_B = Kv (v_d - v), |a| sat.

    Nonfinite inputs → 0. Zero command with nonzero v → braking accel.
    """
    if kv is None:
        kv = K_VEL_P
    if accel_max is None:
        accel_max = K_ACCEL_XY_MAX
    if not math.isfinite(v_dx):
        v_dx = 0.0
    if not math.isfinite(v_dy):
        v_dy = 0.0
    if not math.isfinite(v_x):
        v_x = 0.0
    if not math.isfinite(v_y):
        v_y = 0.0
    if not math.isfinite(kv) or kv < 0.0:
        kv = 0.0
    a_bx = kv * (v_dx - v_x)
    a_by = kv * (v_dy - v_y)
    a_h = math.hypot(a_bx, a_by)
    if a_h > accel_max > 0.0:
        scale = accel_max / a_h
        a_bx *= scale
        a_by *= scale
    return a_bx, a_by


def next_att_source(source, diag_stamp, now, timeout=None):
    """Expire diagnostic ownership → velocity (default cmd_vel path)."""
    if timeout is None:
        timeout = K_DIAG_TIMEOUT
    if source in ('attitude', 'accel') and (now - diag_stamp) > timeout:
        return 'velocity'
    return source


def accel_to_attitude(a_e, a_n, yaw, g=None, tilt_max=None, accel_max=None):
    """Map desired ENU horizontal accel [m/s²] → (roll_d, pitch_d) [rad].

    Verified Webots/Mavic signs (CTRL-A3 live + thrust geometry):
      a_bx ≈ +g tan(pitch)  →  pitch_d = atan2(a_bx, g)
      a_by ≈ -g tan(roll)   →  roll_d  = atan2(-a_by, g)
    Zero accel → level. Nonfinite → level. Combined tilt via |a|≤g tan(tilt_max).
    """
    if g is None:
        g = G_ACCEL
    if tilt_max is None:
        tilt_max = K_TILT_MAX
    if accel_max is None:
        accel_max = K_ACCEL_XY_MAX
    if not (math.isfinite(a_e) and math.isfinite(a_n) and math.isfinite(yaw) and g > 0.0):
        return 0.0, 0.0
    a_h = math.hypot(a_e, a_n)
    if a_h > accel_max > 0.0:
        scale = accel_max / a_h
        a_e *= scale
        a_n *= scale
    # Combined tilt: |a_body| ≤ g tan(tilt_max)
    a_tilt_max = g * math.tan(tilt_max)
    a_bx, a_by = enu_accel_to_body(a_e, a_n, yaw)
    a_bh = math.hypot(a_bx, a_by)
    if a_bh > a_tilt_max > 0.0:
        scale = a_tilt_max / a_bh
        a_bx *= scale
        a_by *= scale
    pitch_d = math.atan2(a_bx, g)
    roll_d = math.atan2(-a_by, g)
    # Independent atan axes can slightly exceed combined thrust tilt;
    # iterate Euler scale so acos(cosφ cosθ) ≤ tilt_max.
    for _ in range(4):
        cos_a = clamp(math.cos(roll_d) * math.cos(pitch_d), -1.0, 1.0)
        alpha = math.acos(cos_a)
        if alpha <= tilt_max or alpha <= 1e-12:
            break
        scale = tilt_max / alpha
        roll_d *= scale
        pitch_d *= scale
    return roll_d, pitch_d


def collective_tilt_compensation(roll_d, pitch_d, tilt_max=None):
    """Scale factor for collective motor ω under T∝ω² so vertical thrust holds.

    T_z = T cosα with cosα = cos(φ)cos(θ); T = 4 c ω².
    Preserve T_z ⇒ ω = ω0 / sqrt(cosα). Unity at level. Bounded near tilt_max.
    Does not add gravity; only compensates geometric cosine loss.
    """
    if tilt_max is None:
        tilt_max = K_TILT_MAX
    if not (math.isfinite(roll_d) and math.isfinite(pitch_d)):
        return 1.0
    cos_a = math.cos(roll_d) * math.cos(pitch_d)
    cos_floor = math.cos(tilt_max)
    if cos_a < cos_floor:
        cos_a = cos_floor
    if cos_a >= 1.0 - 1e-12:
        return 1.0
    if cos_a <= 1e-6:
        return 1.0 / math.sqrt(max(cos_floor, 1e-6))
    return 1.0 / math.sqrt(cos_a)


def _resolve_log_path(filename):
    """Write driver CSVs under a writable dir (webots-controller cwd may be /)."""
    if os.path.isabs(filename):
        path = filename
    else:
        base = os.environ.get(
            'WEBOT_DRIVER_LOG_DIR',
            os.path.join(os.path.expanduser('~'), '.ros', 'webot_package_logs'),
        )
        os.makedirs(base, exist_ok=True)
        path = os.path.join(base, filename)
    parent = os.path.dirname(path)
    if parent:
        os.makedirs(parent, exist_ok=True)
    return path


class Logger:
    def __init__(self, filename):
        self.filename = _resolve_log_path(filename)
        self.start_time = time.time()
        # Open the file in append mode
        with open(self.filename, 'w') as file:
            # Write the header row
            file.write('time,velX_d,velX,velY_d,velY,z_d,z,velYaw_d,velYaw\n')
    
    def log(self, *variables):
        current_time = time.time()
        elapsed_time = current_time - self.start_time
        
        # Format elapsed time to ss.cc
        ss = int(elapsed_time)
        cc = int((elapsed_time - ss) * 100)
        label_time = f'{ss:02}.{cc:02}'
        
         # Format numeric variables to six decimal places
        formatted_variables = [f'{float(var):.6f}' for var in variables]
        
        # Convert formatted variables to a comma-separated string
        variables_str = ','.join(formatted_variables)
        
        # Prepare the log line
        log_line = f'{label_time},{variables_str}\n'
        
        # Write the log line to the file
        with open(self.filename, 'a') as file:
            file.write(log_line)

class Logger_h:
    def __init__(self, filename):
        self.filename = _resolve_log_path(filename)
        self.start_time = time.time()
        # Check if the file exists
        if not os.path.exists(self.filename):
            # Create the file if it does not exist
            # Open the file in append mode
            with open(self.filename, 'w') as file:
                # Write the header row
                file.write('time,error\n')
    
    def log(self, *variables):
        current_time = time.time()
        elapsed_time = current_time - self.start_time
        
        # Format elapsed time to ss.cc
        ss = int(elapsed_time)
        cc = int((elapsed_time - ss) * 100)
        label_time = f'{ss:02}.{cc:02}'
        
         # Format numeric variables to six decimal places
        formatted_variables = [f'{float(var):.6f}' for var in variables]
        
        # Convert formatted variables to a comma-separated string
        variables_str = ','.join(formatted_variables)
        
        # Prepare the log line
        log_line = f'{label_time},{variables_str}\n'
        
        # Write the log line to the file
        with open(self.filename, 'a') as file:
            file.write(log_line)


def clamp(value, value_min, value_max):
    return min(max(value, value_min), value_max)


def saturate_attitude_setpoint(angle_rad, limit_rad=None):
    """Clamp attitude setpoint [rad]; nonfinite → 0."""
    if limit_rad is None:
        limit_rad = K_ATT_LIMIT
    if not math.isfinite(angle_rad):
        return 0.0
    return clamp(angle_rad, -limit_rad, limit_rad)


class MavicDriver:
    def init(self, webots_node, properties):
        self.__robot = webots_node.robot
        self.__timestep = int(self.__robot.getBasicTimeStep())

        # Sensors
        self.__gps = self.__robot.getDevice('gps')
        self.__gyro = self.__robot.getDevice('gyro')
        self.__imu = self.__robot.getDevice('inertial unit')
        self.__camera = self.__robot.getDevice('camera')
        #self.__range = self.__robot.getDevice('ps0')
        
        distance_sensor = self.__robot.getDevice("ds0")
        distance_sensor.enable(1000)  # Enable the distance sensor with a sampling period of 10 milliseconds
        # B3: TFA300-L-inspired laser (URDF ROS plugin also publishes /ds_tfa300l @ 50 Hz).
        laser_sensor = self.__robot.getDevice("ds_tfa300l")
        if laser_sensor is not None:
            laser_sensor.enable(20)  # 20 ms ≈ 50 Hz
        
        print(distance_sensor)

        # Propellers
        self.__propellers = [
            self.__robot.getDevice('front right propeller'),
            self.__robot.getDevice('front left propeller'),
            self.__robot.getDevice('rear right propeller'),
            self.__robot.getDevice('rear left propeller')
        ]
        for propeller in self.__propellers:
            propeller.setPosition(float('inf'))
            propeller.setVelocity(0)

        # State
        self.__target_twist = Twist()
        self.__vertical_ref = LIFT_HEIGHT
        self.__linear_x_integral = 0
        self.__linear_y_integral = 0
        self.__SLAM_robot_state = Robotstate()
        # Latch spawn GPS altitude so vertical control is AGL-relative.
        # Required for Earth worlds where absolute Z is ~15 m (Mars spawn is ~0).
        self.__gps_z0 = None
        # Single-underscore: avoid getattr('__…') missing double-underscore mangling.
        self._alt_hold_mode = False
        # CTRL-S1/S2/S3: exclusive attitude ownership (never blended).
        #   'velocity'  → default: cmd_vel body v → a → S2/S1 (incl. brake at 0)
        #   'attitude'  → /attitude_setpoint [rad] diagnostic (timeout/NaN → velocity)
        #   'accel'     → /accel_setpoint ENU [m/s²] diagnostic (timeout/NaN → velocity)
        # A2 velocity→tilt (mixer units) remains disconnected.
        # cmd_vel.x/y interpreted as body-frame velocity setpoints matching GPS
        # body velocity (no extra Y negate here; ControlQuad's publish-time
        # Y negate is a high-level concern — avoid double negation in S3).
        self._att_source = 'velocity'
        self._diag_stamp = 0.0
        self._roll_d_cmd = 0.0
        self._pitch_d_cmd = 0.0
        self._a_e_cmd = 0.0
        self._a_n_cmd = 0.0
        self._roll_d = 0.0
        self._pitch_d = 0.0
        # CTRL-R4b/R4c: exclusive unified ENU control (disables legacy loops).
        self._unified_mode = False
        self._unified_p_d = (0.0, 0.0, 0.0)
        self._unified_sat = False
        # R4c trajectory lifecycle: None | 'running' | 'hold' (completed).
        self._traj = None
        self._traj_t0 = None
        self._traj_state = 'hold'  # hold | running | done
        self._traj_v_d = (0.0, 0.0, 0.0)
        self._traj_a_d = (0.0, 0.0, 0.0)
        # R4d: acknowledged request / settle status.
        self._traj_mgr = TrajRequestManager(still_speed=TRAJ_STILL_SPEED)
        self._last_status_pub = None
        # R5a production defaults: XY 3.0 / Z 1.5 traj, tracker XY 3.5.
        self._traj_v_max = float(TRAJ_V_MAX)  # legacy isotropic alias
        self._traj_v_max_xy = float(TRAJ_V_MAX_XY)
        self._traj_v_max_z = float(TRAJ_V_MAX_Z)
        self._traj_a_max = float(TRAJ_A_MAX)
        self._traj_j_max = float(TRAJ_J_MAX)
        self._tracker_v_max_xy = float(V_MAX_XY)
        self._tracker_v_max_z = float(V_MAX_Z)
        self._traj_profile_ok = True
        self._traj_profile_msg = "default R5a"

        # ROS interface
        rclpy.init(args=None)
        self.__node = rclpy.create_node('mavic_driver')
        self._load_traj_profile_params()

        self.__node.create_subscription(Twist, 'cmd_vel', self.__cmd_vel_callback, 1)
        # S1 test path: Vector3.x = roll_d [rad], .y = pitch_d [rad], .z ignored.
        self.__node.create_subscription(
            Vector3, 'attitude_setpoint', self.__attitude_setpoint_callback, 1
        )
        # S2 temporary test path: Vector3.x = a_E, .y = a_N [m/s²], .z ignored.
        self.__node.create_subscription(
            Vector3, 'accel_setpoint', self.__accel_setpoint_callback, 1
        )
        # R4b: ENU position hold. Finite x,y,z → enable hold. NaN x → disable unified.
        self.__node.create_subscription(
            Vector3, 'unified_position_setpoint',
            self.__unified_position_setpoint_callback, 1
        )
        # R4c: ENU trajectory target (rest-to-rest). Finite → start traj if still.
        self.__node.create_subscription(
            Vector3, 'unified_trajectory_target',
            self.__unified_trajectory_target_callback, 1
        )
        
        self.__node.create_subscription(Robotstate, 'Robotstate_topic', self.__slam_robot_state_callback, 1)
        
        self.__node.create_service(SimpleServ, "webot_service", self.__service_callback)
        self.__node.create_service(
            UnifiedTrajectory, "unified_trajectory",
            self.__unified_trajectory_service
        )
        self._pub_traj_status = self.__node.create_publisher(
            UnifiedTrajectoryStatus, "unified_trajectory_status", 10
        )
    
        self.__land = False
        
        # log file
        self.logger_lc = Logger('logfile_lc.csv')
        self.logger_eh = Logger_h('logfile_eh.csv')
        
    def __service_callback(self, request, response):
        global K_VERTICAL_THRUST
        cmd = request.cmd
        self.__node.get_logger().info('Incoming request\n cmd: %d' % (cmd) )
        if cmd == 116:  # 't'
            self.__node.get_logger().info("Take off..")
            self.__land = False
            K_VERTICAL_THRUST = 68.5
        if cmd == 108:  # 'l'
            self.__node.get_logger().info("Landing..")
            #K_VERTICAL_THRUST = 0.0  
            self.__land = True
            self._exit_unified_mode()
            error_to_home = math.sqrt(self.gps_x**2 + self.gps_y**2)
            if error_to_home < 1.0: 
                self.logger_eh.log(error_to_home)
        if cmd == 117:  # 'u' — R4b unified position hold (latch current GPS ENU)
            self._enter_unified_hold_latch()
        if cmd == 120:  # exit unified → legacy cmd_vel path
            self.__node.get_logger().info("R4b: exit unified position hold")
            self._exit_unified_mode()
        if cmd == 121:  # cancel active trajectory → hold final/current p_d
            self._cancel_trajectory(hold_current=True)
            
        response.response = True
        return response

    def _clear_trajectory(self):
        self._traj = None
        self._traj_t0 = None
        self._traj_state = 'hold'
        self._traj_v_d = (0.0, 0.0, 0.0)
        self._traj_a_d = (0.0, 0.0, 0.0)

    @staticmethod
    def _env_float(name, default):
        """Optional WEBOT_* env override (Webots plugin node has no CLI -p)."""
        raw = os.environ.get(name, '').strip()
        if not raw:
            return default
        try:
            return float(raw)
        except ValueError:
            return default

    def _load_traj_profile_params(self):
        """R5/R5a: traj v/a/j + optional XY/Z; tracker XY configurable.

        Precedence:
        1. WEBOT_TRAJ_* / WEBOT_TRACKER_* environment (set by launch2) —
           required because MavicDriver creates node ``mavic_driver`` inside
           the Webots plugin process; CLI ``-p`` applies to the outer driver.
        2. ROS parameters on ``mavic_driver`` (if set).
        3. Code defaults (R5a production: traj XY 3.0 / Z 1.5, tracker XY 3.5).

        ``traj_v_max_xy`` / ``traj_v_max_z``: if >0 use that axis; if ≤0
        inherit ``traj_v_max`` (legacy isotropic).
        """
        node = self.__node
        node.declare_parameter('traj_v_max', float(TRAJ_V_MAX))
        node.declare_parameter('traj_v_max_xy', float(TRAJ_V_MAX_XY))
        node.declare_parameter('traj_v_max_z', float(TRAJ_V_MAX_Z))
        node.declare_parameter('traj_a_max', float(TRAJ_A_MAX))
        node.declare_parameter('traj_j_max', float(TRAJ_J_MAX))
        node.declare_parameter('tracker_v_max_xy', float(V_MAX_XY))
        node.declare_parameter('tracker_v_max_z', float(V_MAX_Z))
        self._traj_v_max = self._env_float(
            'WEBOT_TRAJ_V_MAX', float(node.get_parameter('traj_v_max').value))
        self._traj_a_max = self._env_float(
            'WEBOT_TRAJ_A_MAX', float(node.get_parameter('traj_a_max').value))
        self._traj_j_max = self._env_float(
            'WEBOT_TRAJ_J_MAX', float(node.get_parameter('traj_j_max').value))
        self._tracker_v_max_xy = self._env_float(
            'WEBOT_TRACKER_V_MAX_XY',
            float(node.get_parameter('tracker_v_max_xy').value))
        self._tracker_v_max_z = self._env_float(
            'WEBOT_TRACKER_V_MAX_Z',
            float(node.get_parameter('tracker_v_max_z').value))
        raw_xy = self._env_float(
            'WEBOT_TRAJ_V_MAX_XY',
            float(node.get_parameter('traj_v_max_xy').value))
        raw_z = self._env_float(
            'WEBOT_TRAJ_V_MAX_Z',
            float(node.get_parameter('traj_v_max_z').value))
        self._traj_v_max_xy, self._traj_v_max_z = resolve_effective_limits(
            self._traj_v_max, raw_xy, raw_z,
        )
        ok, msg = traj_feasibility_xyz(
            self._traj_v_max_xy, self._traj_v_max_z,
            self._traj_a_max, self._traj_j_max,
            tracker_v_xy=self._tracker_v_max_xy,
            tracker_v_z=self._tracker_v_max_z,
        )
        self._traj_profile_ok = bool(ok)
        self._traj_profile_msg = str(msg)
        node.get_logger().info(
            "R5a traj v_xy=%.3f v_z=%.3f a=%.3f j=%.3f "
            "tracker_xy=%.3f tracker_z=%.3f feasible=%s (%s)" % (
                self._traj_v_max_xy, self._traj_v_max_z,
                self._traj_a_max, self._traj_j_max,
                self._tracker_v_max_xy, self._tracker_v_max_z,
                self._traj_profile_ok, self._traj_profile_msg)
        )
        if not self._traj_profile_ok:
            node.get_logger().error(
                "R5a: trajectory profile infeasible vs tracker sats — "
                "unified traj starts will be rejected (no silent clip)"
            )

    def _publish_traj_status(self, snap):
        msg = UnifiedTrajectoryStatus()
        msg.request_id = int(snap.request_id)
        msg.status = int(snap.status)
        msg.ex = float(snap.ex)
        msg.ey = float(snap.ey)
        msg.ez = float(snap.ez)
        msg.vh = float(snap.vh)
        msg.vz = float(snap.vz)
        msg.message = str(snap.message)
        self._pub_traj_status.publish(msg)
        self._last_status_pub = snap

    def _cancel_trajectory(self, hold_current=False):
        if hold_current:
            try:
                x, y, z = self.__gps.getValues()
                if not any(math.isnan(v) for v in (x, y, z)):
                    self._unified_p_d = (float(x), float(y), float(z))
            except Exception:
                pass
        snap = self._traj_mgr.cancel()
        self._clear_trajectory()
        self.__node.get_logger().info(
            "R4c: trajectory cancelled; hold p_d=(%.3f, %.3f, %.3f)" % self._unified_p_d
        )
        if snap.status == STATUS_CANCELLED:
            self._publish_traj_status(snap)

    def _enter_unified_hold_latch(self):
        """Latch p_d to current GPS ENU; exclusive ownership of motors."""
        try:
            x, y, z = self.__gps.getValues()
        except Exception:
            self.__node.get_logger().error("R4b: cannot latch — GPS unavailable")
            return
        if any(math.isnan(v) for v in (x, y, z)):
            self.__node.get_logger().error("R4b: cannot latch — GPS NaN")
            return
        self._clear_trajectory()
        self._traj_mgr.reset_idle()
        self._traj_mgr.enter_unified()
        self._unified_p_d = (float(x), float(y), float(z))
        self._unified_mode = True
        self._unified_sat = False
        self._att_source = 'velocity'
        self._roll_d = 0.0
        self._pitch_d = 0.0
        self.__target_twist = Twist()
        self.__node.get_logger().info(
            "R4b: unified hold latch p_d=(%.3f, %.3f, %.3f)" % self._unified_p_d
        )

    def _exit_unified_mode(self):
        self._unified_mode = False
        self._unified_sat = False
        self._clear_trajectory()
        self._traj_mgr.exit_unified()
        self._att_source = 'velocity'
        self._a_e_cmd = 0.0
        self._a_n_cmd = 0.0
        self._roll_d_cmd = 0.0
        self._pitch_d_cmd = 0.0

    def _start_trajectory_to(self, target, request_id=None):
        """Rest-to-rest traj from current GPS to target ENU. Requires unified + still."""
        if not self._unified_mode:
            self.__node.get_logger().warn("R4c: traj rejected — not in unified mode")
            return False
        try:
            x, y, z = self.__gps.getValues()
            vx, vy, vz = self.__gps.getSpeedVector()
        except Exception:
            self.__node.get_logger().error("R4c: traj rejected — GPS unavailable")
            return False
        if any(math.isnan(v) for v in (x, y, z, vx, vy, vz)):
            self.__node.get_logger().error("R4c: traj rejected — GPS NaN")
            return False
        if not all(math.isfinite(c) for c in target):
            self.__node.get_logger().error("R4c: traj rejected — nonfinite target")
            return False
        speed = measured_speed((vx, vy, vz))
        traj_busy = (
            self._traj is not None
            and self._traj_state == 'running'
        )
        # When called via ACK service, manager already validated; topic path uses id=0.
        if request_id is None:
            res = self._traj_mgr.handle_request(
                0, target, speed, self._unified_mode, traj_busy
            )
            if not res.accepted:
                self.__node.get_logger().warn(
                    "R4c: traj rejected — %s" % res.message
                )
                return False
        elif speed > TRAJ_STILL_SPEED:
            # Defense in depth if service path skipped gate.
            self.__node.get_logger().warn(
                "R4c: traj rejected — speed %.3f > %.3f (rest-to-rest)" % (
                    speed, TRAJ_STILL_SPEED)
            )
            return False
        if not self._traj_profile_ok:
            self.__node.get_logger().error(
                "R5a: traj rejected — infeasible profile (%s)" % self._traj_profile_msg
            )
            return False
        p0 = (float(x), float(y), float(z))
        pf = (float(target[0]), float(target[1]), float(target[2]))
        ok_seg, msg_seg, v_path = segment_feasibility(
            p0, pf,
            self._traj_v_max_xy, self._traj_v_max_z,
            self._traj_a_max, self._traj_j_max,
            tracker_v_xy=self._tracker_v_max_xy,
            tracker_v_z=self._tracker_v_max_z,
        )
        if not ok_seg:
            self.__node.get_logger().error(
                "R5a: traj rejected — segment infeasible (%s)" % msg_seg
            )
            return False
        # Zero-length: treat as immediate hold at pf (no planner).
        if v_path <= 0.0:
            self._clear_trajectory()
            self._unified_p_d = pf
            self._traj_mgr.mark_running()
            self._traj_mgr.mark_traj_time_done()
            self.__node.get_logger().info(
                "R5a: zero-length traj → hold pf=(%.3f,%.3f,%.3f)" % pf
            )
            return True
        try:
            traj = JerkLimitedTrajectory(
                p0, pf,
                v_max=float(v_path),
                a_max=self._traj_a_max,
                j_max=self._traj_j_max,
                dt=self.__timestep / 1000.0,
            )
        except ValueError as exc:
            self.__node.get_logger().error("R4c: traj rejected — %s" % exc)
            return False
        self._traj = traj
        self._traj_t0 = float(self.__robot.getTime())
        self._traj_state = 'running'
        self._unified_p_d = pf  # terminal hold reference
        self._traj_mgr.mark_running()
        self.__node.get_logger().info(
            "R4c: traj start p0=(%.2f,%.2f,%.2f) pf=(%.2f,%.2f,%.2f) "
            "T=%.2fs id=%s v_path=%.2f (xy=%.2f z=%.2f) a=%.2f j=%.2f" % (
                p0[0], p0[1], p0[2], pf[0], pf[1], pf[2], traj.duration,
                request_id if request_id is not None else 0,
                v_path, self._traj_v_max_xy, self._traj_v_max_z,
                self._traj_a_max, self._traj_j_max)
        )
        return True

    def __unified_trajectory_service(self, request, response):
        """R4d ACK trajectory request (absolute GPS ENU)."""
        target = (float(request.x), float(request.y), float(request.z))
        rid = int(request.request_id)
        if not self._unified_mode:
            self._enter_unified_hold_latch()
        speed = 0.0
        try:
            vx, vy, vz = self.__gps.getSpeedVector()
            if not any(math.isnan(v) for v in (vx, vy, vz)):
                speed = measured_speed((vx, vy, vz))
        except Exception:
            speed = 0.0
        traj_busy = (
            self._traj is not None and self._traj_state == 'running'
        )
        res = self._traj_mgr.handle_request(
            rid, target, speed, self._unified_mode, traj_busy
        )
        response.accepted = bool(res.accepted)
        response.request_id = int(res.request_id)
        response.status = int(res.status)
        response.message = str(res.message)
        if res.accepted and res.status in (STATUS_ACCEPTED,):
            started = self._start_trajectory_to(target, request_id=rid)
            if not started:
                self._traj_mgr.status = STATUS_REJECTED
                self._traj_mgr.message = "start failed"
                response.accepted = False
                response.status = STATUS_REJECTED
                response.message = "start failed"
            else:
                response.status = STATUS_RUNNING
                response.message = "running"
                self._publish_traj_status(self._traj_mgr.snapshot())
        elif res.status == STATUS_REJECTED:
            self._publish_traj_status(StatusSnapshot(
                request_id=rid, status=STATUS_REJECTED, message=res.message
            ))
        return response

    def __unified_position_setpoint_callback(self, msg):
        # NaN x → disable unified (return to legacy). Finite → fixed hold (cancels traj).
        if not math.isfinite(msg.x):
            self._exit_unified_mode()
            self.__node.get_logger().info("R4b: unified disabled via setpoint NaN")
            return
        if not (math.isfinite(msg.y) and math.isfinite(msg.z)):
            self.__node.get_logger().warn("R4b: ignore setpoint — y/z nonfinite")
            return
        self._clear_trajectory()
        self._unified_p_d = (float(msg.x), float(msg.y), float(msg.z))
        self._unified_mode = True
        self._att_source = 'velocity'
        self.__target_twist = Twist()
        self.__node.get_logger().info(
            "R4b: unified hold p_d=(%.3f, %.3f, %.3f)" % self._unified_p_d
        )

    def __unified_trajectory_target_callback(self, msg):
        if not (math.isfinite(msg.x) and math.isfinite(msg.y) and math.isfinite(msg.z)):
            self.__node.get_logger().warn("R4c: ignore traj target — nonfinite")
            return
        if not self._unified_mode:
            # Auto-enter unified from current pose, then start (if still).
            self._enter_unified_hold_latch()
            if not self._unified_mode:
                return
        self._start_trajectory_to((msg.x, msg.y, msg.z))
        
    def __cmd_vel_callback(self, twist):
        # Unified mode owns motors — ignore competing cmd_vel.
        if self._unified_mode:
            return
        # Sanitize nonfinite horizontal commands → 0 (brake / hover).
        t = Twist()
        t.linear.x = float(twist.linear.x) if math.isfinite(twist.linear.x) else 0.0
        t.linear.y = float(twist.linear.y) if math.isfinite(twist.linear.y) else 0.0
        t.linear.z = float(twist.linear.z) if math.isfinite(twist.linear.z) else 0.0
        t.angular.z = float(twist.angular.z) if math.isfinite(twist.angular.z) else 0.0
        self.__target_twist = t

    def __attitude_setpoint_callback(self, msg):
        if self._unified_mode:
            return
        # Diagnostic angles [rad]. NaN → release ownership back to velocity.
        if not (math.isfinite(msg.x) and math.isfinite(msg.y)):
            self._att_source = 'velocity'
            self._roll_d_cmd = 0.0
            self._pitch_d_cmd = 0.0
            return
        self._att_source = 'attitude'
        self._diag_stamp = time.time()
        self._a_e_cmd = 0.0
        self._a_n_cmd = 0.0
        self._roll_d_cmd = saturate_attitude_setpoint(msg.x)
        self._pitch_d_cmd = saturate_attitude_setpoint(msg.y)

    def __accel_setpoint_callback(self, msg):
        if self._unified_mode:
            return
        # Diagnostic ENU accel [m/s²]. NaN → release ownership back to velocity.
        if not (math.isfinite(msg.x) and math.isfinite(msg.y)):
            self._att_source = 'velocity'
            self._a_e_cmd = 0.0
            self._a_n_cmd = 0.0
            self._roll_d_cmd = 0.0
            self._pitch_d_cmd = 0.0
            return
        self._att_source = 'accel'
        self._diag_stamp = time.time()
        a_h = math.hypot(msg.x, msg.y)
        if a_h > K_ACCEL_XY_MAX > 0.0:
            scale = K_ACCEL_XY_MAX / a_h
            self._a_e_cmd = msg.x * scale
            self._a_n_cmd = msg.y * scale
        else:
            self._a_e_cmd = float(msg.x)
            self._a_n_cmd = float(msg.y)

    def __slam_robot_state_callback(self,Robotstate):
        self.__SLAM_robot_state = Robotstate

    def step(self):
        rclpy.spin_once(self.__node, timeout_sec=0)

        velocity_x = 0.0
        velocity_y = 0.0

        # Read sensors
        roll, pitch, yaw = self.__imu.getRollPitchYaw()
        self.gps_x, self.gps_y, gps_z = self.__gps.getValues()  # barometer / optical-flow stand-in
        roll_velocity, pitch_velocity, twist_yaw = self.__gyro.getValues()

        if self.__gps_z0 is None and not math.isnan(gps_z):
            self.__gps_z0 = gps_z
        # AGL-relative altitude (Earth worlds spawn far above z=0).
        vertical = gps_z - (0.0 if self.__gps_z0 is None else self.__gps_z0)

        # Horizontal ground speed only — total GPS speed includes climb and
        # was being mis-read as lateral motion (destabilized fast ascents).
        try:
            gps_vx, gps_vy, gps_vz = self.__gps.getSpeedVector()
            if any(math.isnan(v) for v in (gps_vx, gps_vy, gps_vz)):
                raise ValueError('nan speed vector')
            velocity_h = math.hypot(gps_vx, gps_vy)
            # Body-frame horizontal velocity from yaw (ENU → body).
            c = math.cos(yaw)
            s = math.sin(yaw)
            velocity_x = c * gps_vx + s * gps_vy
            velocity_y = -s * gps_vx + c * gps_vy
        except Exception:
            vel_gps = self.__gps.getSpeed()
            if math.isnan(vel_gps):
                return
            # Fallback: attitude projection of horizontal-only estimate unavailable
            attitude_norm = abs(roll) + abs(pitch)
            if attitude_norm > 1e-3:
                velocity_x = (pitch / attitude_norm) * vel_gps
                velocity_y = -(roll / attitude_norm) * vel_gps
            else:
                velocity_x = 0.0
                velocity_y = 0.0
            velocity_h = math.hypot(velocity_x, velocity_y)

        vel_linear = self.__SLAM_robot_state.vels.linear
        vel_slam = math.sqrt(vel_linear.x**2 + vel_linear.y**2)

        #print("gps_vel_h: {}, slam_vel: {}".format(velocity_h, vel_slam ))

        dt = self.__timestep / 1000.0
        global K_VERTICAL_THRUST

        # ENU velocity (gps speed vector); fallback zeros if missing.
        try:
            gps_vx, gps_vy, gps_vz = self.__gps.getSpeedVector()
            if any(math.isnan(v) for v in (gps_vx, gps_vy, gps_vz)):
                raise ValueError('nan')
            v_enu = (float(gps_vx), float(gps_vy), float(gps_vz))
            vz = float(gps_vz)
        except Exception:
            v_enu = (0.0, 0.0, 0.0)
            vz = 0.0

        # ---------------------------------------------------------------
        # CTRL-R4b/R4c: exclusive unified ENU cascade (+ optional traj FF).
        # Disables legacy climb/hold, cmd_vel S3, and diagnostic att/accel.
        # ---------------------------------------------------------------
        if self._unified_mode and not self.__land:
            p_enu = (float(self.gps_x), float(self.gps_y), float(gps_z))
            # Sample active trajectory in simulation time.
            v_d = (0.0, 0.0, 0.0)
            a_d = (0.0, 0.0, 0.0)
            p_ref = self._unified_p_d
            if self._traj is not None and self._traj_t0 is not None:
                t_el = float(self.__robot.getTime()) - self._traj_t0
                if t_el < 0.0:
                    t_el = 0.0
                samp = self._traj.sample(t_el)
                p_ref = samp.position
                v_d = samp.velocity
                a_d = samp.acceleration
                if samp.complete:
                    # Exact terminal hold; keep feedback active (no separate settle law).
                    self._unified_p_d = self._traj.pf
                    p_ref = self._unified_p_d
                    v_d = (0.0, 0.0, 0.0)
                    a_d = (0.0, 0.0, 0.0)
                    self._traj_mgr.mark_traj_time_done()
                    if self._traj_state != 'done':
                        self._traj_state = 'done'
                        self.__node.get_logger().info(
                            "R4c: traj complete → hold pf=(%.3f, %.3f, %.3f)" % (
                                self._unified_p_d)
                        )
                else:
                    self._traj_state = 'running'
                self._traj_v_d = v_d
                self._traj_a_d = a_d
            # R4d settle: after planned time, require band + dwell (same unified law).
            if self._traj_mgr.status in (STATUS_ACCEPTED, STATUS_RUNNING):
                snap = self._traj_mgr.update_settle(p_enu, v_enu, dt)
                if snap.status == STATUS_SETTLED:
                    self.__node.get_logger().info(
                        "R4d: settled id=%d e=(%.3f,%.3f,%.3f) vh=%.3f vz=%.3f dwell_ok" % (
                            snap.request_id, snap.ex, snap.ey, snap.ez,
                            snap.vh, snap.vz)
                    )
                    self._publish_traj_status(snap)
                elif self._traj_mgr.traj_time_elapsed:
                    # Publish settling progress sparsely via last change of message.
                    if (
                        self._last_status_pub is None
                        or self._last_status_pub.message != snap.message
                        or self._last_status_pub.status != snap.status
                    ):
                        self._publish_traj_status(snap)
            if any(math.isnan(c) for c in p_enu):
                collective = HOVER_OMEGA if K_VERTICAL_THRUST > 0 else 0.0
                roll_cmd = pitch_cmd = 0.0
            else:
                _v_sp, a_sp, _f_d, collective, self._unified_sat = unified_step(
                    p_ref, p_enu, v_enu, roll, pitch, v_d=v_d, a_d=a_d,
                    v_max_xy=self._tracker_v_max_xy,
                    v_max_z=self._tracker_v_max_z,
                )
                roll_cmd, pitch_cmd = accel_to_attitude(a_sp[0], a_sp[1], yaw)
            if abs(roll) > K_ATT_SAFE or abs(pitch) > K_ATT_SAFE:
                roll_cmd = 0.0
                pitch_cmd = 0.0
            max_step = K_ATT_SLEW * max(dt, 1e-3)
            self._roll_d = clamp(
                roll_cmd, self._roll_d - max_step, self._roll_d + max_step
            )
            self._pitch_d = clamp(
                pitch_cmd, self._pitch_d - max_step, self._pitch_d + max_step
            )
            roll_input = clamp(
                attitude_pd(roll, self._roll_d, roll_velocity, K_ROLL_P, K_ROLL_D),
                -K_ATT_MIXER_LIMIT,
                K_ATT_MIXER_LIMIT,
            )
            pitch_input = clamp(
                attitude_pd(
                    pitch, self._pitch_d, pitch_velocity, K_PITCH_P, K_PITCH_D
                ),
                -K_ATT_MIXER_LIMIT,
                K_ATT_MIXER_LIMIT,
            )
            yaw_ref = 0.0
            yaw_input = K_YAW_P * (yaw_ref - twist_yaw)
            # Collective already includes tilt compensation — do not re-apply
            # collective_tilt_compensation (would double-count cosine).
            if not math.isfinite(collective) or collective < 0.0:
                collective = HOVER_OMEGA if K_VERTICAL_THRUST > 0 else 0.0
            m1 = collective + yaw_input + pitch_input + roll_input
            m2 = collective - yaw_input + pitch_input - roll_input
            m3 = collective - yaw_input - pitch_input + roll_input
            m4 = collective + yaw_input - pitch_input - roll_input
            self.__propellers[0].setVelocity(-m1)
            self.__propellers[1].setVelocity(m2)
            self.__propellers[2].setVelocity(m3)
            self.__propellers[3].setVelocity(-m4)
            if vertical > 1:
                self.logger_lc.log(
                    self._unified_p_d[0], self.gps_x,
                    self._unified_p_d[1], self.gps_y,
                    self._unified_p_d[2], gps_z,
                    yaw_ref, twist_yaw,
                )
            return

        # ---- Legacy path (inactive while unified_mode) --------------------
        # Allow high level altitude control once the drone is lifted.
        # CTRL-S1: A2 velocity→tilt (mixer disturbance units) is NOT connected
        # to attitude setpoints. Hold level unless attitude_setpoint is set.
        if vertical > 0.2:
            # Dual-mode altitude (CTRL-A1):
            #   climb  — preserve fast rate (large lead + ref×1.35 + high P)
            #   hold   — freeze altitude setpoint + stronger vz damping
            # Do NOT bleed vertical_ref toward measured altitude: that couples any
            # thrust deficit into a permanent ~0.2 m/s walk-down after arrival.
            climb_cmd = self.__target_twist.linear.z
            xy_cmd = abs(self.__target_twist.linear.x) + abs(self.__target_twist.linear.y)
            # Hysteresis avoids climb/hold chatter near the waypoint.
            if self._alt_hold_mode:
                hold_mode = (abs(climb_cmd) < 0.55) or (xy_cmd > 0.10 and abs(climb_cmd) < 1.0)
            else:
                hold_mode = (abs(climb_cmd) < 0.30) or (xy_cmd > 0.20 and abs(climb_cmd) < 0.70)
            if self.__land == False:
                if hold_mode:
                    if not self._alt_hold_mode:
                        # Entering hold: kill leftover climb lead, lock hover AGL.
                        self.__vertical_ref = clamp(
                            self.__vertical_ref, vertical - 0.05, vertical + 0.20
                        )
                    # CTRL-S4: during XY motion, ignore small climb chatter so
                    # z_ref does not walk down with tilt-induced sink (S4 logs:
                    # cruise segments lost 1–2.7 m while z_d followed z).
                    if xy_cmd > 0.10:
                        if abs(climb_cmd) >= 0.45:
                            self.__vertical_ref = self.__vertical_ref + climb_cmd * dt
                    else:
                        self.__vertical_ref = self.__vertical_ref + climb_cmd * dt
                    lead = 4.0
                else:
                    lead = clamp(0.6 + 0.9 * abs(climb_cmd), 0.6, 2.2)
                    self.__vertical_ref = self.__vertical_ref + climb_cmd * dt * 1.35
                # Asymmetric band during XY hold: never drag z_ref down just
                # because measured altitude dipped (upper clamp tracks z).
                if hold_mode and xy_cmd > 0.10:
                    if self.__vertical_ref > vertical + lead:
                        self.__vertical_ref = vertical + lead
                    if self.__vertical_ref < LIFT_HEIGHT:
                        self.__vertical_ref = LIFT_HEIGHT
                else:
                    self.__vertical_ref = clamp(
                        self.__vertical_ref,
                        max(vertical - lead, LIFT_HEIGHT),
                        vertical + lead,
                    )
            else:
                self.__vertical_ref = clamp(
                    self.__vertical_ref + climb_cmd * dt,
                    max(vertical - 0.5, 0.15),
                    vertical + 0.15,
                )
            self._alt_hold_mode = hold_mode
        else:
            self._alt_hold_mode = False

        if self._alt_hold_mode:
            k_vert = 2.5
            k_d = K_VERTICAL_D
        else:
            k_vert = K_VERTICAL_P
            k_d = K_VERTICAL_D_CLIMB
        vertical_input = k_vert * (self.__vertical_ref - vertical) - k_d * vz

        # CTRL-S1/S2/S3: resolve exclusive attitude command, then track with PD.
        # Diagnostic ownership expires → normal cmd_vel velocity loop.
        prev_src = self._att_source
        self._att_source = next_att_source(
            self._att_source, self._diag_stamp, time.time()
        )
        if prev_src != self._att_source and self._att_source == 'velocity':
            self._a_e_cmd = 0.0
            self._a_n_cmd = 0.0
        if self._att_source == 'accel':
            roll_cmd, pitch_cmd = accel_to_attitude(
                self._a_e_cmd, self._a_n_cmd, yaw
            )
            self._roll_d_cmd = roll_cmd
            self._pitch_d_cmd = pitch_cmd
        elif self._att_source == 'attitude':
            roll_cmd = saturate_attitude_setpoint(self._roll_d_cmd)
            pitch_cmd = saturate_attitude_setpoint(self._pitch_d_cmd)
        else:
            # Default: body velocity P → body accel → ENU → S2 attitude.
            # v_d = 0 still runs the loop (brake toward measured v = 0).
            v_dx = self.__target_twist.linear.x
            v_dy = self.__target_twist.linear.y
            a_bx, a_by = velocity_to_accel(v_dx, v_dy, velocity_x, velocity_y)
            a_e, a_n = body_accel_to_enu(a_bx, a_by, yaw)
            self._a_e_cmd = a_e
            self._a_n_cmd = a_n
            roll_cmd, pitch_cmd = accel_to_attitude(a_e, a_n, yaw)
            self._roll_d_cmd = roll_cmd
            self._pitch_d_cmd = pitch_cmd
        # Safety: if already heavily tilted, drive setpoints to level.
        if abs(roll) > K_ATT_SAFE or abs(pitch) > K_ATT_SAFE:
            roll_cmd = 0.0
            pitch_cmd = 0.0
        max_step = K_ATT_SLEW * max(dt, 1e-3)
        self._roll_d = clamp(roll_cmd, self._roll_d - max_step, self._roll_d + max_step)
        self._pitch_d = clamp(pitch_cmd, self._pitch_d - max_step, self._pitch_d + max_step)
        roll_d = self._roll_d
        pitch_d = self._pitch_d
        roll_input = clamp(
            attitude_pd(roll, roll_d, roll_velocity, K_ROLL_P, K_ROLL_D),
            -K_ATT_MIXER_LIMIT,
            K_ATT_MIXER_LIMIT,
        )
        pitch_input = clamp(
            attitude_pd(pitch, pitch_d, pitch_velocity, K_PITCH_P, K_PITCH_D),
            -K_ATT_MIXER_LIMIT,
            K_ATT_MIXER_LIMIT,
        )
        yaw_ref = self.__target_twist.angular.z
        yaw_input = K_YAW_P * (yaw_ref - twist_yaw)

        # Collective: CTRL-A1 vertical_input + hover thrust, then S2 cosine
        # compensation on ω (T∝ω² ⇒ factor 1/sqrt(cosα)). Differentials unscaled.
        # CTRL-S4: use *measured* tilt for T_z = T cosα (desired leads during
        # accel/brake and under-compensates the actual cosine loss).
        collective = (K_VERTICAL_THRUST + vertical_input) * collective_tilt_compensation(
            roll, pitch
        )
        m1 = collective + yaw_input + pitch_input + roll_input
        m2 = collective - yaw_input + pitch_input - roll_input
        m3 = collective - yaw_input - pitch_input + roll_input
        m4 = collective + yaw_input - pitch_input - roll_input

        if self.__land == True and vertical < .2:
            m1 = 0
            m2 = 0
            m3 = 0
            m4 = 0

        # Apply control
        self.__propellers[0].setVelocity(-m1)
        self.__propellers[1].setVelocity(m2)
        self.__propellers[2].setVelocity(m3)
        self.__propellers[3].setVelocity(-m4)

        # log out
        if self.__land == False and vertical > 1:
            self.logger_lc.log(self.__target_twist.linear.x, velocity_x, self.__target_twist.linear.y, velocity_y, self.__vertical_ref, vertical, yaw_ref, twist_yaw)
