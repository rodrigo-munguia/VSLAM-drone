#!/usr/bin/env python3
"""Integrated quad experiment launch.

Webots is launched separately (see ./launch2.sh).

Viewer selection (propagated from launch2.sh):
  use_plot2:=true|false   Plot2 inside the app container (default true)

Global SLAM (G4, opt-in):
  use_globalslam:=true|false   default false (local-only)
  When true: load globalslam::Gslam with KF/reference remaps, same
  use_sim_time, Processing_start_enabled true. Requires YAML↔YAML Mono_cam
  match; rejects Calibration_source=camera_info.

G6d precedence (explicit CLI > YAML > node declare_parameter default):
  enable_global_ba / enable_tag_loop_closure / enable_visual_loop_closure
  apply_ba_corrections / apply_tag_loop_corrections / apply_visual_loop_corrections
  Empty launch args keep YAML (do not hard-overwrite). Tag+visual both on
  is rejected. Any effective Apply_*=true forces Coordinate_mode=required
  (rejects absent_ok/offline_bypass). No uncoordinated fallback.

Compatibility master:
  global_corrections:=true|false|''   empty = leave YAML Apply_global_corrections
  true forces Apply_* for each enabled source only (never enables visual
  processing). Explicit apply_*=false with master true → hard error.

RViz2 is started by launch2.sh as a separate process when --viewer rviz2.
"""

import os
import sys

import launch
import yaml
from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

# Sibling policy module (installed next to this launch script).
_SCRIPTS_DIR = os.path.dirname(os.path.abspath(__file__))
if _SCRIPTS_DIR not in sys.path:
    sys.path.insert(0, _SCRIPTS_DIR)
from g6d_launch_policy import resolve_gslam_g6d_params, resolve_slam_feedback  # noqa: E402


_MONO_CAM_KEYS = (
    'Mono_cam_img_rows',
    'Mono_cam_img_cols',
    'Mono_cam_distortions',
    'Mono_cam_cc_u',
    'Mono_cam_cc_v',
    'Mono_cam_fc_u',
    'Mono_cam_fc_v',
    'Mono_cam_alpha_c',
)


def _assert_globalslam_calibration(lslam_params, gslam_params):
    """G4: resolution alone is insufficient — require matching Mono_cam YAML."""
    cal = str(lslam_params.get('Calibration_source', 'yaml')).lower()
    if cal != 'yaml':
        raise RuntimeError(
            'G4 integrated Global SLAM requires Local SLAM Calibration_source=yaml '
            f'(got {cal!r}). CameraInfo-driven Local SLAM with Global SLAM YAML is '
            'unsupported until calibration is transported on keyframes. '
            'Use calibration_source:=yaml or omit --globalslam.')

    mismatches = []
    for key in _MONO_CAM_KEYS:
        if key not in lslam_params or key not in gslam_params:
            mismatches.append(f'{key} missing on /lslam or /gslam')
            continue
        if lslam_params[key] != gslam_params[key]:
            mismatches.append(
                f'{key}: /lslam={lslam_params[key]!r} /gslam={gslam_params[key]!r}')
    if mismatches:
        raise RuntimeError(
            'G4 integrated Global SLAM requires matching Mono_cam_* between '
            '/lslam and /gslam in config/quad_params.yaml:\n  - ' +
            '\n  - '.join(mismatches))


def _build(context, *args, **kwargs):
    bringup_share = get_package_share_directory('ekf_monoslam_bringup')
    default_config = os.path.join(bringup_share, 'config', 'quad_params.yaml')
    params_override = LaunchConfiguration('params_file').perform(context).strip()
    if params_override:
        config_file = os.path.abspath(params_override)
        if not os.path.isfile(config_file):
            raise RuntimeError(f'params_file not found: {config_file}')
    else:
        config_file = default_config
    default_plan = os.path.join(bringup_share, 'control_plan.txt')
    use_plot2 = LaunchConfiguration('use_plot2').perform(context).lower() in (
        '1', 'true', 'yes')
    def _opt_bool(name):
        raw = LaunchConfiguration(name).perform(context).strip().lower()
        if raw == '':
            return None
        if raw in ('1', 'true', 'yes'):
            return True
        if raw in ('0', 'false', 'no'):
            return False
        raise RuntimeError(f'{name} must be true|false (got {raw!r})')

    use_globalslam = LaunchConfiguration('use_globalslam').perform(context).lower() in (
        '1', 'true', 'yes')
    global_corrections_raw = LaunchConfiguration('global_corrections').perform(
        context).strip().lower()
    if global_corrections_raw == '':
        global_corrections = None
    elif global_corrections_raw in ('1', 'true', 'yes'):
        global_corrections = True
    elif global_corrections_raw in ('0', 'false', 'no'):
        global_corrections = False
    else:
        raise RuntimeError(
            f'global_corrections must be true|false|empty (got {global_corrections_raw!r})')
    enable_ba = _opt_bool('enable_global_ba')
    enable_tag = _opt_bool('enable_tag_loop_closure')
    enable_vis = _opt_bool('enable_visual_loop_closure')
    apply_ba = _opt_bool('apply_ba_corrections')
    apply_tag = _opt_bool('apply_tag_loop_corrections')
    apply_vis = _opt_bool('apply_visual_loop_corrections')
    slam_feedback = LaunchConfiguration('slam_feedback').perform(context)
    slam_iface = LaunchConfiguration('slam_feedback_interface').perform(context)

    cli_wants_feedback = (
        global_corrections is True or
        apply_ba is True or apply_tag is True or apply_vis is True)
    if cli_wants_feedback and not use_globalslam:
        raise RuntimeError(
            'Local feedback / global_corrections requires use_globalslam:=true '
            '(./launch2.sh --globalslam true ...).')

    range_sensor = LaunchConfiguration('range_sensor').perform(context).strip().lower()
    if range_sensor not in ('sonar', 'tfa300l'):
        raise RuntimeError(
            f"range_sensor must be 'sonar' or 'tfa300l' (got {range_sensor!r})")

    with open(config_file, 'r') as file:
        raw = yaml.safe_load(file)

    lslam_params = dict(raw['/lslam']['ros__parameters'])
    webotbridge_params = dict(raw['/webotbridge']['ros__parameters'])
    control_params = resolve_slam_feedback(
        dict(raw['/control']['ros__parameters']),
        slam_feedback=slam_feedback if slam_feedback.strip() != '' else None,
        slam_feedback_interface=slam_iface if slam_iface.strip() != '' else None,
    )

    # B3: optional /lslam-only overlay (gates); never touches velocity/Σ_a/uv.
    range_overlay_path = ''
    if range_sensor == 'tfa300l':
        candidates = [
            os.path.join(bringup_share, 'config', 'range_sensor_tfa300l.yaml'),
            os.path.join(os.getcwd(), 'config', 'range_sensor_tfa300l.yaml'),
        ]
        for cand in candidates:
            if os.path.isfile(cand):
                range_overlay_path = os.path.abspath(cand)
                break
        if not range_overlay_path:
            raise RuntimeError(
                'range_sensor=tfa300l overlay not found '
                '(install ekf_monoslam_bringup or run from repo root)')
        with open(range_overlay_path, 'r') as of:
            overlay_raw = yaml.safe_load(of) or {}
        overlay_lslam = dict(
            overlay_raw.get('/lslam', {}).get('ros__parameters', {}) or {})
        forbidden = {
            'Speed_update', 'Visual_update_min_vel_for_update_feats',
            'Sigma_a', 'Sigma_cam_uv'}
        clash = forbidden.intersection(overlay_lslam.keys())
        if clash:
            raise RuntimeError(
                'range_sensor overlay must not set velocity/noise keys: '
                + ', '.join(sorted(clash)))
        lslam_params.update(overlay_lslam)

    plan_override = LaunchConfiguration('control_plan').perform(context)
    if plan_override:
        control_params['Control_plan_path'] = plan_override
    plan_param = control_params.get('Control_plan_path', '')
    if (not plan_param) or (not os.path.isabs(str(plan_param))) or (
            not os.path.isfile(str(plan_param))):
        control_params['Control_plan_path'] = default_plan

    for p in (lslam_params, webotbridge_params, control_params):
        p['use_sim_time'] = True

    calibration_source = LaunchConfiguration('calibration_source').perform(context)
    if calibration_source:
        lslam_params['Calibration_source'] = calibration_source

    downward_range_topic = (
        '/laser_range' if range_sensor == 'tfa300l' else '/downward_range')

    lslam_remaps = [
        ('~/image', '/image'),
        ('~/camera_info', '/camera_info'),
        ('~/imu', '/imu_out'),
        ('~/body_velocity', '/body_velocity'),
        ('~/downward_range', downward_range_topic),
        ('~/altitude', '/altitude'),
        ('~/odometry', '/odometry'),
        ('~/odometry_meta', '/EKFslam/odometry_meta'),
        ('~/correction_applied', '/EKFslam/correction_applied'),
        ('~/landmarks', '/EKFslam/landmarks'),
        ('~/path', '/EKFslam/path'),
        ('~/estimator_reference', '/EKFslam/estimator_reference'),
    ]
    webotbridge_remaps = [
        ('camera', '/camera/image_color'),
        ('camera_info', '/camera_info'),
        ('imu', '/imu'),
        ('ds0', '/ds0'),
        ('ds_tfa300l', '/ds_tfa300l'),
        ('EKFslam/estimator_reference', '/EKFslam/estimator_reference'),
        ('ground_truth/path', '/ground_truth/path'),
    ]
    control_remaps = [
        ('odometry', '/odometry'),
        ('odometry_meta', '/EKFslam/odometry_meta'),
        ('correction_applied', '/EKFslam/correction_applied'),
        ('correction_feedback_accepted', '/control/correction_feedback_accepted'),
        ('global_loop_committed', '/gslam/global_loop_committed'),
    ]
    plot_remaps = [
        ('odometry', '/odometry'),
        ('landmarks', '/EKFslam/landmarks'),
        ('path', '/EKFslam/path'),
        ('global_keyframes', '/gslam/global_keyframes'),
        ('global_landmarks', '/gslam/global_landmarks'),
    ]

    nodes = [
        ComposableNode(
            package='localslam',
            plugin='localslam::EKFslam',
            name='EKFslam',
            parameters=[lslam_params],
            remappings=lslam_remaps),
        ComposableNode(
            package='webotbridge',
            plugin='webotbridge::WEBOTb',
            name='WEBOTb',
            parameters=[webotbridge_params],
            remappings=webotbridge_remaps),
        ComposableNode(
            package='control',
            plugin='control::ControlQuad',
            name='ControlQuad',
            parameters=[control_params],
            remappings=control_remaps),
    ]

    extra_actions = [
        LogInfo(msg=[
            f'B3 range_sensor={range_sensor}; LocalSLAM ~/downward_range → '
            f'{downward_range_topic}'
            + (f'; overlay={range_overlay_path})' if range_overlay_path else ''),
        ]),
    ]

    if use_globalslam:
        try:
            get_package_share_directory('globalslam')
        except PackageNotFoundError as exc:
            raise RuntimeError(
                'use_globalslam:=true but globalslam is not installed in this overlay. '
                'Build with GTSAM on CMAKE_PREFIX_PATH (see tools/gtsam_prefix.env.example '
                'and docs/history/G1_globalslam_interface_contract.md). '
                'Do not install system linker workarounds from this launcher.'
            ) from exc

        gslam_yaml = dict(raw.get('/gslam', {}).get('ros__parameters', {}))
        resolved = resolve_gslam_g6d_params(
            gslam_yaml,
            enable_ba=enable_ba,
            enable_tag=enable_tag,
            enable_vis=enable_vis,
            apply_ba=apply_ba,
            apply_tag=apply_tag,
            apply_vis=apply_vis,
            global_corrections=global_corrections,
        )
        needs_coordination = bool(resolved.pop('_needs_coordination'))
        gslam_params = dict(gslam_yaml)
        gslam_params.update(resolved)
        gslam_params['use_sim_time'] = True
        # Integrated bringup: avoid a permanently idle GS component. Operator
        # can still call /gslam/enable false. Standalone YAML default remains
        # false (this is an intentional integrated override, not a feedback flag).
        gslam_params['Processing_start_enabled'] = True
        _assert_globalslam_calibration(lslam_params, gslam_params)

        if needs_coordination:
            gslam_params.setdefault('Coordinate_feedback_barrier_timeout_ms', 5000)
            gslam_params.setdefault('Coordinate_pause_timeout_ms', 3000)

        eff_ba = bool(gslam_params['Enable_global_ba'])
        eff_tag = bool(gslam_params['Enable_tag_loop_closure'])
        eff_vis = bool(gslam_params['Enable_visual_loop_closure'])
        fb_effective = control_params.get('Slam_feedback', False)
        fb_iface = control_params.get('Slam_feedback_interface', 'odometry')
        fb_mode = (
            f'odometry ({fb_iface})' if fb_effective else 'gt (Webots GPS)')
        matrix = (
            f'ba={int(eff_ba)}/fb={int(bool(gslam_params.get("Apply_ba_corrections")))} '
            f'tag={int(eff_tag)}/fb={int(bool(gslam_params.get("Apply_tag_loop_corrections")))} '
            f'vis={int(eff_vis)}/fb={int(bool(gslam_params.get("Apply_visual_loop_corrections")))}')
        any_apply = needs_coordination

        gslam_remaps = [
            ('Kf_topic', 'Kf_topic'),
            ('Kf_cl_topic', 'Kf_cl_topic'),
            ('~/global_keyframes', '/gslam/global_keyframes'),
            ('~/global_landmarks', '/gslam/global_landmarks'),
            ('~/global_loop_committed', '/gslam/global_loop_committed'),
            ('correction_feedback_accepted', '/control/correction_feedback_accepted'),
        ]
        nodes.append(ComposableNode(
            package='globalslam',
            plugin='globalslam::Gslam',
            name='gslam',
            parameters=[gslam_params],
            remappings=gslam_remaps))
        if any_apply:
            extra_actions.append(LogInfo(msg=[
                'G6d: coordinated Local feedback ENABLED. '
                f'matrix [{matrix}] Coordinate_mode=required nav_feedback={fb_mode}. '
                'Pause≠hover. No uncoordinated fallback. Topics: /gslam/global_*, '
                '/gslam/global_loop_committed, /EKFslam/correction_applied, '
                '/control/correction_feedback_accepted.'
            ]))
        else:
            extra_actions.append(LogInfo(msg=[
                f'G6d: Global SLAM enabled (Local feedback OFF; matrix [{matrix}]; '
                'Processing_start_enabled=true). Topics: /gslam/global_keyframes, '
                '/gslam/global_landmarks, /gslam/global_loop_committed. Disable: '
                'ros2 service call /gslam/enable std_srvs/srv/SetBool "{data: false}"'
            ]))

    if use_plot2:
        try:
            get_package_share_directory('plot2')
            plot_params = dict(raw.get('/plot2', {}).get('ros__parameters', {}))
            plot_params['use_sim_time'] = True
            plot_params['Subscribe_global_outputs'] = use_globalslam
            nodes.insert(1, ComposableNode(
                package='plot2',
                plugin='plot2::PLOTscene2',
                name='PLOTscene2',
                parameters=[plot_params],
                remappings=plot_remaps))
        except PackageNotFoundError:
            raise RuntimeError(
                'use_plot2:=true but plot2 is not installed in this overlay')

    container = ComposableNodeContainer(
        name='quad_experiment_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=nodes,
        output='screen',
    )
    return extra_actions + [container]


def generate_launch_description():
    return launch.LaunchDescription([
        DeclareLaunchArgument('use_plot2', default_value='true'),
        DeclareLaunchArgument(
            'use_globalslam',
            default_value='false',
            description='Opt-in Global SLAM (G4). Default false = local-only.'),
        DeclareLaunchArgument(
            'global_corrections',
            default_value='',
            description=(
                'G6d compatibility master: empty=leave YAML; true=force Apply_* '
                'for enabled sources; false=clear Apply_global_corrections only. '
                'Requires use_globalslam:=true when true. Explicit apply_*=false '
                'with true is rejected. Does not enable visual processing.')),
        DeclareLaunchArgument('enable_global_ba', default_value=''),
        DeclareLaunchArgument('enable_tag_loop_closure', default_value=''),
        DeclareLaunchArgument('enable_visual_loop_closure', default_value=''),
        DeclareLaunchArgument('apply_ba_corrections', default_value=''),
        DeclareLaunchArgument('apply_tag_loop_corrections', default_value=''),
        DeclareLaunchArgument('apply_visual_loop_corrections', default_value=''),
        DeclareLaunchArgument(
            'slam_feedback',
            default_value='',
            description='Override Slam_feedback (true/false). Empty = YAML default.'),
        DeclareLaunchArgument(
            'slam_feedback_interface',
            default_value='',
            description='Override interface (odometry|legacy). Empty = YAML default.'),
        DeclareLaunchArgument(
            'calibration_source',
            default_value='',
            description="Override Calibration_source (yaml|camera_info). Empty = YAML default."),
        DeclareLaunchArgument(
            'control_plan',
            default_value='',
            description='Absolute path to control plan. Empty = installed default / YAML.'),
        DeclareLaunchArgument(
            'params_file',
            default_value='',
            description=(
                'Absolute path to experiment YAML (e.g. quad_params_no_velocity.yaml). '
                'Empty = installed config/quad_params.yaml.')),
        DeclareLaunchArgument(
            'range_sensor',
            default_value='sonar',
            description=(
                "B3: 'sonar' → /downward_range (ds0); "
                "'tfa300l' → /laser_range (ds_tfa300l) + range gate overlay.")),
        OpaqueFunction(function=_build),
    ])
