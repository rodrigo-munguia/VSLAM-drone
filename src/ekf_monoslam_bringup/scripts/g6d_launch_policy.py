#!/usr/bin/env python3
"""G6d integrated-launch parameter precedence (pure, unit-testable).

Precedence: explicit CLI/launch override > YAML > node declare_parameter default.

Omitted overrides must not rewrite YAML. The legacy global_corrections master is
handled separately: true forces Apply_* for enabled sources; explicit
apply_*=false while master is true is a hard error.
"""

from __future__ import annotations

from typing import Any, Mapping, MutableMapping, Optional


def _as_bool(value: Any, default: bool) -> bool:
    if value is None:
        return default
    if isinstance(value, bool):
        return value
    s = str(value).strip().lower()
    if s in ('1', 'true', 'yes'):
        return True
    if s in ('0', 'false', 'no'):
        return False
    return default


def resolve_gslam_g6d_params(
    yaml_gslam: Mapping[str, Any],
    *,
    enable_ba: Optional[bool] = None,
    enable_tag: Optional[bool] = None,
    enable_vis: Optional[bool] = None,
    apply_ba: Optional[bool] = None,
    apply_tag: Optional[bool] = None,
    apply_vis: Optional[bool] = None,
    global_corrections: Optional[bool] = None,
    node_defaults: Optional[Mapping[str, bool]] = None,
) -> dict[str, Any]:
    """Merge YAML + optional CLI into final /gslam G6d fields.

    Args:
      yaml_gslam: /gslam ros__parameters dict (may be empty).
      enable_*/apply_*: None = omitted (keep YAML).
      global_corrections: None = omitted; True = master force; False = clear
        Apply_global_corrections master flag only (does not wipe per-source
        Apply_* from YAML/CLI).
      node_defaults: declare_parameter defaults used when YAML key absent.

    Returns:
      Dict of resolved G6d keys plus helper `_needs_coordination` (bool).
    Raises:
      RuntimeError on invalid combinations / master conflicts.
    """
    nd = {
        'Enable_global_ba': True,
        'Enable_tag_loop_closure': True,
        'Enable_visual_loop_closure': False,
        'Apply_ba_corrections': False,
        'Apply_tag_loop_corrections': False,
        'Apply_visual_loop_corrections': False,
        'Apply_global_corrections': False,
        'Coordinate_mode': 'required',
    }
    if node_defaults:
        nd.update(node_defaults)

    out: MutableMapping[str, Any] = dict(yaml_gslam)

    # Start from YAML, falling back to node defaults for missing keys.
    out['Enable_global_ba'] = _as_bool(
        out.get('Enable_global_ba'), nd['Enable_global_ba'])
    out['Enable_tag_loop_closure'] = _as_bool(
        out.get('Enable_tag_loop_closure'), nd['Enable_tag_loop_closure'])
    out['Enable_visual_loop_closure'] = _as_bool(
        out.get('Enable_visual_loop_closure'), nd['Enable_visual_loop_closure'])
    out['Apply_ba_corrections'] = _as_bool(
        out.get('Apply_ba_corrections'), nd['Apply_ba_corrections'])
    out['Apply_tag_loop_corrections'] = _as_bool(
        out.get('Apply_tag_loop_corrections'), nd['Apply_tag_loop_corrections'])
    out['Apply_visual_loop_corrections'] = _as_bool(
        out.get('Apply_visual_loop_corrections'),
        nd['Apply_visual_loop_corrections'])
    out['Apply_global_corrections'] = _as_bool(
        out.get('Apply_global_corrections'), nd['Apply_global_corrections'])

    # Explicit CLI overrides (None = leave YAML/node).
    if enable_ba is not None:
        out['Enable_global_ba'] = enable_ba
    if enable_tag is not None:
        out['Enable_tag_loop_closure'] = enable_tag
    if enable_vis is not None:
        out['Enable_visual_loop_closure'] = enable_vis
    if apply_ba is not None:
        out['Apply_ba_corrections'] = apply_ba
    if apply_tag is not None:
        out['Apply_tag_loop_corrections'] = apply_tag
    if apply_vis is not None:
        out['Apply_visual_loop_corrections'] = apply_vis

    if out['Enable_tag_loop_closure'] and out['Enable_visual_loop_closure']:
        raise RuntimeError(
            'Enable_tag_loop_closure and Enable_visual_loop_closure cannot '
            'both be true (no simultaneous loop-source arbitration).')

    # Master: omitted → leave YAML Apply_global_corrections; false → clear master
    # only; true → force Apply_* for enabled sources.
    if global_corrections is False:
        out['Apply_global_corrections'] = False
    elif global_corrections is True:
        # Explicit per-source apply=false conflicts with master force.
        conflicts = []
        if out['Enable_global_ba']:
            if apply_ba is False:
                conflicts.append(
                    '--apply-ba-corrections false conflicts with '
                    '--global-corrections true')
            out['Apply_ba_corrections'] = True
        if out['Enable_tag_loop_closure']:
            if apply_tag is False:
                conflicts.append(
                    '--apply-tag-loop-corrections false conflicts with '
                    '--global-corrections true')
            out['Apply_tag_loop_corrections'] = True
        if out['Enable_visual_loop_closure']:
            if apply_vis is False:
                conflicts.append(
                    '--apply-visual-loop-corrections false conflicts with '
                    '--global-corrections true')
            out['Apply_visual_loop_corrections'] = True
        if conflicts:
            raise RuntimeError(
                'Contradictory Global SLAM correction options: ' +
                '; '.join(conflicts) +
                '. Omit the per-source flag or set it true, or drop the master.')
        out['Apply_global_corrections'] = True
    # else None: keep YAML Apply_global_corrections; node apply_master_compatibility
    # will expand YAML master if set.

    # YAML master still expands Apply_* for enabled sources (node does this too;
    # mirror here so coordination detection matches the node).
    if out['Apply_global_corrections']:
        if out['Enable_global_ba']:
            out['Apply_ba_corrections'] = True
        if out['Enable_tag_loop_closure']:
            out['Apply_tag_loop_corrections'] = True
        if out['Enable_visual_loop_closure']:
            out['Apply_visual_loop_corrections'] = True

    if out['Apply_ba_corrections'] and not out['Enable_global_ba']:
        raise RuntimeError('Apply_ba_corrections requires Enable_global_ba')
    if out['Apply_tag_loop_corrections'] and not out['Enable_tag_loop_closure']:
        raise RuntimeError(
            'Apply_tag_loop_corrections requires Enable_tag_loop_closure')
    if out['Apply_visual_loop_corrections'] and not out['Enable_visual_loop_closure']:
        raise RuntimeError(
            'Apply_visual_loop_corrections requires Enable_visual_loop_closure')

    needs_coordination = bool(
        out['Apply_ba_corrections'] or
        out['Apply_tag_loop_corrections'] or
        out['Apply_visual_loop_corrections'] or
        out['Apply_global_corrections'])
    out['_needs_coordination'] = needs_coordination

    if needs_coordination:
        mode = str(out.get('Coordinate_mode', nd['Coordinate_mode'])).lower()
        if mode in ('absent_ok', 'offline_bypass'):
            raise RuntimeError(
                'G6d integrated Local feedback rejects '
                f'Coordinate_mode={mode!r}. Use Coordinate_mode=required '
                '(forced). Do not use absent_ok/offline_bypass in '
                'integrated vehicle-connected mode.')
        out['Coordinate_mode'] = 'required'

    return dict(out)


def resolve_slam_feedback(
    yaml_control: Mapping[str, Any],
    *,
    slam_feedback: Optional[str] = None,
    slam_feedback_interface: Optional[str] = None,
    node_defaults: Optional[Mapping[str, Any]] = None,
) -> dict[str, Any]:
    """CLI > YAML > node default for Control Slam_feedback fields."""
    nd = {'Slam_feedback': False, 'Slam_feedback_interface': 'odometry'}
    if node_defaults:
        nd.update(node_defaults)
    out = dict(yaml_control)
    out['Slam_feedback'] = _as_bool(
        out.get('Slam_feedback'), bool(nd['Slam_feedback']))
    iface = out.get('Slam_feedback_interface', nd['Slam_feedback_interface'])
    out['Slam_feedback_interface'] = str(iface) if iface is not None else nd[
        'Slam_feedback_interface']

    if slam_feedback is not None and str(slam_feedback).strip() != '':
        out['Slam_feedback'] = _as_bool(slam_feedback, out['Slam_feedback'])
    if slam_feedback_interface is not None and str(
            slam_feedback_interface).strip() != '':
        out['Slam_feedback_interface'] = str(slam_feedback_interface).strip()
    return out
