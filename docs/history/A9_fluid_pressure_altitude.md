# A9 — Optional FluidPressure altitude input

`Altitude_input_mode` selects exactly one altitude source. Default remains
`derived_altitude` (mavros / Webots GPS-height). No weather service, bias
estimation, or fusion of pressure with derived altitude.

## Conversion model

**Primary sources:** ICAO Standard Atmosphere (ISA) troposphere / ICAO Doc 7488;
equivalent US Standard Atmosphere 1976 lower layer; NCAR pressure-altitude notes.

Absolute ISA geopotential height from absolute static pressure \(P\) [Pa]:

\[
h_\mathrm{isa}(P)=\frac{T_0}{L}\left(1-\left(\frac{P}{P_0}\right)^{R_\mathrm{spec} L/g}\right)
\]

| Symbol | Value | Meaning |
|---|---|---|
| \(P_0\) | 101325 Pa | Sea-level standard pressure |
| \(T_0\) | 288.15 K (param `Pressure_t0_K`) | Sea-level standard temperature |
| \(L\) | 0.0065 K/m | Tropospheric lapse magnitude |
| \(g\) | 9.80665 m/s² | Standard gravity |
| \(R_\mathrm{spec}\) | 287.05287 J/(kg·K) | Dry-air specific gas constant |

**Assumptions:** dry ideal gas, hydrostatic equilibrium, constant lapse, \(g\) and
\(R\) altitude-independent. **Supported band:** absolute ISA height roughly
−500…11000 m (`Pressure_min_Pa` / `Pressure_max_Pa`). No live temperature or
humidity correction.

### Relative altitude (ROS boundary)

Policy `Pressure_reference_policy=first_valid`:

1. Latch \(P_\mathrm{ref}\) on the first valid `FluidPressure` after startup/reset.
2. \(h_\mathrm{rel}=h_\mathrm{isa}(P)-h_\mathrm{isa}(P_\mathrm{ref})\) [m, **+up**].
3. Subtract optional `Pressure_z_lever_arm_m` (+up body-origin → sensor).

Feed \(h_\mathrm{rel}\) into the **existing** altitude observation path
(`data_type=alt`). Acquisition stamp preserved (`header.stamp` → ms).

### Double-offset avoidance

`EKF::Altitude_update` still latches the first internal altitude sample as home.
Because \(h_\mathrm{rel}\approx 0\) on the reference sample, that EKF latch is a
no-op (home ≈ 0). Do **not** also feed absolute ISA height — that would
double-offset against the EKF latch.

## Exclusive modes

| `Altitude_input_mode` | Subscription | Not subscribed |
|---|---|---|
| `derived_altitude` (default) | `~/altitude` (`mavros_msgs/Altitude`) | `~/pressure` |
| `fluid_pressure` | `~/pressure` (`sensor_msgs/FluidPressure`) | `~/altitude` |

Requires `Sensor_input_mode=standard`. No automatic fallback.

## Noise contract

- Authoritative: `Sigma_alt_update` **[m]** (existing altitude update).
- **Ignored:** `FluidPressure.variance` (Pa² ≠ m²). Never mapped to altitude noise.

## Frame / mounting

Default `Pressure_z_lever_arm_m=0`: pressure is treated as measured at the same
altitude reference point as `Altitude.relative` / the altitude state (body
origin). Nonzero lever arm moves the observation to body origin explicitly.
Optional `Pressure_expected_frame_id` rejects mismatched `header.frame_id`.

## Reset

`~/reset` clears the pressure reference, clears held rejects, and drops buffered
`alt` observations so a pre-reset sample cannot become the new reference.
**Limitation:** `Altitude_update`’s process-static home latch is **not** cleared
(pre-existing; not redesigned in A9).

## CameraInfo QoS (separate from pressure)

`Camera_info_qos_reliability`: `reliable` (default) or `best_effort`.
Durability remains **volatile** (not transient-local). Publishers must match
reliability; e.g. best-effort camera drivers need `best_effort` on the
subscription.

## Real barometer connection (example)

```bash
source install_a9/setup.bash
ros2 launch localslam localslam.launch.py \
  sensor_input_mode:=standard \
  altitude_input_mode:=fluid_pressure \
  pressure_topic:=/baro/fluid_pressure \
  pressure_expected_frame_id:=baro_link \
  pressure_z_lever_arm_m:=0.0 \
  sigma_alt_update:=0.5
```

Publish absolute pressure in **pascals** on that topic. Do not publish mavros
Altitude in the same mode (it is not subscribed).

## A6 experiment

Unchanged: `Altitude_input_mode: derived_altitude`. Webots does **not**
synthesize FluidPressure.
