# ground_vio

Monocular visual-inertial odometry for the RC car: the fixed camera and the ISM330DHCX IMU
are fused in a multi-state constraint Kalman filter (MSCKF). It runs on the laptop, reads
`/imu/data_raw` and the fixed camera image from the Raspberry Pi, and publishes
`nav_msgs/Odometry`.

**Status:** the filter and the feature tracker are tested on synthetic data (see
[Tests](#tests)). The ROS 2 node is written but has not been built or run yet, and nothing
has been tested on the car. Treat the first drives as an experiment, not as a result.

## How it works

- **State:** position, velocity and orientation of the IMU, gyroscope and accelerometer
  bias, and (optionally) the camera-to-IMU pose. A sliding window of the last 11 camera
  poses is kept in the state.
- **IMU:** every sample propagates the state. Image times are reached by interpolating the
  IMU between two samples, so an image stamp can fall between IMU stamps.
- **Camera:** KLT tracks (OpenCV) follow corner features. When a feature is lost, or its
  oldest camera pose leaves the window, it is triangulated from the window and its
  reprojection error updates the filter. The feature position itself is removed from the
  problem (null-space projection), so the map is not part of the state.
- **Gating:** a chi-square test drops tracks that do not fit, which handles mismatched
  tracks. Tracks without enough parallax are not used.
- **Start:** the car has to stand still for about one second. The gravity direction gives
  roll and pitch; the heading and the position at that moment become the world frame
  (x forward at start, z up).
- **Time:** only the Pi stamps are used. The camera stamps can be moved onto the IMU time
  axis with `camera_time_offset_s` once the camera-to-IMU offset has been measured.

Optional constraints for a car on a flat floor (both off by default):

- `planar_z_sigma_m` keeps the height at its start value.
- `zupt.enabled` sets the velocity to zero when the IMU and the image both show that the
  car stands still. The IMU alone cannot tell standing still from constant speed, so the
  tracked features must stand still as well.

Monocular VIO can see the scale only through acceleration. A car driving straight at
constant speed on a flat floor gives the scale little to work with, so expect the scale
and the position to drift more than in the simulation, which includes turns and speed
changes.

## Before the first run

1. **Camera intrinsics (required).** The node refuses to start while `camera.calibrated`
   is `false`.

   ```bash
   sudo apt install python3-opencv
   python3 scripts/calibrate_intrinsics.py --images ~/calib --cols 8 --rows 6 --square 0.025
   ```

   Print a checkerboard, take 20 to 40 pictures with the fixed camera at 640 x 480, and
   tilt and move the board so that it reaches the corners of the image. Copy the printed
   block into `config/ground_vio.yaml`. A reprojection error below about 0.5 px is good.
   The script was checked on rendered boards with a known lens, where it found the focal
   lengths within 0.2 % and the centre within 0.5 px.

2. **IMU axes and camera mounting.** `extrinsic.imu_R_cam` maps camera axes into IMU axes.
   The default assumes the IMU has x forward, y left, z up, and the camera looks forward.
   Check it with the car standing still:

   ```bash
   ros2 topic echo /imu/data_raw --field linear_acceleration
   ```

   The axis pointing up must read about +9.8. Then tilt the nose up and look at the
   angular velocity (`--field angular_velocity`): nose up must give a negative y rate for
   a y axis pointing left. If the sensor is mounted differently, change `imu_R_cam` so that
   it is a proper rotation (determinant +1). The filter corrects small errors (a few
   degrees) when `extrinsic.estimate` is true, but it cannot repair swapped or flipped axes.
   Measure `imu_p_cam` (camera position in the IMU frame, meters) with a ruler.

3. **Time offset.** Leave `camera_time_offset_s` at 0 until the offset has been measured.

## Run

Build on the laptop together with the other packages, then:

```bash
ros2 launch ground_vio ground_vio.launch.py
```

Start the Pi publishers first (`ros2 launch sensor_publisher sensors.launch.py`), keep the
car still until the log says `Initialized`, then drive. The node prints a status line every
5 seconds.

| Topic | Type | Direction |
|---|---|---|
| `/imu/data_raw` | `sensor_msgs/Imu` | in |
| `/camera/fixed/image_raw/compressed` | `sensor_msgs/CompressedImage` | in (`image_topic`, `image_compressed: false` for a raw `Image`) |
| `/ground_vio/odometry` | `nav_msgs/Odometry` | out |

The odometry pose is the IMU pose in the world frame; the twist is in the IMU frame. To feed
`driftmodel_env`, remap the output to the odometry topic it expects
(`/loop_fusion_node/odometry_rect` in the current launch).

## Tuning

- **Rejected by gate** in the status line is high (more than about 30 %): raise
  `pixel_sigma_px` or `chi2_scale`, or check the intrinsics.
- **Rejected by geometry** is high: normal for distant features and slow motion. Raise
  `max_depth_m` only if the scene is large.
- **Drift while standing still:** enable `zupt.enabled`.
- **Height drifts on a flat floor:** set `planar_z_sigma_m` to 0.05.
- IMU noise values are larger than the data sheet values on purpose; real vibration on the
  car is not white noise.

## Tests

The filter and the tracker do not depend on ROS and can be built and tested alone:

```bash
cmake -S . -B build -DGROUND_VIO_STANDALONE=ON
cmake --build build -j
ctest --test-dir build --output-on-failure
```

Requirements: Eigen 3 and OpenCV (`sudo apt install libeigen3-dev libopencv-dev`).

| Test | What it checks |
|---|---|
| `test_jacobians` | Analytic camera Jacobians against numerical derivatives |
| `test_filter_sim` | A 40 s synthetic drive (30 m, turns, 1.5 % wrong tracks): position error with known and with wrong extrinsics, the flat-floor constraints, and that an image arriving before its IMU data gives the same result |
| `test_tracker` | Track ids and motion on a synthetic moving texture |

In the simulation (200 Hz IMU, 30 Hz camera, 0.5 px image noise) the position error after
30 m was 0.15 m on average over eight noise seeds for 3D motion, and 0.3 m for a flat
drive with the constraints on. A wrong camera rotation of 2.5 degrees was corrected to
0.03 degrees. The simulation has ideal tracks, a perfect camera model and known noise, so
real results will be worse.
