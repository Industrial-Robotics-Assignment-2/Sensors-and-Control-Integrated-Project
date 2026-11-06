# Sensors and Control Integrated Project

## Project information

| Item | Details |
|---|---|
| Track | A - TurtleBot3 Burger visual waypoint navigation |
| Group number | **42** |
| Team members | **Cody Frank  25318179     Alex Vella  25309372     Liam Dascalos  25455965** |
| GitHub repository | [Industrial-Robotics-Assignment-2/Sensors-and-Control-Integrated-Project](https://github.com/Industrial-Robotics-Assignment-2/Sensors-and-Control-Integrated-Project) |
| Demo video | **https://youtu.be/223eHUPFVhE** |

## System overview

The robot navigates a simulated environment using wheel odometry, 2D LiDAR,
and a camera. It follows predefined waypoints, avoids obstacles detected by
LiDAR, and visually docks at a charging station marked with an ArUco tag.
The controller uses the estimated pose from the estimator; Gazebo ground truth
is used only for evaluation.

## Frame diagram

```text
                         evaluation only
                  +--------------------------+
                  | world                    |
                  |   -> start_frame         |
                  |       -> base_footprint  |
                  |           -> base_link   |
                  |               -> laser   |
                  |               -> camera_link
                  |                   -> camera_link_optical
                  +--------------------------+

wheel encoders -> /odom -> dead-reckoning prediction
LiDAR /scan ------------------------------+
                                             v
                                      +-------------+
                                      | EKF + ICP   |
                                      +-------------+
                                             |
                                             v
                              /ekf_estimated_pose_X
                                             |
                                             v
                                      +-------------+
                                      | LQR / APF  |
                                      +-------------+
                                             |
                                             v
                                         /cmd_vel

Gazebo ground truth -> /ground_truth/pose
                    -> start-frame evaluator
                    -> /pose_comparison
```

`start_frame` is defined by the robot's first ground-truth pose. Consequently,
the robot starts at approximately `(x, y, yaw) = (0, 0, 0)` in this frame.

## Estimator design

The system uses an Extended Kalmann Filter using LiDAR Iterative Closest Point

The estimator state is:

```text
X = [x, y, theta]^T
```

### Prediction

Wheel odometry supplies forward velocity `v` and angular velocity `w`.
Forward Euler integration is used:

```text
x[k+1]     = x[k] + v * dt * cos(theta[k])
y[k+1]     = y[k] + v * dt * sin(theta[k])
theta[k+1] = wrap(theta[k] + w * dt)
```

The covariance is propagated using the state-transition Jacobian:

```text
P[k+1] = F P[k] F^T + Q
```

### LiDAR scan matching and correction

Each LiDAR scan is converted into 2D Cartesian points. Open3D point-to-point
ICP estimates the relative scan transform. The odometry increment provides
the initial ICP guess. The ICP increment and odometry increment are fused with
an identity observation model:

```text
y = z_icp - z_odom
S = H P H^T + R
K = P H^T S^-1
```

The correction is rotated from the robot frame into the start/world estimate
frame before updating the state. The covariance update uses the Joseph form.

### Verified estimator parameters

| Parameter | Value |
|---|---:|
| Initial pose | `[0, 0, 0]` |
| Initial position standard deviation | `0.30 m` |
| Initial heading standard deviation | `10 deg` |
| Position process-noise standard deviation | `0.02 m/sample` |
| Heading process-noise standard deviation | `4 deg/sample` |
| Measurement covariance | `R = Q` |
| ICP maximum correspondence distance | `0.20 m` |
| ICP maximum iterations | `30` |
| LiDAR input | `/scan` |
| Wheel odometry input | `/odom` |
| EKF output | `/ekf_estimated_pose_X` |

The estimator also publishes `/ekf_estimated_pose_X_covariance`. Its position
variances are covariance entries `0` (`x`) and `7` (`y`); their square roots
are the one-sigma position uncertainties.

## Controller design

Linear Quadratic Controller with Artificial Potential Field Obstacle Avoidance

The controller subscribes to `/ekf_estimated_pose_X` and publishes `/cmd_vel`.
It uses a discrete LQR trajectory tracker for waypoint following. LiDAR
obstacle data adds artificial-potential-field repulsion, and the controller
switches to camera-based alignment and docking when the ArUco tag is detected.

### Verified controller parameters

| Parameter | Value |
|---|---:|
| Reference linear velocity | `0.18 m/s` |
| Reference angular velocity | `0.20 rad/s` |
| Control period | `0.02 s` (`50 Hz`) |
| Maximum linear velocity | `0.22 m/s` |
| Maximum angular velocity | `1.00 rad/s` |
| Obstacle influence distance | `0.60 m` |
| LiDAR obstacle cone | `+/-60 deg` |
| Goal tolerance | `0.20 m` |
| LQR `Q` | `diag(2.0, 5.0, 1.2)` |
| LQR `R` | `diag(0.2, 0.2)` |
| Docking linear gain | `0.25` |
| Docking angular gain | `1.50` |
| Docking stop distance | `0.18 m` |

The camera detects ArUco marker ID `0`, using dictionary `DICT_4X4_50` and a
physical marker side length of `0.18 m`.

## Quantitative results

Ground truth is generated by Gazebo.
The evaluator also logs position error and one-sigma uncertainty once per
second. These values should be used instead of Gazebo ground truth in the
controller itself.

## Installation and running

```bash
sudo apt install ros-humble-gazebo-ros-pkgs ros-humble-turtlebot3-gazebo \
  ros-humble-turtlebot3-description

cd ~/Sensors-and-Control-Integrated-Project
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
ros2 launch Sensors-and-Control-Integrated-Project final.launch.py
```

Note: Some issues can arise depending on the version of opencv installed, due to
a function needed for ArUco detection being deprecated in the newest versions.

Useful monitoring commands:

```bash
ros2 topic echo /ekf_estimated_pose_X
```

For camera-only testing:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch Sensors-and-Control-Integrated-Project camera_test.launch.py
```

## Individual contributions

- **Liam Dascalos:** **Designed the lqr_controller and set up the LiDar, as well as working on testing and debugging most of the system**
- **[Member 2]:** **Designed the environment and obstacles within it, and worked on debugging**
- **Alex Vella:** **Implemented the camera and ArUco detection logic**

## Video of demonstration:
https://youtu.be/223eHUPFVhE

## Generative AI declaration

Generative AI tools were used as development aids for various aspects of the project.
This includes debugging of software and the database, as well as to assist in determining
correct logic for some aspects of the environment. Specific tools utilised were Gemini, Claude
and the in-agent Generative AI within VS-code, with these tools supporting human generated code.
All generated suggestions were reviewed, tested, and integrated by the project team. The
team remains responsible for the final implementation, analysis, and report.
