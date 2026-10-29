## Camera subsystem

The camera test launch uses the existing assignment world and starts the
camera, optical TF frame, and ArUco detector without Nav2 or SLAM:

```bash
pkill -9 -f 'Sensors-and-Control-Integrated-Project' 2>/dev/null || true
pkill -9 -f 'gzserver' 2>/dev/null || true
pkill -9 -f 'gzclient' 2>/dev/null || true
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch Sensors-and-Control-Integrated-Project camera_test.launch.py
```

Camera inputs:

- `/camera/camera_sensor/image_raw`
- `/camera/camera_sensor/camera_info`
- TF: `base_link -> camera_link_optical`

Detector outputs:

- `/camera/aruco_debug`
- `/camera/aruco_pose` (`PoseWithCovarianceStamped`)

The pose output is the charging-station marker pose expressed in `base_link`.
The marker ID is `0`, its physical side is `0.80 m`, and the detector uses
the `DICT_4X4_50` dictionary.
