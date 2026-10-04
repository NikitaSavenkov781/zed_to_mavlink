# zed_to_mavlink

ROS2 workspace + package that subscribes to ZED odometry (`nav_msgs/msg/Odometry`) and:
- sends it to ArduPilot/PX4 as external vision via MAVSDK Mocap (`VisionPositionEstimate`)
- republishes pose as `geometry_msgs/PoseStamped` on `/uav1/local_position/pose`

## Build

```bash
cd ~/zed_to_mavlink
source /opt/ros/*/setup.bash
colcon build --symlink-install
```

## Run

```bash
cd ~/zed_to_mavlink
source install/setup.bash
ros2 run zed_to_mavlink zed_to_mavlink_node
```

## Parameters

- `connection_url` (string, default: `udpin://0.0.0.0:14590`)
- `pose_topic` (string, default: `/zed/zed_node/odom`)
- `local_pose_topic` (string, default: `/uav1/local_position/pose`)
- `publish_period_ms` (int, default: `100`) ~10 Hz to FC
# zed_to_mavlink
