
# ROS2 Differential Drive Controller (Odometry + TF)

This project implements a ROS 2-based differential drive robot controller that:
- Converts velocity commands (`/cmd_vel`) into wheel commands
- Computes odometry from wheel encoder feedback (`joint_states`)
- Publishes robot pose as `nav_msgs/Odometry`
- Broadcasts TF between `odom` and `base_footprint`

The implementation is based on a simple kinematic model of a differential drive robot.

---

## Features

- Velocity command to wheel velocity conversion
- Encoder-based odometry estimation
- Real-time pose integration (x, y, theta)
- TF broadcasting for visualization in RViz
- Compatible with Gazebo simulation

---

## ROS Interfaces

### Subscribed Topics

- `/cmd_vel` (`geometry_msgs/TwistStamped`)
  - Provides linear and angular velocity commands

- `joint_states` (`sensor_msgs/JointState`)
  - Provides left and right wheel encoder positions

---

### Published Topics

- `robot_controller/odom` (`nav_msgs/Odometry`)
  - Estimated robot pose and velocity

- `simple_velocity_controller/commands` (`std_msgs/Float64MultiArray`)
  - Wheel velocity commands

---

### TF Frames

- `odom → base_footprint`

---

## System Logic (Based on Code)

### 1. Velocity to Wheel Conversion

Incoming `/cmd_vel` is converted into left and right wheel velocities using the robot kinematic model.

---

### 2. Encoder Processing

From `joint_states`, the controller computes:
- Change in wheel positions
- Wheel angular velocities

These values are used for motion estimation.

---

### 3. Odometry Estimation

The robot pose is updated incrementally using wheel displacement:
- Linear displacement from both wheels
- Angular change from difference in wheel motion

Pose is integrated over time to estimate:
- Position (x, y)
- Orientation (theta)

---

### 4. TF Broadcasting

The computed pose is published as a TF transform:
- Parent frame: `odom`
- Child frame: `base_footprint`

This allows visualization in RViz and integration with other ROS tools.

---

## Build Instructions

```bash
colcon build
source install/setup.bash
````

---

## Run Instructions

### Terminal 1 – Build workspace

```bash
colcon build
```

---

### Terminal 2 – Robot description / simulation

```bash
source install/setup.bash
ros2 launch robot_description gazebo.launch.py
```

---

### Terminal 3 – Controller node

```bash
source install/setup.bash
ros2 launch robot_controller controller.launch.py
```

---

### Terminal 4 – Visualization

```bash
rviz2
```

In RViz:

* Set Fixed Frame → `odom`
* Add → TF display

---

### Terminal 5 – Send velocity command

```bash
ros2 topic pub /cmd_vel geometry_msgs/TwistStamped "
header:
  stamp:
    sec: 0
    nanosec: 0
  frame_id: ''
twist:
  linear:
    x: 0.2
    y: 0.0
    z: 0.0
  angular:
    x: 0.0
    y: 0.0
    z: 0.5
"
```

---

## Notes

* This is a differential drive odometry implementation based on wheel encoders
* Assumes correct ordering of wheel joints in `JointState`
* No sensor fusion (pure odometry)
* Designed for ROS 2 simulation and visualization

```

