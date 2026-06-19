# ROS2 Differential Drive Odometry Controller

This project implements a ROS 2 differential drive controller that performs:
- velocity control from `/cmd_vel`
- wheel velocity conversion
- odometry estimation from wheel encoders
- TF broadcasting (`odom → base_footprint`)

---

# Differential Drive Kinematics

## 1. Wheel Encoder Changes

\[
\Delta p_l = p_l(t) - p_l(t-1)
\]

\[
\Delta p_r = p_r(t) - p_r(t-1)
\]

---

## 2. Wheel Angular Velocities

\[
\omega_l = \frac{\Delta p_l}{\Delta t}, \quad \omega_r = \frac{\Delta p_r}{\Delta t}
\]

---

## 3. Robot Linear Velocity

\[
v = \frac{r}{2}\left(\omega_r + \omega_l\right)
\]

---

## 4. Robot Angular Velocity

\[
\omega = \frac{r}{b}\left(\omega_r - \omega_l\right)
\]

Where:
- \( r \) = wheel radius  
- \( b \) = wheel separation  

---

## 5. Discrete Motion (Odometry)

\[
\Delta s = \frac{r}{2}\left(\Delta p_r + \Delta p_l\right)
\]

\[
\Delta \theta = \frac{r}{b}\left(\Delta p_r - \Delta p_l\right)
\]

---

## 6. Pose Update

\[
x_{t+1} = x_t + \Delta s \cos(\theta)
\]

\[
y_{t+1} = y_t + \Delta s \sin(\theta)
\]

\[
\theta_{t+1} = \theta_t + \Delta \theta
\]

---

# ROS Topics

**Subscribed:**
- `/cmd_vel` → `geometry_msgs/TwistStamped`
- `joint_states` → `sensor_msgs/JointState`

**Published:**
- `robot_controller/odom` → `nav_msgs/Odometry`
- `simple_velocity_controller/commands` → `std_msgs/Float64MultiArray`

---

# TF Tree

\[
odom \rightarrow base\_footprint
\]

---

# Run Instructions

## Terminal 1
```bash
colcon build
````

---

## Terminal 2

```bash
source install/setup.bash
ros2 launch robot_description gazebo.launch.py
```

---

## Terminal 3

```bash
source install/setup.bash
ros2 launch robot_controller controller.launch.py
```

---

## Terminal 4

```bash
rviz2
```

Set:

* Fixed Frame → `odom`
* Add → TF

---

## Terminal 5 (Correct cmd_vel)

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

```

---
```
