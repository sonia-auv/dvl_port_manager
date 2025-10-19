# dvl_port_manager

*description here*

---

## Dependencies

### ROS 2 Distro

* Humble

### ROS 2 Packages

* `ament_cmake`
* `rclcpp`
* `std_msgs`
* `std_srvs`
* `sonia_common_ros2`
* `sensor_msgs`

### Additional packages

* `sonia_common_cpp`

---

## Node

* Name: `dvl_provider`
* Port type: TCP, UDP

---

## Registered Topics / Services / Actions

| Type                             | Name                               | Direction       | Message/Service Type                 | Description                        |
| -------------------------------- | ---------------------------------- | ----------------| ------------------------------------ | ---------------------------------- |
| Topic                            | `/provider_dvl/dvl_velocity`       | Published       | `sonia_common_ros2/msg/BodyVelocity` | Velocity data from the dvl         |
| Topic                            | `/provider_dvl/dvl_leak_sensor`    | Published       | `std_msgs/msg/Bool`                  | Signal for leakage from the dvl    |
| Topic                            | `/provider_dvl/enable_disable_dvl` | Subscribed      | `std_msgs/msg/Bool`                  | Signal the dvl to start            |

---
## Build Instructions
To build the project, the following commands should be run directly from your ROS2 workspace.

```bash
colcon build --packages-select dvl_port_manager --symlink-install
source install/setup.bash
```
---

## Launch Instructions

### Default launch

```bash
ros2 launch dvl_port_manager launch.py
```

---

## Useful ROS 2 Commands

```bash
ros2 node list
ros2 node info /dvl_provider
ros2 topic echo /provider_dvl/dvl_velocity
ros2 param list /dvl_provider
```

---

## References

* [ROS 2 Humble Documentation](https://docs.ros.org/en/humble/)
* [sonia_common_ros2](https://github.com/sonia-auv/sonia_common_ros2)

---