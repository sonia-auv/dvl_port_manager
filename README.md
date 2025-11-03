# dvl_port_manager

This project provides a ROS 2 interface for Doppler Velocity Log (DVL) sensors, supporting both the **Nortek DVL 500** and **Pathfinder DVL** models. It communicates with the DVL over an Ethernet connection using either TCP or UDP protocols, depending on configuration. The node receives raw data streams containing velocity, altitude, and diagnostic information, parses them to extract bottom-track and water-track velocity measurements, and publishes the results through **ROS2** communication to the prototypes control system for navigation. 

---

## Dependencies

### ROS 2 Distro

* Humble

### ROS 2 Packages

* `ament_cmake`
* `rclcpp`
* `std_msgs`
* `std_srvs`
* `sensor_msgs`

### Sonia packages

* `sonia_common_cpp`
* `sonia_common_ros2`

---

## Node

* Name: `dvl_provider`
* Port Name: `{dvl_ip_address}`
* Port type: Ethernet
* Port: TCP, UDP

---

## Registered Topics / Services / Actions

| Type                  | Name                               | Direction       | Message/Service Type                 | Description                        |
| --------------------- | ---------------------------------- | ----------------| ------------------------------------ | ---------------------------------- |
| Topic                 | `/provider_dvl/dvl_velocity`       | Published       | `sonia_common_ros2/msg/BodyVelocity` | Body velocity data from the dvl    |
| Topic                 | `/provider_dvl/dvl_leak_sensor`    | Published       | `std_msgs/msg/Bool`                  | Signal for leakage from the dvl    |
| Topic                 | `/provider_dvl/enable_disable_dvl` | Subscribed      | `std_msgs/msg/Bool`                  | Signal the dvl to start or stop    |

---
## Build Instructions
To build the project, the following commands should be run directly from your **ROS2** workspace.

```bash
colcon build --packages-select dvl_port_manager --symlink-install
source install/setup.bash
```
---

## Launch Instructions

### Environment variables
Required environment variables to launch the project

```bash
export AUV={prototype_identifier}
```
replace `{prototype_identifier}` with available options: `AUV8` | `AUV7`.

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

* [sonia_common_ros2](https://github.com/sonia-auv/sonia_common_ros2)
* [Teledyne Marine Pathfinder DVL](https://www.teledynemarine.com/brands/rdi/pathfinder-dvl)
* [Teledyne Marine Pathfinder DVL User Manual](https://www.teledynemarine.com/en-us/resources/Documents/Brand%20Support/RD%20INSTRUMENTS/Technical%20Resources/Manuals%20and%20Guides/Pathfinder/PathFinder%20DVL%20Guide_Apr22.pdf)
* [Nortek DVL 500](https://www.nortekgroup.com/products/dvl500-300-m)

---