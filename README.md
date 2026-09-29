# mujoco_ros2_plugins

![Project Status](https://img.shields.io/badge/Status-Active-brightgreen)
![ROS 2](https://img.shields.io/badge/ROS_2-blue?style=flat&logo=ros&logoSize=auto)
![C++](https://img.shields.io/badge/C++-17-blue?style=flat&logo=cplusplus&logoColor=white)
![MuJoCo](https://img.shields.io/badge/MuJoCo-3.x-orange)
[![CI](https://github.com/adityakamath/mujoco_ros2_plugins/actions/workflows/ci.yml/badge.svg)](https://github.com/adityakamath/mujoco_ros2_plugins/actions/workflows/ci.yml)
[![Ask DeepWiki (Experimental)](https://deepwiki.com/badge.svg)](https://deepwiki.com/adityakamath/mujoco_ros2_plugins)
![License](https://img.shields.io/github/license/adityakamath/mujoco_ros2_plugins?label=License)

Plugins for [mujoco_ros2_control](https://github.com/ros-controls/mujoco_ros2_control) that give a
simulation what a real robot's hardware interfaces provide. They know nothing about any particular
robot: they work from the actuators and joints in the MuJoCo model. Used by
[lekiwi_ros2](https://github.com/adityakamath/lekiwi_ros2) and
[pantilt_ros2](https://github.com/adityakamath/pantilt_ros2). The emergency-stop plugin keeps its ROS-free core in `mujoco_ros2_plugins.hpp`. Camera IMUs use the upstream `mujoco_ros2_control` IMU sensor mapping and standard `imu_sensor_broadcaster`; this package does not provide an IMU plugin.

| Plugin class | What it does |
|---|---|
| `mujoco_ros2_plugins/EmergencyStopPlugin` | Serves a configurable `std_srvs/SetBool` service (default `/emergency_stop`) and disables all simulated motor actuation while active |

## Adding a plugin

Add `src/<name>_plugin.cpp` that includes `mujoco_ros2_plugins.hpp`, derives from `PluginBase` and ends
with `MUJOCO_ROS2_PLUGINS_EXPORT(mujoco_ros2_plugins::<Name>Plugin)`, then list the file in `PLUGIN_SOURCES`
in `CMakeLists.txt` and the class in `plugins.xml`. Put logic that needs no ROS in that header (as
`EmergencyStop`) so it can be tested with plain MuJoCo.

## EmergencyStopPlugin

### Behaviour

While the stop is enabled, the plugin overrides every actuator command immediately before each physics
step, disabling MuJoCo actuation for all motors while controllers keep running. Releasing it restores actuation.

All actuator forces are zero while stopped, including velocity servos, position
servos and torque motors. Joints may coast or move under gravity; the plugin does
not latch joint positions or apply a brake. Controllers stay active throughout.
The stop remains active across a world reset until the configured service
(default `/emergency_stop`) is called with `false`.

### Use

Clone it into the workspace `src/` and build it with `colcon build --packages-select
mujoco_ros2_plugins`. It needs `mujoco_ros2_control` and its plugin base
package, `mujoco_ros2_control_plugins` (0.1.2 or newer, which added the `pre_step` and `on_reset` hooks used here):

```sh
sudo apt install ros-$ROS_DISTRO-mujoco-ros2-control ros-$ROS_DISTRO-mujoco-ros2-control-plugins
```

Both are released for Humble, Jazzy, Kilted, Lyrical and Rolling, but only 0.1.2 has the hooks this plugin
needs. CI builds and tests it on Humble, Kilted and Lyrical. On Jazzy and Rolling the apt binaries were still
older than 0.1.2 when last checked (the release is registered but not yet built there), so the build fails
until they catch up, or you build `mujoco_ros2_control` from source. Locally it has only been tested on Kilted.
Then add it to the robot's `mujoco_plugins` parameters:

```yaml
/**:
  ros__parameters:
    mujoco_plugins:
      emergency_stop_plugin:
        type: "mujoco_ros2_plugins/EmergencyStopPlugin"
        service_name: /emergency_stop  # optional; this is the default
```

```sh
ros2 service call /emergency_stop std_srvs/srv/SetBool "{data: true}"    # stop
ros2 service call /emergency_stop std_srvs/srv/SetBool "{data: false}"   # release
```

A robot package that loads the plugin should list `mujoco_ros2_plugins` as an `exec_depend`: a plugin
class that cannot be found is a fatal error in `mujoco_ros2_control`, not a skip.

### Limitations

`service_name` is read from `mujoco_plugins.<plugin-key>.service_name`; leaving it out uses
`/emergency_stop`. It may be set to an absolute name such as `/robot_a/emergency_stop`.
The setting changes only the service endpoint. The plugin uses MuJoCo's global actuation switch,
so every actuator in a shared model stops together even if multiple plugin instances have different
service names. Independent per-robot stops would require actuator-scoped control. Passive joint
friction still applies. Only simulation is covered: on real hardware the stop comes from the
hardware interface.

## Tests

`colcon test --packages-select mujoco_ros2_plugins`. The C++ tests run the stop against real MuJoCo
models (velocity and position servos, torque disable and release); the Python test checks
the plugin registration; the service test checks the default and configured endpoints.
