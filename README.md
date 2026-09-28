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
[pantilt_ros2](https://github.com/adityakamath/pantilt_ros2). All plugins live in one library and share
one header, `mujoco_ros2_plugins.hpp`.

| Plugin class | What it does |
|---|---|
| `mujoco_ros2_plugins/EmergencyStopPlugin` | Serves `/emergency_stop` (`std_srvs/SetBool`) and disables all simulated motor actuation while active |
| `mujoco_ros2_plugins/ImuPlugin` | Publishes ideal six-axis MuJoCo accelerometer and gyro readings as `sensor_msgs/Imu` |

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
The stop remains active across a world reset until `/emergency_stop` is called
with `false`.

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
```

```sh
ros2 service call /emergency_stop std_srvs/srv/SetBool "{data: true}"    # stop
ros2 service call /emergency_stop std_srvs/srv/SetBool "{data: false}"   # release
```

A robot package that loads the plugin should list `mujoco_ros2_plugins` as an `exec_depend`: a plugin
class that cannot be found is a fatal error in `mujoco_ros2_control`, not a skip.

### Limitations

The service name is fixed to `/emergency_stop`, so a model with several independent robots shares one
stop. The stop disables MuJoCo actuation, corresponding to a torque cut; passive joint friction still applies. Only simulation is covered: on real hardware the stop comes from the hardware interface.

## Tests

`colcon test --packages-select mujoco_ros2_plugins`. The C++ tests run the stop against real MuJoCo
models (velocity and position servos, torque disable, release and reset); the Python test checks
the plugin registration.


## ImuPlugin

This plugin publishes one ideal six-axis IMU from a MuJoCo accelerometer and gyro
attached to the same site. Add both sensors to the MJCF, then configure the plugin
under `mujoco_plugins`:

```xml
<sensor>
  <accelerometer name="camera_accelerometer" site="camera_imu_site"/>
  <gyro name="camera_gyroscope" site="camera_imu_site"/>
</sensor>
```

```yaml
/**:
  ros__parameters:
    mujoco_plugins:
      camera_imu:
        type: "mujoco_ros2_plugins/ImuPlugin"
        accelerometer: camera_accelerometer
        gyroscope: camera_gyroscope
        topic: /camera/imu
        frame_id: camera_imu_frame
        publish_rate: 100.0
```

The five settings use the defaults `accelerometer`, `gyroscope`, `imu`,
`imu_link` and 100 Hz, respectively. Initialization fails if the named sensors
are missing, have the wrong types, or refer to different sites; `publish_rate`
must be finite and positive. Samples use simulation time and MuJoCo specific
force, including gravity at rest. `orientation_covariance[0] = -1` marks the
unavailable orientation estimate; zero velocity and acceleration covariance
matrices mean unknown uncertainty, not calibrated noise. The requested rate is
bounded by the simulation update rate. Sampling resumes after a simulation-time
reset. LeKiwi uses this plugin for its simulated Gemini 2 camera IMU.
