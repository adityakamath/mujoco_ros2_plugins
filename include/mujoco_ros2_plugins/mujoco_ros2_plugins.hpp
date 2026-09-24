#ifndef MUJOCO_ROS2_PLUGINS__MUJOCO_ROS2_PLUGINS_HPP_
#define MUJOCO_ROS2_PLUGINS__MUJOCO_ROS2_PLUGINS_HPP_

#include <mujoco/mujoco.h>

#include <atomic>

#include <mujoco_ros2_control_plugins/mujoco_ros2_control_plugins_base.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>

// The one header of this package: the plugin base class, the export macro and the ROS-free
// cores of the plugins (testable with plain MuJoCo).
namespace mujoco_ros2_plugins
{

using PluginBase = mujoco_ros2_control_plugins::MuJoCoROS2ControlPluginBase;

// ROS-free core of the emergency stop, so it can be tested with plain MuJoCo. Robot-agnostic.
// Matches sts_hardware_interface's real behavior exactly: EnableTorque(motor, 0) on every
// motor, nothing else - no position hold, no brake. A torque-disabled motor free-spins or
// free-swings under a hand or gravity; whatever keeps a joint from moving (e.g. a pan-tilt's
// self-locking gearbox) has to come from that joint's own passive friction, not from here.
class EmergencyStop
{
public:
  // Any thread.
  void set_active(bool active) {active_.store(active);}
  bool active() const {return active_.load();}
  // Physics thread: no latched state to clear on a world reset.
  void reset_latch() {}

  // Physics thread, immediately before every mj_step(). Toggles MuJoCo's whole-simulation
  // actuation switch, forcing every actuator's force to zero regardless of type or gains -
  // the simulated equivalent of cutting power to every motor. Commands (ctrl) keep flowing
  // in from controllers as normal; they just stop reaching the joints, same as the real
  // interface leaving hw_cmd_* alone and disabling torque at the servo instead.
  void apply(const mjModel * model, mjData * /*data*/)
  {
    // model is nominally const (compiled, shared) but mjOption's runtime toggles, including
    // this one, are designed to be flipped live - MuJoCo's own UI does the same.
    mjOption & opt = const_cast<mjModel *>(model)->opt;
    if (active_.load()) {
      opt.disableflags |= mjDSBL_ACTUATION;
    } else {
      opt.disableflags &= ~mjDSBL_ACTUATION;
    }
  }

private:
  std::atomic<bool> active_{false};
};

}  // namespace mujoco_ros2_plugins

// Registers a plugin class under this package's loader base class.
#define MUJOCO_ROS2_PLUGINS_EXPORT(plugin_class) \
  PLUGINLIB_EXPORT_CLASS(plugin_class, mujoco_ros2_plugins::PluginBase)

#endif  // MUJOCO_ROS2_PLUGINS__MUJOCO_ROS2_PLUGINS_HPP_
