#include <cmath>
#include <string>
#include <sensor_msgs/msg/imu.hpp>
#include "mujoco_ros2_plugins/mujoco_ros2_plugins.hpp"

namespace mujoco_ros2_plugins
{
// Ideal six-axis sensor at a MuJoCo site: includes gravity and articulated-body motion.
class ImuPlugin : public PluginBase
{
public:
  bool init(rclcpp::Node::SharedPtr node, const mjModel * model, mjData *) override
  {
    const std::string prefix = "mujoco_plugins.camera_imu.";
    auto text = [&](const std::string & key, const std::string & fallback) {
        const auto name = prefix + key;
        if (!node->has_parameter(name)) {node->declare_parameter(name, fallback);}
        return node->get_parameter(name).as_string();
      };
    const auto accel = text("accelerometer", "accelerometer");
    const auto gyro = text("gyroscope", "gyroscope");
    frame_ = text("frame_id", "imu_link");
    const auto topic = text("topic", "imu");
    const auto rate_name = prefix + "publish_rate";
    if (!node->has_parameter(rate_name)) {node->declare_parameter(rate_name, 100.0);}
    const double rate = node->get_parameter(rate_name).as_double();
    if (!std::isfinite(rate) || rate <= 0) {return false;}
    period_ = 1.0 / rate;
    const int a = mj_name2id(model, mjOBJ_SENSOR, accel.c_str());
    const int g = mj_name2id(model, mjOBJ_SENSOR, gyro.c_str());
    if (a < 0 || g < 0 || model->sensor_type[a] != mjSENS_ACCELEROMETER ||
      model->sensor_type[g] != mjSENS_GYRO || model->sensor_objid[a] != model->sensor_objid[g])
    {
      RCLCPP_ERROR(node->get_logger(), "IMU requires accelerometer and gyro at the same site");
      return false;
    }
    accel_ = model->sensor_adr[a];
    gyro_ = model->sensor_adr[g];
    publisher_ = node->create_publisher<sensor_msgs::msg::Imu>(topic, 5);
    return true;
  }

  void update(const mjModel *, mjData * data) override
  {
    if (data->time >= last_ && data->time - last_ + 1e-9 < period_) {return;}
    last_ = data->time;
    sensor_msgs::msg::Imu msg;
    msg.header.stamp = rclcpp::Time(static_cast<int64_t>(data->time * 1e9));
    msg.header.frame_id = frame_;
    msg.orientation_covariance[0] = -1;  // Six-axis IMU supplies no orientation estimate.
    msg.linear_acceleration.x = data->sensordata[accel_];
    msg.linear_acceleration.y = data->sensordata[accel_ + 1];
    msg.linear_acceleration.z = data->sensordata[accel_ + 2];
    msg.angular_velocity.x = data->sensordata[gyro_];
    msg.angular_velocity.y = data->sensordata[gyro_ + 1];
    msg.angular_velocity.z = data->sensordata[gyro_ + 2];
    // Zero covariance denotes unknown, rather than claiming calibrated hardware noise.
    publisher_->publish(msg);
  }

  void cleanup() override {publisher_.reset();}

private:
  int accel_{0}, gyro_{0};
  double period_{0.01}, last_{-1};
  std::string frame_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_;
};
}  // namespace mujoco_ros2_plugins
MUJOCO_ROS2_PLUGINS_EXPORT(mujoco_ros2_plugins::ImuPlugin)
