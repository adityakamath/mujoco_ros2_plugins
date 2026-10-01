#include <gtest/gtest.h>

#include <chrono>
#include <memory>
#include <string>

#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/set_bool.hpp>

#include "mujoco_ros2_plugins/mujoco_ros2_plugins.hpp"

namespace
{

using namespace std::chrono_literals;
using PluginBase = mujoco_ros2_plugins::PluginBase;

class EmergencyStopServiceTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    int argc = 0;
    rclcpp::init(argc, nullptr);
  }

  static void TearDownTestSuite() {rclcpp::shutdown();}

  void check_service(const std::string & configured_name, const std::string & expected_name)
  {
    rclcpp::NodeOptions options;
    options.automatically_declare_parameters_from_overrides(true);
    if (!configured_name.empty()) {
      options.append_parameter_override(
        "mujoco_plugins.emergency_stop_plugin.service_name", configured_name);
    }
    auto node = std::make_shared<rclcpp::Node>("emergency_stop_service_test", options);
    pluginlib::ClassLoader<PluginBase> loader(
      "mujoco_ros2_control_plugins", "mujoco_ros2_control_plugins::MuJoCoROS2ControlPluginBase");
    auto plugin = loader.createSharedInstance("mujoco_ros2_plugins/EmergencyStopPlugin");
    ASSERT_TRUE(plugin->init(node->create_sub_node("emergency_stop_plugin"), nullptr, nullptr));

    auto client = node->create_client<std_srvs::srv::SetBool>(expected_name);
    ASSERT_TRUE(client->wait_for_service(10s)) << "Service not available: " << expected_name;
    auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
    request->data = true;
    auto future = client->async_send_request(request);
    ASSERT_EQ(rclcpp::spin_until_future_complete(node, future, 10s),
      rclcpp::FutureReturnCode::SUCCESS) << "Service response timed out: " << expected_name;
    EXPECT_TRUE(future.get()->success);
    plugin->cleanup();
  }
};

TEST_F(EmergencyStopServiceTest, UsesExistingDefaultName)
{
  check_service("", "/emergency_stop");
}

TEST_F(EmergencyStopServiceTest, UsesConfiguredName)
{
  check_service("/robot_a/emergency_stop", "/robot_a/emergency_stop");
}

}  // namespace
