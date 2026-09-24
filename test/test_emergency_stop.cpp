#include <gtest/gtest.h>
#include <mujoco/mujoco.h>

#include <cmath>

#include "mujoco_ros2_plugins/mujoco_ros2_plugins.hpp"

namespace
{

// Two independent joints: a wheel (velocity servo) and a pan joint (position servo, limited
// range, with damping/friction/armature - a torque-disabled real STS3215 isn't frictionless).
const char * kModel = R"(
<mujoco>
  <compiler angle="radian"/>
  <option timestep="0.002"/>
  <worldbody>
    <body name="wheel" pos="0 0 0">
      <joint name="wheel_joint" type="hinge" axis="0 0 1" armature="0.01"/>
      <geom size="0.05"/>
    </body>
    <body name="arm" pos="1 0 0">
      <joint name="pan_joint" type="hinge" axis="0 0 1" range="-1.5 1.5" armature="0.01"
             damping="1.0" frictionloss="0.2"/>
      <geom size="0.05"/>
    </body>
  </worldbody>
  <actuator>
    <velocity name="wheel_joint" joint="wheel_joint" kv="5"/>
    <position name="pan_joint" joint="pan_joint" kp="50" kv="5" ctrlrange="-1.5 1.5"/>
  </actuator>
</mujoco>)";

class EmergencyStopTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    char error[512] = "";
    spec_ = mj_parseXMLString(kModel, nullptr, error, sizeof(error));
    ASSERT_NE(spec_, nullptr) << error;
    model_ = mj_compile(spec_, nullptr);
    ASSERT_NE(model_, nullptr);
    data_ = mj_makeData(model_);
  }
  void TearDown() override
  {
    mj_deleteData(data_);
    mj_deleteModel(model_);
    mj_deleteSpec(spec_);
  }
  void run(int steps, double wheel_command, double pan_command)
  {
    for (int i = 0; i < steps; ++i) {
      data_->ctrl[0] = wheel_command;  // what a controller writes every cycle
      data_->ctrl[1] = pan_command;
      stop_.apply(model_, data_);
      mj_step(model_, data_);
    }
  }
  double wheel_speed() const {return data_->qvel[model_->jnt_dofadr[0]];}
  double pan() const {return data_->qpos[model_->jnt_qposadr[1]];}

  mjSpec * spec_{nullptr};
  mjModel * model_{nullptr};
  mjData * data_{nullptr};
  mujoco_ros2_plugins::EmergencyStop stop_;
};

TEST_F(EmergencyStopTest, InactiveLeavesTheControllersCommandsAlone)
{
  run(500, 2.0, 0.6);
  EXPECT_DOUBLE_EQ(data_->ctrl[0], 2.0);
  EXPECT_DOUBLE_EQ(data_->ctrl[1], 0.6);
  EXPECT_GT(wheel_speed(), 1.5);
  EXPECT_NEAR(pan(), 0.6, 0.05);
  EXPECT_EQ(model_->opt.disableflags & mjDSBL_ACTUATION, 0);
}

TEST_F(EmergencyStopTest, DisablesActuationForEveryMotorEvenWhileTheControllerKeepsCommanding)
{
  run(500, 2.0, 0.6);
  ASSERT_GT(wheel_speed(), 1.5);
  const double speed_at_stop = wheel_speed();
  stop_.set_active(true);
  run(200, 2.0, 0.6);
  EXPECT_NE(model_->opt.disableflags & mjDSBL_ACTUATION, 0);
  // No friction on this test wheel, so a torque-disabled wheel coasts forever, exactly like
  // the real one: it isn't commanded to zero and it isn't braked, it's just no longer driven.
  // qfrc_actuator (the actuator's actual force output) is what proves that, not the speed.
  EXPECT_DOUBLE_EQ(data_->qfrc_actuator[model_->jnt_dofadr[0]], 0.0);
  EXPECT_NEAR(wheel_speed(), speed_at_stop, 1e-6);
  // ctrl itself is left alone; it's the actuator's *effect* that's cut, same as the real
  // interface leaving hw_cmd_* untouched and disabling torque at the servo instead.
  EXPECT_DOUBLE_EQ(data_->ctrl[0], 2.0);
  EXPECT_DOUBLE_EQ(data_->ctrl[1], 0.6);
}

TEST_F(EmergencyStopTest, APositionServoJointFreelyDriftsUnderAnExternalPushInsteadOfBeingHeld)
{
  run(500, 0.0, 0.6);
  const double at_stop = pan();
  stop_.set_active(true);
  const int pan_dof = model_->jnt_dofadr[1];
  for (int i = 0; i < 200; ++i) {
    data_->qfrc_applied[pan_dof] = 8.0;  // a hand pushing on the payload
    data_->ctrl[1] = 0.6;                // the controller still asking to hold position
    stop_.apply(model_, data_);
    mj_step(model_, data_);
  }
  // Torque is off, so a real external push moves it - it is not pinned to at_stop.
  EXPECT_GT(std::abs(pan() - at_stop), 0.1);
}

TEST_F(EmergencyStopTest, ReleaseHandsControlBack)
{
  run(300, 2.0, 0.6);
  stop_.set_active(true);
  run(300, 2.0, 0.6);
  stop_.set_active(false);
  run(500, 2.0, -0.5);
  EXPECT_EQ(model_->opt.disableflags & mjDSBL_ACTUATION, 0);
  EXPECT_DOUBLE_EQ(data_->ctrl[0], 2.0);
  EXPECT_DOUBLE_EQ(data_->ctrl[1], -0.5);
  EXPECT_GT(wheel_speed(), 1.5);
  EXPECT_NEAR(pan(), -0.5, 0.05);
}

}  // namespace
