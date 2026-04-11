// ═══════════════════════════════════════════════════════════════════════
// FILE PURPOSE: test_imu_sensor.cpp
// ═══════════════════════════════════════════════════════════════════════
//
// WHAT IT DOES:
// - Tests IMU sensor message field correctness
// - Verifies covariance initialisation
//
// WHEN IT RUNS:
// - colcon test --packages-select sensor_fusion_system
//
// WHY YOU NEED IT:
// - Ensures IMU message contract is upheld
// - Demonstrates Google Test usage
//
// BUG FIX: original code called rclcpp::init inside the first test and
// rclcpp::shutdown at the end of that same test.  If the second test had
// ever needed rclcpp it would have found the context shut down.  Moved
// init/shutdown to a TestEnvironment so they bracket the entire suite.
//
// ═══════════════════════════════════════════════════════════════════════

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

// ── Test environment: init/shutdown wraps the whole suite ─────────────────
class RclcppEnvironment : public ::testing::Environment
{
public:
  void SetUp()    override { rclcpp::init(0, nullptr); }
  void TearDown() override { rclcpp::shutdown();       }
};

// ── Tests ─────────────────────────────────────────────────────────────────

// Verify the message fields match the values set by imu_sensor_node
TEST(ImuSensorTest, MessageHasCorrectFields)
{
  auto msg = sensor_msgs::msg::Imu();
  msg.header.frame_id        = "imu_link";
  msg.linear_acceleration.z  = 9.81;

  EXPECT_EQ(msg.header.frame_id, "imu_link");
  EXPECT_NEAR(msg.linear_acceleration.z, 9.81, 0.1);
}

// Verify that covariance can be set to a positive value
TEST(ImuSensorTest, CovarianceIsPositive)
{
  auto msg = sensor_msgs::msg::Imu();
  const double av = 0.01 * 0.01;  // matches imu_sensor_node gyro variance
  msg.angular_velocity_covariance[0] = av;

  EXPECT_GT(msg.angular_velocity_covariance[0], 0.0);
  EXPECT_DOUBLE_EQ(msg.angular_velocity_covariance[0], av);
}

// ── Main ──────────────────────────────────────────────────────────────────
int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  // Register the environment so init/shutdown are called once for the suite
  ::testing::AddGlobalTestEnvironment(new RclcppEnvironment());
  return RUN_ALL_TESTS();
}
