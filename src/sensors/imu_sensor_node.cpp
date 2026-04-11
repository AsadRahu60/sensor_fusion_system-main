#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <random>
#include <cmath>

using namespace std::chrono_literals;

// ── IMU Sensor Node ──────────────────────────────────────────────────────────
//
// Simulation stub: publishes a gravity-only IMU signal at 100 Hz.
//
// What is simulated:
//   - linear_acceleration.z = 9.81 m/s²  (gravity, constant)
//   - All other fields (x/y acceleration, angular velocity) = 0
//
// What is NOT simulated (intentional simplification for this portfolio demo):
//   - Robot motion  — no translational velocity or rotation is modelled
//   - Sensor noise  — see the `add_noise` parameter; enable for realism
//   - Bias drift    — real IMUs accumulate gyro/accel bias over time
//
// In a real deployment, replace this node with a hardware driver (e.g.
// ros2_imu_tools, Adafruit BNO055, or similar) that streams actual IMU data.

class ImuNode : public rclcpp::Node
{
public:
  ImuNode() : Node("imu_sensor_node"),
    rng_(std::random_device{}()),
    accel_noise_(0.0, 0.05),   // σ = 0.05 m/s²
    gyro_noise_ (0.0, 0.01)    // σ = 0.01 rad/s
  {
    // Declare parameters (configurable via params.yaml)
    this->declare_parameter("publish_rate", 100.0);
    this->declare_parameter("add_noise",    true);

    double rate = this->get_parameter("publish_rate").as_double();
    add_noise_  = this->get_parameter("add_noise").as_bool();

    pub_ = this->create_publisher<sensor_msgs::msg::Imu>("imu/data", 10);

    timer_ = this->create_wall_timer(
      std::chrono::duration<double>(1.0 / rate),
      [this]() { this->publish(); }
    );

    RCLCPP_INFO(this->get_logger(),
      "IMU Node started at %.0f Hz (add_noise=%s — gravity-only stub)",
      rate, add_noise_ ? "true" : "false");
  }

private:
  void publish()
  {
    auto msg = sensor_msgs::msg::Imu();
    msg.header.stamp    = this->now();
    msg.header.frame_id = "imu_link";

    // Gravity along Z (sensor is assumed level)
    msg.linear_acceleration.z = 9.81;

    // Optional Gaussian noise — models sensor measurement uncertainty.
    // Disabled by default; set add_noise: true in config/params.yaml to enable.
    if (add_noise_) {
      msg.linear_acceleration.x += accel_noise_(rng_);
      msg.linear_acceleration.y += accel_noise_(rng_);
      msg.linear_acceleration.z += accel_noise_(rng_);
      msg.angular_velocity.x    += gyro_noise_(rng_);
      msg.angular_velocity.y    += gyro_noise_(rng_);
      msg.angular_velocity.z    += gyro_noise_(rng_);
    }

    // Covariance: diagonal, tuned to match noise model above
    // Row-major 3x3 matrix: [σ²_x, 0, 0,  0, σ²_y, 0,  0, 0, σ²_z]
    const double av = 0.01 * 0.01;  // angular velocity variance
    const double la = 0.05 * 0.05;  // linear acceleration variance
    msg.angular_velocity_covariance    = {av, 0, 0,  0, av, 0,  0, 0, av};
    msg.linear_acceleration_covariance = {la, 0, 0,  0, la, 0,  0, 0, la};

    pub_->publish(msg);
  }

  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr                        timer_;

  bool                              add_noise_;
  std::mt19937                      rng_;
  std::normal_distribution<double>  accel_noise_;
  std::normal_distribution<double>  gyro_noise_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImuNode>());
  rclcpp::shutdown();
  return 0;
}
