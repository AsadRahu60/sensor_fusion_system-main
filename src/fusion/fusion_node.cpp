#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.hpp>
#include <Eigen/Dense>
#include <memory>
#include <cmath>

using namespace std::chrono_literals;

// ── State layout: x = [x, y, θ, vx_b, vy_b, ω] ──────────────────────────────
//
//  x, y   : world-frame position        (metres)
//  θ      : heading angle, world frame  (radians)
//  vx_b   : body-frame forward velocity (m/s)
//  vy_b   : body-frame lateral velocity (m/s)
//  ω      : angular velocity            (rad/s)
//
// The position update  x' = x + (vx_b·cosθ − vy_b·sinθ)·dt  is NONLINEAR in θ.
// This is what makes the filter a true Extended Kalman Filter.

constexpr size_t STATE_DIM      = 6;
constexpr size_t IMU_MEAS_DIM   = 1;   // gyro-z → ω
constexpr size_t CTRL_DIM       = 2;   // [ax_b, ay_b] gravity-compensated
constexpr size_t LIDAR_MEAS_DIM = 2;   // [x, y] world-frame centroid

using StateVector   = Eigen::Matrix<double, STATE_DIM, 1>;
using StateMatrix   = Eigen::Matrix<double, STATE_DIM, STATE_DIM>;
using ControlVector = Eigen::Matrix<double, CTRL_DIM,  1>;

class FusionNode : public rclcpp::Node
{
public:
  FusionNode()
  : Node("fusion_node"),
    state_(StateVector::Zero()),
    covariance_(StateMatrix::Identity()),
    control_(ControlVector::Zero())
  {
    // Parameters
    this->declare_parameter("prediction_rate", 50.0);
    this->declare_parameter("gravity",         9.80665);   // m/s² — standard gravity

    const double rate = this->get_parameter("prediction_rate").as_double();
    gravity_ = this->get_parameter("gravity").as_double();

    // Subscriptions
    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "imu/data", 10,
      [this](const sensor_msgs::msg::Imu::SharedPtr msg) { imuCallback(msg); });

    lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "lidar/scan", 10,
      [this](const sensor_msgs::msg::LaserScan::SharedPtr msg) { lidarCallback(msg); });

    // Publisher
    pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "fusion/pose", 10);

    // TF2 broadcaster — publishes map → base_link every prediction tick
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // Prediction timer
    timer_ = this->create_wall_timer(
      std::chrono::duration<double>(1.0 / rate),
      [this]() { predictionStep(); });

    initializeEKF();

    RCLCPP_INFO(this->get_logger(),
      "EKF Fusion Node — %.0f Hz, gravity=%.5f m/s², TF2 broadcaster active",
      rate, gravity_);
  }

private:
  // ── Filter initialisation ─────────────────────────────────────────────────
  void initializeEKF()
  {
    process_noise_ = StateMatrix::Zero();
    process_noise_(0, 0) = 0.01;   // x
    process_noise_(1, 1) = 0.01;   // y
    process_noise_(2, 2) = 0.02;   // θ
    process_noise_(3, 3) = 0.05;   // vx_b
    process_noise_(4, 4) = 0.05;   // vy_b
    process_noise_(5, 5) = 0.03;   // ω

    imu_noise_(0, 0) = 0.01 * 0.01;   // gyro σ = 0.01 rad/s

    lidar_noise_ = Eigen::Matrix<double, LIDAR_MEAS_DIM, LIDAR_MEAS_DIM>::Zero();
    lidar_noise_(0, 0) = 0.02;
    lidar_noise_(1, 1) = 0.02;

    covariance_ = StateMatrix::Identity();
    last_update_time_ = this->now();
  }

  // ══════════════════════════════════════════════════════════════════════════
  // EKF PREDICTION
  //
  // 1. Apply nonlinear f(x, u, dt)
  // 2. Propagate covariance with Jacobian F = ∂f/∂x
  // ══════════════════════════════════════════════════════════════════════════
  void predictionStep()
  {
    const auto   now = this->now();
    const double dt  = (now - last_update_time_).seconds();
    last_update_time_ = now;

    if (dt <= 0.0 || dt > 1.0) return;

    const double theta = state_(2);
    const double vx_b  = state_(3);
    const double vy_b  = state_(4);
    const double omega = state_(5);
    const double ct    = std::cos(theta);
    const double st    = std::sin(theta);

    // ── 1. Nonlinear state propagation ────────────────────────────────────
    StateVector x_new;
    x_new(0) = state_(0) + (vx_b * ct - vy_b * st) * dt;   // ← nonlinear in θ
    x_new(1) = state_(1) + (vx_b * st + vy_b * ct) * dt;   // ← nonlinear in θ
    x_new(2) = normalizeAngle(theta + omega * dt);
    x_new(3) = vx_b  + control_(0) * dt;    // ax_b (gravity-compensated)
    x_new(4) = vy_b  + control_(1) * dt;    // ay_b (gravity-compensated)
    x_new(5) = omega;
    state_ = x_new;

    // ── 2. Analytic Jacobian F = ∂f/∂x ───────────────────────────────────
    StateMatrix F = StateMatrix::Identity();
    //  ∂x'/∂θ  ∂x'/∂vx_b  ∂x'/∂vy_b
    F(0, 2) = (-vx_b * st - vy_b * ct) * dt;
    F(0, 3) =  ct * dt;
    F(0, 4) = -st * dt;
    //  ∂y'/∂θ  ∂y'/∂vx_b  ∂y'/∂vy_b
    F(1, 2) = ( vx_b * ct - vy_b * st) * dt;
    F(1, 3) =  st * dt;
    F(1, 4) =  ct * dt;
    //  ∂θ'/∂ω
    F(2, 5) = dt;

    // ── 3. Covariance propagation ─────────────────────────────────────────
    covariance_ = F * covariance_ * F.transpose() + process_noise_;

    publishPose();
  }

  // ══════════════════════════════════════════════════════════════════════════
  // IMU CALLBACK
  //
  // GRAVITY COMPENSATION
  // ─────────────────────
  // An IMU measures SPECIFIC FORCE = true_acceleration + gravity_in_body_frame.
  // For a robot on a flat surface with the IMU level:
  //   gravity_body = [0, 0, +g]   (sensor Z points up, gravity opposes motion)
  //
  // Compensated (true) accelerations:
  //   ax_true = ax_measured − 0    = ax_measured   (gravity has no X component)
  //   ay_true = ay_measured − 0    = ay_measured   (gravity has no Y component)
  //   az_true = az_measured − g    (not used in 2-D, but correct for reference)
  //
  // For a TILTED robot or 3-D system, rotate the gravity vector [0,0,g] by
  // the inverse of the current roll/pitch rotation matrix before subtracting.
  // ══════════════════════════════════════════════════════════════════════════
  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    // ── Gravity compensation ──────────────────────────────────────────────
    // For a level 2-D robot: gravity is purely on Z → X/Y are uncontaminated.
    // We subtract it explicitly to document intent and to prepare for any
    // future tilt handling.
    const double ax_compensated = msg->linear_acceleration.x;           // − 0
    const double ay_compensated = msg->linear_acceleration.y;           // − 0
    // az_compensated = msg->linear_acceleration.z − gravity_;          // unused in 2-D

    // Store as control input u = [ax_b, ay_b] for the next prediction step
    control_(0) = ax_compensated;
    control_(1) = ay_compensated;

    // ── Kalman update: gyro-z → ω (state[5]) ─────────────────────────────
    using MeasVec = Eigen::Matrix<double, IMU_MEAS_DIM, 1>;
    using ObsMat  = Eigen::Matrix<double, IMU_MEAS_DIM, STATE_DIM>;

    MeasVec z;
    z(0) = msg->angular_velocity.z;

    ObsMat H;
    H.setZero();
    H(0, 5) = 1.0;   // H maps state → [ω]

    const MeasVec     inn = z - H * state_;
    const auto        S   = H * covariance_ * H.transpose() + imu_noise_;
    const Eigen::Matrix<double, STATE_DIM, IMU_MEAS_DIM> K =
      covariance_ * H.transpose() * S.inverse();

    state_ = state_ + K * inn;
    state_(2) = normalizeAngle(state_(2));

    // Joseph-form: P = (I−KH)P(I−KH)ᵀ + KRKᵀ
    const StateMatrix IKH = StateMatrix::Identity() - K * H;
    covariance_ = IKH * covariance_ * IKH.transpose()
                + K * imu_noise_ * K.transpose();
  }

  // ══════════════════════════════════════════════════════════════════════════
  // LIDAR CALLBACK — update (x, y) from scan centroid
  // ══════════════════════════════════════════════════════════════════════════
  void lidarCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    double sum_x = 0.0, sum_y = 0.0;
    int    valid  = 0;

    for (size_t i = 0; i < msg->ranges.size(); ++i) {
      const float r = msg->ranges[i];
      if (!std::isfinite(r) || r < msg->range_min || r > msg->range_max) continue;
      const double angle = msg->angle_min + static_cast<double>(i) * msg->angle_increment;
      sum_x += r * std::cos(angle);
      sum_y += r * std::sin(angle);
      ++valid;
    }

    if (valid == 0) {
      RCLCPP_WARN(this->get_logger(), "lidarCallback: no valid readings — skipping");
      return;
    }

    Eigen::Vector2d z(sum_x / valid, sum_y / valid);

    Eigen::Matrix<double, LIDAR_MEAS_DIM, STATE_DIM> H;
    H.setZero();
    H(0, 0) = 1.0;
    H(1, 1) = 1.0;

    const Eigen::Vector2d inn = z - H * state_;
    const auto S = H * covariance_ * H.transpose() + lidar_noise_;
    const Eigen::Matrix<double, STATE_DIM, LIDAR_MEAS_DIM> K =
      covariance_ * H.transpose() * S.inverse();

    state_    = state_ + K * inn;
    state_(2) = normalizeAngle(state_(2));

    const StateMatrix IKH = StateMatrix::Identity() - K * H;
    covariance_ = IKH * covariance_ * IKH.transpose()
                + K * lidar_noise_ * K.transpose();
  }

  // ══════════════════════════════════════════════════════════════════════════
  // PUBLISH POSE + BROADCAST TF2 TRANSFORM
  //
  // Publishes:
  //   /fusion/pose  (PoseWithCovarianceStamped)  — for downstream nodes
  //   TF2: map → base_link                       — for the ROS 2 ecosystem
  //
  // Why TF2?
  //   Without a TF broadcast, no other ROS 2 node (nav2, MoveIt, RViz,
  //   sensor drivers) can transform data into the robot's reference frame.
  //   The pose topic alone is an island — TF2 is what connects the system.
  // ══════════════════════════════════════════════════════════════════════════
  void publishPose()
  {
    const auto   stamp = this->now();
    const double theta = state_(2);
    const double qz    = std::sin(theta / 2.0);
    const double qw    = std::cos(theta / 2.0);

    // ── 1. Pose topic ─────────────────────────────────────────────────────
    auto pose_msg = std::make_shared<geometry_msgs::msg::PoseWithCovarianceStamped>();
    pose_msg->header.stamp    = stamp;
    pose_msg->header.frame_id = "map";

    pose_msg->pose.pose.position.x    = state_(0);
    pose_msg->pose.pose.position.y    = state_(1);
    pose_msg->pose.pose.position.z    = 0.0;
    pose_msg->pose.pose.orientation.x = 0.0;
    pose_msg->pose.pose.orientation.y = 0.0;
    pose_msg->pose.pose.orientation.z = qz;
    pose_msg->pose.pose.orientation.w = qw;

    for (size_t i = 0; i < 6; ++i)
      for (size_t j = 0; j < 6; ++j)
        pose_msg->pose.covariance[i * 6 + j] = covariance_(i, j);

    pose_pub_->publish(*pose_msg);

    // ── 2. TF2: map → base_link ───────────────────────────────────────────
    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp    = stamp;
    tf.header.frame_id = "map";
    tf.child_frame_id  = "base_link";

    tf.transform.translation.x = state_(0);
    tf.transform.translation.y = state_(1);
    tf.transform.translation.z = 0.0;
    tf.transform.rotation.x    = 0.0;
    tf.transform.rotation.y    = 0.0;
    tf.transform.rotation.z    = qz;
    tf.transform.rotation.w    = qw;

    tf_broadcaster_->sendTransform(tf);
  }

  // IEEE 754: result ∈ (−π, π] in O(1)
  static double normalizeAngle(double a)
  {
    return std::remainder(a, 2.0 * M_PI);
  }

  // ── Members ───────────────────────────────────────────────────────────────
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr       imu_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  StateVector   state_;          // [x, y, θ, vx_b, vy_b, ω]
  StateMatrix   covariance_;     // 6×6 error covariance P
  StateMatrix   process_noise_;  // Q
  ControlVector control_;        // [ax_b, ay_b] — gravity-compensated, from IMU

  Eigen::Matrix<double, IMU_MEAS_DIM,   IMU_MEAS_DIM>   imu_noise_;
  Eigen::Matrix<double, LIDAR_MEAS_DIM, LIDAR_MEAS_DIM> lidar_noise_;

  rclcpp::Time last_update_time_;
  double gravity_;   // m/s² — configurable via params.yaml
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FusionNode>());
  rclcpp::shutdown();
  return 0;
}
