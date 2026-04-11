#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <cmath>

// ── Safety thresholds ────────────────────────────────────────────────────────
static constexpr double kMaxPositionBound  = 100.0;   // metres
static constexpr double kMaxCovariance     = 10.0;    // filter-divergence sentinel
static constexpr double kMaxAccelMagnitude = 50.0;    // m/s²
static constexpr double kDropoutTimeout    = 1.0;     // seconds of silence = fault

class SafetyMonitorNode : public rclcpp::Node
{
public:
  SafetyMonitorNode()
  : Node("safety_monitor_node"),
    pose_healthy_(true),
    imu_healthy_(true)
  {
    pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "fusion/pose", 10,
      [this](const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
        last_pose_time_ = this->now();
        this->checkPoseSafety(msg);
      }
    );

    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "imu/data", 10,
      [this](const sensor_msgs::msg::Imu::SharedPtr msg) {
        last_imu_time_ = this->now();
        this->checkImuSafety(msg);
      }
    );

    service_ = this->create_service<std_srvs::srv::Trigger>(
      "safety/check",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
             std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        const bool healthy = isSystemHealthy();
        response->success = healthy;
        response->message = healthy ? "System OK" : buildFaultString();
      }
    );

    // ── UPGRADE: Dropout watchdog ─────────────────────────────────────────
    // Fires every 500 ms.  If either sensor has been silent for longer than
    // kDropoutTimeout seconds the monitor raises a fault even if the last
    // message it received was perfectly valid.
    watchdog_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(500),
      [this]() { this->checkDropouts(); }
    );

    // Seed timestamps so the watchdog doesn't trip immediately at startup
    last_imu_time_  = this->now();
    last_pose_time_ = this->now();

    RCLCPP_INFO(this->get_logger(),
      "Safety Monitor initialized (dropout timeout = %.1f s)", kDropoutTimeout);
  }

private:
  // ── Content checks ───────────────────────────────────────────────────────

  void checkPoseSafety(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
  {
    bool ok = true;

    if (std::abs(msg->pose.pose.position.x) > kMaxPositionBound ||
        std::abs(msg->pose.pose.position.y) > kMaxPositionBound)
    {
      RCLCPP_WARN(this->get_logger(), "Position out of bounds! (%.2f, %.2f)",
        msg->pose.pose.position.x, msg->pose.pose.position.y);
      ok = false;
    }

    const double pos_cov = msg->pose.covariance[0];
    if (pos_cov > kMaxCovariance) {
      RCLCPP_WARN(this->get_logger(),
        "Covariance too large (%.3f) — Kalman filter may be diverging!", pos_cov);
      ok = false;
    }

    pose_healthy_ = ok;
  }

  void checkImuSafety(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    const double accel_mag = std::sqrt(
      msg->linear_acceleration.x * msg->linear_acceleration.x +
      msg->linear_acceleration.y * msg->linear_acceleration.y +
      msg->linear_acceleration.z * msg->linear_acceleration.z
    );

    if (accel_mag > kMaxAccelMagnitude) {
      RCLCPP_WARN(this->get_logger(),
        "Unrealistic acceleration! (|a| = %.2f m/s²)", accel_mag);
      imu_healthy_ = false;
    } else {
      imu_healthy_ = true;
    }
  }

  // ── UPGRADE: Dropout watchdog ────────────────────────────────────────────
  // Checks that both sensors are publishing within the timeout window.
  void checkDropouts()
  {
    const auto now = this->now();

    const double imu_age  = (now - last_imu_time_).seconds();
    const double pose_age = (now - last_pose_time_).seconds();

    if (imu_age > kDropoutTimeout) {
      if (imu_healthy_) {   // log only on the first detection
        RCLCPP_ERROR(this->get_logger(),
          "IMU dropout detected! No message for %.1f s", imu_age);
      }
      imu_healthy_ = false;
    }

    if (pose_age > kDropoutTimeout) {
      if (pose_healthy_) {
        RCLCPP_ERROR(this->get_logger(),
          "Fusion pose dropout detected! No message for %.1f s", pose_age);
      }
      pose_healthy_ = false;
    }
  }

  bool isSystemHealthy() const
  {
    return pose_healthy_ && imu_healthy_;
  }

  std::string buildFaultString() const
  {
    std::string msg;
    if (!pose_healthy_) msg += "[POSE fault] ";
    if (!imu_healthy_)  msg += "[IMU fault] ";
    return msg.empty() ? "Unknown fault" : msg;
  }

  // ── Members ───────────────────────────────────────────────────────────────
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr service_;
  rclcpp::TimerBase::SharedPtr watchdog_timer_;

  bool pose_healthy_;
  bool imu_healthy_;

  rclcpp::Time last_imu_time_;
  rclcpp::Time last_pose_time_;

  // NOTE: With SingleThreadedExecutor, callbacks run sequentially so bool
  // access is safe.  For MultiThreadedExecutor, wrap flags in std::atomic<bool>.
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SafetyMonitorNode>());
  rclcpp::shutdown();
  return 0;
}
