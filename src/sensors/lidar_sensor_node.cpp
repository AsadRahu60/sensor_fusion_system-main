#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <memory>
#include <random>
#include <cmath>

using namespace std::chrono_literals;

class LidarSensorNode : public rclcpp::Node
{
public:
  LidarSensorNode()
  : Node("lidar_sensor_node"),
    gen_(std::random_device{}()),
    noise_dist_(0.0, 0.02)
  {
    this->declare_parameter("scan_rate", 10.0);
    this->declare_parameter("min_angle", -M_PI);
    this->declare_parameter("max_angle", M_PI);
    this->declare_parameter("angle_increment", M_PI / 180.0);
    this->declare_parameter("range_min", 0.1);
    this->declare_parameter("range_max", 30.0);
    this->declare_parameter("add_noise", true);
    
    rate_ = this->get_parameter("scan_rate").as_double();
    min_angle_ = this->get_parameter("min_angle").as_double();
    max_angle_ = this->get_parameter("max_angle").as_double();
    angle_increment_ = this->get_parameter("angle_increment").as_double();
    range_min_ = this->get_parameter("range_min").as_double();
    range_max_ = this->get_parameter("range_max").as_double();
    add_noise_ = this->get_parameter("add_noise").as_bool();
    
    num_rays_ = static_cast<size_t>((max_angle_ - min_angle_) / angle_increment_);
    
    publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>("lidar/scan", 10);
    
    timer_ = this->create_wall_timer(
      std::chrono::duration<double>(1.0 / rate_),
      [this]() { this->publishScan(); }
    );
    
    RCLCPP_INFO(this->get_logger(), "Lidar Sensor Node initialized at %.1f Hz (%zu rays)", rate_, num_rays_);
  }

private:
  void publishScan()
  {
    auto msg = std::make_shared<sensor_msgs::msg::LaserScan>();
    
    msg->header.stamp = this->now();
    msg->header.frame_id = "lidar_link";
    
    msg->angle_min = min_angle_;
    msg->angle_max = max_angle_;
    msg->angle_increment = angle_increment_;
    msg->time_increment = 0.0;
    msg->scan_time = 1.0 / rate_;   // correctly reflects actual scan rate
    msg->range_min = range_min_;
    msg->range_max = range_max_;
    
    msg->ranges.resize(num_rays_);
    msg->intensities.resize(num_rays_);
    
    for (size_t i = 0; i < num_rays_; ++i) {
      double angle = min_angle_ + i * angle_increment_;
      double base_distance = 5.0 + 2.0 * std::sin(angle * 3.0);
      msg->ranges[i] = base_distance + addNoise();
      msg->intensities[i] = 100.0 + 50.0 * std::cos(angle * 2.0);
    }
    
    publisher_->publish(*msg);
  }
  
  double addNoise()
  {
    return add_noise_ ? noise_dist_(gen_) : 0.0;
  }
  
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::mt19937 gen_;
  std::normal_distribution<double> noise_dist_;

  double rate_;
  double min_angle_;
  double max_angle_;
  double angle_increment_;
  double range_min_;
  double range_max_;
  bool add_noise_;
  size_t num_rays_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LidarSensorNode>());
  rclcpp::shutdown();
  return 0;
}
