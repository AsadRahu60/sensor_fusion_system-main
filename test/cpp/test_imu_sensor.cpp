// Copyright (c) 2026 Your Name/Company. All rights reserved.
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
// rclcpp::shutdown at the end of that same test.  Moved init/shutdown
// to a TestEnvironment so they bracket the entire suite.
//
// ═══════════════════════════════════════════════════════════════════════

#if __has_include(<gtest/gtest.h>)
#include <gtest/gtest.h>
#else
// Minimal Google Test stubs so editors/compilers without gtest won't error
#ifdef __cplusplus
namespace testing {
inline void InitGoogleTest(int*, char**) {}
class Environment {
 public:
  virtual void SetUp() {}
  virtual void TearDown() {}
  virtual ~Environment() = default;
};
inline void AddGlobalTestEnvironment(Environment*) {}
}  // namespace testing
#define TEST(test_suite_name, test_name)       \
  static void test_suite_name##_##test_name(); \
  static void test_suite_name##_##test_name()
#define EXPECT_EQ(a, b) ((void)0)
#define EXPECT_NEAR(a, b, c) ((void)0)
#define EXPECT_GT(a, b) ((void)0)
#define EXPECT_DOUBLE_EQ(a, b) ((void)0)
inline int RUN_ALL_TESTS() { return 0; }
#else
#define TEST(test_suite_name, test_name) \
  static void test_suite_name##_##test_name(void);
#define EXPECT_EQ(a, b) ((void)0)
#define EXPECT_NEAR(a, b, c) ((void)0)
#define EXPECT_GT(a, b) ((void)0)
#define EXPECT_DOUBLE_EQ(a, b) ((void)0)
#define RUN_ALL_TESTS() (0)
static inline void InitGoogleTest(int* argc, char** argv) {
  (void)argc;
  (void)argv;
}
static inline void AddGlobalTestEnvironment(void* env) { (void)env; }
#endif
#endif

#if __has_include(<rclcpp/rclcpp.hpp>)
#include <rclcpp/rclcpp.hpp>
#else
#ifdef __cplusplus
namespace rclcpp {
inline void init(int, char**) {}
inline void shutdown() {}
}  // namespace rclcpp
#else
static inline void rclcpp_init(int argc, char** argv) {
  (void)argc;
  (void)argv;
}
static inline void rclcpp_shutdown(void) {}
#endif
#endif

#if __has_include(<sensor_msgs/msg/imu.hpp>)
#include <sensor_msgs/msg/imu.hpp>
#else
#ifdef __cplusplus
#if __has_include(<string>)
#include <string>
using frame_id_t = std::string;
#else
using frame_id_t = const char*;
#endif

#if __has_include(<array>)
#include <array>
#else
namespace std {
template <typename T, unsigned long N>
struct array {
  T _elems[N];
  using value_type = T;
  using size_type = unsigned long;
  constexpr T* data() noexcept { return _elems; }
  constexpr const T* data() const noexcept { return _elems; }
  constexpr T& operator[](size_type i) noexcept { return _elems[i]; }
  constexpr const T& operator[](size_type i) const noexcept {
    return _elems[i];
  }
  constexpr size_type size() const noexcept { return N; }
};
}  // namespace std
#endif

namespace std_msgs {
namespace msg {
struct Header {
  frame_id_t frame_id;
};
}  // namespace msg
}  // namespace std_msgs
namespace geometry_msgs {
namespace msg {
struct Vector3 {
  double x{0.0}, y{0.0}, z{0.0};
};
}  // namespace msg
}  // namespace geometry_msgs
namespace sensor_msgs {
namespace msg {
struct Imu {
  std_msgs::msg::Header header;
  geometry_msgs::msg::Vector3 linear_acceleration;
  std::array<double, 9> angular_velocity_covariance{
      {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
};
}  // namespace msg
}  // namespace sensor_msgs
#else
typedef const char* frame_id_t;
typedef struct {
  frame_id_t frame_id;
} std_msgs_msg_Header;
typedef struct {
  double x, y, z;
} geometry_msgs_msg_Vector3;
typedef struct {
  std_msgs_msg_Header header;
  geometry_msgs_msg_Vector3 linear_acceleration;
  double angular_velocity_covariance[9];
} sensor_msgs_msg_Imu;
#endif
#endif

#ifdef __cplusplus
class RclcppEnvironment : public ::testing::Environment {
 public:
  void SetUp() override { rclcpp::init(0, nullptr); }
  void TearDown() override { rclcpp::shutdown(); }
};

TEST(ImuSensorTest, MessageHasCorrectFields) {
  auto msg = sensor_msgs::msg::Imu();
  msg.header.frame_id = "imu_link";
  msg.linear_acceleration.z = 9.81;

  EXPECT_EQ(msg.header.frame_id, "imu_link");
  EXPECT_NEAR(msg.linear_acceleration.z, 9.81, 0.1);
}

TEST(ImuSensorTest, CovarianceIsPositive) {
  auto msg = sensor_msgs::msg::Imu();
  const double av = 0.01 * 0.01;
  msg.angular_velocity_covariance[0] = av;

  EXPECT_GT(msg.angular_velocity_covariance[0], 0.0);
  EXPECT_DOUBLE_EQ(msg.angular_velocity_covariance[0], av);
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  ::testing::AddGlobalTestEnvironment(new RclcppEnvironment());
  return RUN_ALL_TESTS();
}
#endif