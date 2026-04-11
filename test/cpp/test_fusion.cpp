#include <gtest/gtest.h>
#include <Eigen/Dense>
#include <cmath>

// ══════════════════════════════════════════════════════════════════════════════
// EKF helpers — mirror the production fusion_node.cpp so tests are independent
//
// State: x = [x, y, θ, vx_b, vy_b, ω]
//   vx_b / vy_b are BODY-FRAME velocities.
//   World-frame velocity = R(θ) * [vx_b, vy_b]ᵀ
// ══════════════════════════════════════════════════════════════════════════════

static constexpr int N = 6;
using Vec = Eigen::Matrix<double, N, 1>;
using Mat = Eigen::Matrix<double, N, N>;

static double normalizeAngle(double a)
{
  return std::remainder(a, 2.0 * M_PI);
}

// ── Nonlinear state transition f(x, u, dt) ────────────────────────────────
static Vec ekfPredict_f(const Vec & x, const Eigen::Vector2d & u, double dt)
{
  const double theta  = x(2);
  const double vx_b   = x(3);
  const double vy_b   = x(4);
  const double omega  = x(5);
  const double ct     = std::cos(theta);
  const double st     = std::sin(theta);

  Vec x_new;
  x_new(0) = x(0) + (vx_b * ct - vy_b * st) * dt;  // nonlinear in θ
  x_new(1) = x(1) + (vx_b * st + vy_b * ct) * dt;  // nonlinear in θ
  x_new(2) = normalizeAngle(theta + omega * dt);
  x_new(3) = vx_b  + u(0) * dt;
  x_new(4) = vy_b  + u(1) * dt;
  x_new(5) = omega;
  return x_new;
}

// ── Analytic Jacobian F = ∂f/∂x ──────────────────────────────────────────
static Mat ekfJacobian_F(const Vec & x, double dt)
{
  const double theta = x(2);
  const double vx_b  = x(3);
  const double vy_b  = x(4);
  const double ct    = std::cos(theta);
  const double st    = std::sin(theta);

  Mat F = Mat::Identity();
  F(0, 2) = (-vx_b * st - vy_b * ct) * dt;
  F(0, 3) =  ct * dt;
  F(0, 4) = -st * dt;

  F(1, 2) = ( vx_b * ct - vy_b * st) * dt;
  F(1, 3) =  st * dt;
  F(1, 4) =  ct * dt;

  F(2, 5) = dt;
  return F;
}

// ── Full EKF prediction (state + covariance) ──────────────────────────────
static void ekfPredict(Vec & x, Mat & P, const Eigen::Vector2d & u,
                       double dt, const Mat & Q)
{
  const Mat F = ekfJacobian_F(x, dt);    // Jacobian at current state
  x = ekfPredict_f(x, u, dt);            // nonlinear propagation
  P = F * P * F.transpose() + Q;         // linearised covariance propagation
}

// ── Joseph-form Kalman update (works for any measurement dimension M) ─────
template<int M>
static void kalmanUpdate(Vec & x, Mat & P,
                         const Eigen::Matrix<double, M, N> & H,
                         const Eigen::Matrix<double, M, 1> & z,
                         const Eigen::Matrix<double, M, M> & R)
{
  const Eigen::Matrix<double, M, M> S = H * P * H.transpose() + R;
  const Eigen::Matrix<double, N, M> K = P * H.transpose() * S.inverse();
  x = x + K * (z - H * x);
  const Mat IKH = Mat::Identity() - K * H;
  P = IKH * P * IKH.transpose() + K * R * K.transpose();
}

// ══════════════════════════════════════════════════════════════════════════════
// Test 1 — Eigen algebra sanity
// ══════════════════════════════════════════════════════════════════════════════

TEST(EKFTest, EigenMatrixMultiplication)
{
  Eigen::Matrix3d A = Eigen::Matrix3d::Identity();
  Eigen::Vector3d b(1, 2, 3);
  Eigen::Vector3d c = A * b;
  EXPECT_DOUBLE_EQ(c(0), 1.0);
  EXPECT_DOUBLE_EQ(c(1), 2.0);
  EXPECT_DOUBLE_EQ(c(2), 3.0);
}

// ══════════════════════════════════════════════════════════════════════════════
// Test 2 — Angle normalisation
// ══════════════════════════════════════════════════════════════════════════════

TEST(EKFTest, AngleNormalization)
{
  EXPECT_NEAR(normalizeAngle( 4.0),  4.0 - 2*M_PI, 1e-10);
  EXPECT_NEAR(normalizeAngle(-4.0), -4.0 + 2*M_PI, 1e-10);
  EXPECT_NEAR(normalizeAngle( 0.0),  0.0,           1e-10);
  EXPECT_NEAR(normalizeAngle( M_PI), M_PI,          1e-10);
}

// ══════════════════════════════════════════════════════════════════════════════
// Test 3 — NONLINEAR motion model: heading-dependent position update
//
// Robot starts at origin facing East (θ = 0), moving forward at 1 m/s.
// After 1 s:  x ≈ +1 m,  y ≈ 0 m  (correct: East)
//
// Now rotate 90° (θ = π/2, facing North), same forward speed.
// After 1 s:  x ≈ 0 m,  y ≈ +1 m  (correct: North)
//
// A LINEAR KF with constant F would give wrong results for the rotated case
// because it ignores the cos/sin(θ) coupling.
// ══════════════════════════════════════════════════════════════════════════════

TEST(EKFTest, NonlinearMotionModel_HeadingDependentPosition)
{
  const Eigen::Vector2d u_zero = Eigen::Vector2d::Zero();
  const double dt = 1.0;

  // ── Case A: θ = 0 (East), vx_b = 1 m/s ───────────────────────────────
  Vec x_east = Vec::Zero();
  x_east(3) = 1.0;   // vx_b = 1 m/s
  x_east = ekfPredict_f(x_east, u_zero, dt);
  EXPECT_NEAR(x_east(0), 1.0, 1e-9) << "x should be +1 m heading East";
  EXPECT_NEAR(x_east(1), 0.0, 1e-9) << "y should be 0 heading East";

  // ── Case B: θ = π/2 (North), vx_b = 1 m/s ────────────────────────────
  Vec x_north = Vec::Zero();
  x_north(2) = M_PI / 2.0;   // θ = 90°
  x_north(3) = 1.0;           // vx_b = 1 m/s
  x_north = ekfPredict_f(x_north, u_zero, dt);
  EXPECT_NEAR(x_north(0), 0.0, 1e-9) << "x should be 0 m heading North";
  EXPECT_NEAR(x_north(1), 1.0, 1e-9) << "y should be +1 m heading North";

  // ── Case C: θ = π/4 (NE diagonal), vx_b = 1 m/s ──────────────────────
  Vec x_ne = Vec::Zero();
  x_ne(2) = M_PI / 4.0;
  x_ne(3) = 1.0;
  x_ne = ekfPredict_f(x_ne, u_zero, dt);
  EXPECT_NEAR(x_ne(0), std::cos(M_PI/4), 1e-9) << "x NE diagonal";
  EXPECT_NEAR(x_ne(1), std::sin(M_PI/4), 1e-9) << "y NE diagonal";
}

// ══════════════════════════════════════════════════════════════════════════════
// Test 4 — Jacobian matches numerical finite-difference approximation
//
// The analytic Jacobian F = ∂f/∂x should match a numerically computed
// finite-difference approximation at any operating point.  If they disagree
// the EKF covariance propagation will be incorrect.
// ══════════════════════════════════════════════════════════════════════════════

TEST(EKFTest, AnalyticJacobianMatchesNumerical)
{
  Vec x0;
  x0 << 1.0, 2.0, 0.785, 1.5, 0.3, 0.2;   // arbitrary non-trivial state

  const double dt = 0.02;
  const Eigen::Vector2d u(0.1, 0.05);
  const double eps = 1e-5;

  // Analytic Jacobian
  const Mat F_analytic = ekfJacobian_F(x0, dt);

  // Numerical Jacobian via central differences
  Mat F_numeric = Mat::Zero();
  for (int j = 0; j < N; ++j) {
    Vec xp = x0, xm = x0;
    xp(j) += eps;
    xm(j) -= eps;
    const Vec fp = ekfPredict_f(xp, u, dt);
    const Vec fm = ekfPredict_f(xm, u, dt);
    F_numeric.col(j) = (fp - fm) / (2.0 * eps);
  }

  // Columns 0 and 1 (x, y derivatives) should match within 1e-6
  for (int i = 0; i < N; ++i) {
    for (int j = 0; j < N; ++j) {
      EXPECT_NEAR(F_analytic(i, j), F_numeric(i, j), 1e-5)
        << "Jacobian mismatch at (" << i << ", " << j << ")";
    }
  }
}

// ══════════════════════════════════════════════════════════════════════════════
// Test 5 — EKF CONVERGENCE to true position
//
// Run 200 predict + lidar update cycles. The filter should converge to the
// true position (3, 4) within ±0.1 m from an initial estimate of (0, 0).
// ══════════════════════════════════════════════════════════════════════════════

TEST(EKFTest, EKFConvergesToTruePosition)
{
  Vec x = Vec::Zero();
  Mat P = Mat::Identity();

  Mat Q = Mat::Zero();
  Q(0,0) = 0.01; Q(1,1) = 0.01; Q(2,2) = 0.02;
  Q(3,3) = 0.05; Q(4,4) = 0.05; Q(5,5) = 0.03;

  Eigen::Matrix<double, 2, 2> R_lidar;
  R_lidar << 0.02, 0.0, 0.0, 0.02;

  Eigen::Matrix<double, 2, N> H_lidar;
  H_lidar.setZero();
  H_lidar(0, 0) = 1.0;
  H_lidar(1, 1) = 1.0;

  const Eigen::Vector2d true_pos(3.0, 4.0);
  const double dt = 0.02;
  const Eigen::Vector2d u_zero = Eigen::Vector2d::Zero();

  for (int i = 0; i < 200; ++i) {
    ekfPredict(x, P, u_zero, dt, Q);
    kalmanUpdate<2>(x, P, H_lidar, true_pos, R_lidar);
  }

  EXPECT_NEAR(x(0), 3.0, 0.1) << "x estimate did not converge";
  EXPECT_NEAR(x(1), 4.0, 0.1) << "y estimate did not converge";
}

// ══════════════════════════════════════════════════════════════════════════════
// Test 6 — Covariance remains POSITIVE SEMI-DEFINITE after 500 EKF cycles
// ══════════════════════════════════════════════════════════════════════════════

TEST(EKFTest, CovarianceRemainsPositiveSemiDefinite)
{
  Vec x = Vec::Zero();
  x(2) = 0.3;   // non-zero heading to exercise nonlinear Jacobian
  x(3) = 0.5;   // non-zero body-frame velocity

  Mat P = Mat::Identity();
  Mat Q = Mat::Zero();
  Q(0,0) = 0.01; Q(1,1) = 0.01; Q(2,2) = 0.02;
  Q(3,3) = 0.05; Q(4,4) = 0.05; Q(5,5) = 0.03;

  Eigen::Matrix<double, 2, 2> R_lidar;
  R_lidar << 0.02, 0.0, 0.0, 0.02;
  Eigen::Matrix<double, 2, N> H_lidar;
  H_lidar.setZero(); H_lidar(0,0) = 1.0; H_lidar(1,1) = 1.0;

  Eigen::Matrix<double, 1, 1> R_imu;
  R_imu(0,0) = 1e-4;
  Eigen::Matrix<double, 1, N> H_imu;
  H_imu.setZero(); H_imu(0, 5) = 1.0;

  const double dt = 0.02;
  const Eigen::Vector2d u(0.05, 0.0);
  const Eigen::Vector2d meas_pos(1.0, 2.0);
  const Eigen::Matrix<double, 1, 1> meas_gyro = Eigen::Matrix<double, 1, 1>::Constant(0.1);

  for (int i = 0; i < 500; ++i) {
    ekfPredict(x, P, u, dt, Q);
    kalmanUpdate<2>(x, P, H_lidar, meas_pos, R_lidar);
    kalmanUpdate<1>(x, P, H_imu,   meas_gyro, R_imu);
  }

  // Symmetry check
  EXPECT_NEAR((P - P.transpose()).norm(), 0.0, 1e-9) << "Covariance not symmetric";

  // Positive semi-definiteness check
  Eigen::SelfAdjointEigenSolver<Mat> eig(P);
  ASSERT_EQ(eig.info(), Eigen::Success);
  EXPECT_GE(eig.eigenvalues().minCoeff(), -1e-9)
    << "Negative eigenvalue: " << eig.eigenvalues().minCoeff();
}

// ══════════════════════════════════════════════════════════════════════════════
// Test 7 — Control input drives body-frame velocity
// ══════════════════════════════════════════════════════════════════════════════

TEST(EKFTest, ControlInputDrivesBodyFrameVelocity)
{
  Vec x = Vec::Zero();
  Mat P = Mat::Identity();
  Mat Q = Mat::Zero(); Q(3,3) = 0.001; Q(4,4) = 0.001;

  const double dt = 0.02;     // 50 Hz
  const int steps = 50;       // 1 second
  const Eigen::Vector2d u(1.0, 0.0);   // ax_b = 1 m/s²

  for (int i = 0; i < steps; ++i) {
    ekfPredict(x, P, u, dt, Q);
  }

  // After 1 s at 1 m/s² from rest, θ=0: vx_b ≈ 1 m/s, x ≈ 0.5 m
  EXPECT_NEAR(x(3), 1.0, 0.05) << "vx_b did not accumulate from control input";
  EXPECT_NEAR(x(0), 0.5, 0.05) << "x position not consistent with velocity";
}

// ══════════════════════════════════════════════════════════════════════════════

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
