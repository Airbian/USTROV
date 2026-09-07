#include <gtest/gtest.h>
#include <ustrov_state_estimation/ekf.h>
#include <ustrov_state_estimation/sensor_sim/sensor_sim.h>

#include <cmath>

class EkfBasicTest : public ::testing::Test {
 public:
  EkfBasicTest()
      : ::testing::Test(), ekf_{std::make_shared<Ekf>()}, sensor_sim_(ekf_) {};

  void SetUp() override {
    ekf_->Init(0);
    sensor_sim_.RunSeconds(init_period_);
  }

  void TearDown() override {}

  std::shared_ptr<Ekf> ekf_{nullptr};
  SensorSim sensor_sim_;
  const double init_period_{4.0};
};

TEST_F(EkfBasicTest, tiltAlign) { EXPECT_TRUE(ekf_->AttitudeValid()); }

TEST_F(EkfBasicTest, stationaryStateRemainsBounded) {
  sensor_sim_.RunSeconds(2.0);

  EXPECT_NEAR(ekf_->Orientation().norm(), 1.0, 1e-9);
  EXPECT_LT(ekf_->Velocity().norm(), 0.05);
  EXPECT_LT(ekf_->Position().norm(), 0.05);
}

TEST_F(EkfBasicTest, integratesConstantYawRate) {
  sensor_sim_.vision_.Stop();
  const double yaw_rate = 0.5;
  const double duration = 2.0;
  const double yaw_before =
      std::atan2(2.0 * (ekf_->Orientation().w() * ekf_->Orientation().z() +
                       ekf_->Orientation().x() * ekf_->Orientation().y()),
                 1.0 - 2.0 * (std::pow(ekf_->Orientation().y(), 2) +
                              std::pow(ekf_->Orientation().z(), 2)));

  sensor_sim_.imu_.SetData(Eigen::Vector3d{0.0, 0.0, kGravity},
                           Eigen::Vector3d{0.0, 0.0, yaw_rate});
  sensor_sim_.RunSeconds(duration);

  const auto q = ekf_->Orientation();
  const double yaw_after =
      std::atan2(2.0 * (q.w() * q.z() + q.x() * q.y()),
                 1.0 - 2.0 * (std::pow(q.y(), 2) + std::pow(q.z(), 2)));
  const double yaw_change =
      std::atan2(std::sin(yaw_after - yaw_before),
                 std::cos(yaw_after - yaw_before));
  EXPECT_NEAR(yaw_change, yaw_rate * duration, 0.08);
}

TEST_F(EkfBasicTest, followsUpdatedVisionPosition) {
  const Eigen::Vector3d target{0.4, 0.8, 0.0};
  sensor_sim_.vision_.SetPosition(target);
  sensor_sim_.vision_.SetPositionCovariance(
      Eigen::Vector3d::Constant(0.01));
  sensor_sim_.RunSeconds(6.0);

  EXPECT_NEAR(ekf_->Position().x(), target.x(), 0.1);
  EXPECT_NEAR(ekf_->Position().y(), target.y(), 0.1);
}
