#include "gtest/gtest.h"
#include <iostream>
#include "ekf/estimator_continuous_discrete.hpp"

class Estimator_tester : public roscopter::EstimatorContinuousDiscrete{
public:
  Estimator_tester()
{
  roscopter::EstimatorContinuousDiscrete();
};

  Input input;
  Output output;

  auto test_estimate(){
    estimate(input, output) ;
    return output;
  };
  
};

TEST(estimator_test, size_testing)
{
  roscopter::EstimatorContinuousDiscrete estimator = roscopter::EstimatorContinuousDiscrete();

  Eigen::MatrixXf Q = estimator.get_Q_();

  ASSERT_EQ(Q.rows(), 6);
}

TEST(estimator_test, covariance_mult_test)
{
  Estimator_tester estimator = Estimator_tester();

  Eigen::MatrixXf Q = estimator.get_Q_();
  Eigen::MatrixXf Q_g = estimator.get_Q_g_();
  Eigen::MatrixXf P = estimator.get_P_();
  Eigen::MatrixXf xhat = estimator.get_xhat_();
  float Ts = 0.01;
  
  Eigen::VectorXf imu_measurements;
  imu_measurements = Eigen::VectorXf::Zero(6);
  imu_measurements << 1.0, 1.0, 10.0, 0.1, 0.1, 0.1;

  auto dynamics = estimator.get_dynamics();
  auto jacobian = estimator.get_jacobian();
  auto input_jacobian = estimator.get_input_jacobian();

  auto P_prev_rows = P.rows();
  auto P_prev_cols = P.cols();

  estimator.input.gps_n = 100.0;
  estimator.input.gps_e = 100.0;
  estimator.input.static_pres = 100.0;
  estimator.input.gps_Vg = 15.0;
  estimator.input.gps_course = .5;

  estimator.test_estimate();

  P = estimator.get_P_();

  ASSERT_EQ(P_prev_rows, P.rows());
  ASSERT_EQ(P_prev_cols, P.cols());
  
}

TEST(estimator_test, jacobian_test)
{
  roscopter::EstimatorContinuousDiscrete estimator = roscopter::EstimatorContinuousDiscrete();

  auto jacobian = estimator.get_jacobian();

  Eigen::VectorXf measures;
  measures = Eigen::VectorXf::Zero(6);
  measures << 1.0,1.0,1.0,1.0,1.0,1.0;
  
  Eigen::VectorXf state;
  state = Eigen::VectorXf::Zero(12);
  state << 1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,0.0,0.0,0.0;

  auto jac = jacobian(state, measures);

  std::cout << jac << std::endl;
  
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
