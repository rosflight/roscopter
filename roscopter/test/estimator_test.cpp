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

  auto G = input_jacobian(xhat, imu_measurements);

  auto result = G * Q_g * G.transpose();
  
  ASSERT_EQ(result.rows(), P.rows());
  ASSERT_EQ(result.cols(), P.cols());
  
}

TEST(estimator_test, jacobian_test)
{
  roscopter::EstimatorContinuousDiscrete estimator = roscopter::EstimatorContinuousDiscrete();

  auto jacobian = estimator.get_jacobian();
  auto input_jacobian = estimator.get_input_jacobian();

  Eigen::VectorXf measures;
  measures = Eigen::VectorXf::Zero(6);
  measures << 1.0,1.1,1.2,1.0,1.0,1.0;
  
  Eigen::VectorXf state;
  state = Eigen::VectorXf::Zero(12);
  state << 1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,0.0,0.0,0.0;

  auto jac = jacobian(state, measures);
  auto input_jac = input_jacobian(state, measures);
}

TEST(estimator_test, input_output_test)
{
  Estimator_tester estimator = Estimator_tester();

  Eigen::MatrixXf Q = estimator.get_Q_();
  Eigen::MatrixXf Q_g = estimator.get_Q_g_();
  Eigen::MatrixXf P = estimator.get_P_();
  Eigen::MatrixXf xhat = estimator.get_xhat_();
  float Ts = 0.01;
  
  Eigen::VectorXf imu_measurements;
  imu_measurements = Eigen::VectorXf::Zero(6);
  imu_measurements << .10, 1.0, 10.0, 0.1, 0.1, 0.1;

  auto dynamics = estimator.get_dynamics();
  auto jacobian = estimator.get_jacobian();
  auto input_jacobian = estimator.get_input_jacobian();

  auto P_prev_rows = P.rows();
  auto P_prev_cols = P.cols();

  estimator.input.gps_n = 1.0;
  estimator.input.gps_e = 1.0;
  estimator.input.static_pres = 1.0;
  estimator.input.gps_Vg = 15.0;
  estimator.input.gps_course = .5;
  estimator.input.gyro_x = 0.1;
  estimator.input.gyro_y = 0.1;
  estimator.input.gyro_z = 0.1;
  estimator.input.accel_x = 0.1;
  estimator.input.accel_y = 0.1;
  estimator.input.accel_z = 0.1;
  
  std::cout << "BEFORE:" << std::endl;
  std::cout << estimator.get_P_() << std::endl;

  estimator.test_estimate();

  P = estimator.get_P_();
  
  std::cout << "AFTER:" << std::endl;
  std::cout << estimator.get_P_() << std::endl;

  // std::cout << "output:\n";
  // std::cout << "pn: " << estimator.output.pn << "\n";
  // std::cout << "pe: " << estimator.output.pe << "\n";
  // std::cout << "pd: " << estimator.output.pd << "\n";
  // std::cout << "vn: " << estimator.output.vn << "\n";
  // std::cout << "ve: " << estimator.output.ve << "\n";
  // std::cout << "vd: " << estimator.output.vd << "\n";
  // std::cout << "phi: " << estimator.output.phi << "\n";
  // std::cout << "theta: " << estimator.output.theta << "\n";
  // std::cout << "psi: " << estimator.output.psi << "\n";
  // std::cout << "bx: " << estimator.output.bx << "\n";
  // std::cout << "by: " << estimator.output.by << "\n";
  // std::cout << "bz: " << estimator.output.bz << "\n";
  // std::cout << "p: " << estimator.output.p << "\n";
  // std::cout << "q: " << estimator.output.q << "\n";
  // std::cout << "r: " << estimator.output.r << "\n";
  // std::cout << "Vg: " << estimator.output.Vg << "\n";

  ASSERT_EQ(P_prev_rows, P.rows());
  ASSERT_EQ(P_prev_cols, P.cols());
}

// TODO: create test with inputs and outputs, and just make sure outputs make sense.

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
