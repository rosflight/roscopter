#ifndef ESTIMATOR_CONTINUOUS_DISCRETE_H // FIXME: redefine the header guards when you rename the ekf.
#define ESTIMATOR_CONTINUOUS_DISCRETE_H

#include <math.h>

#include <Eigen/Geometry>
#include <yaml-cpp/yaml.h>
#include "geomag.h"

#include "estimator_ekf.hpp"
#include "estimator_ros.hpp"

namespace roscopter
{

class EstimatorContinuousDiscrete : public EstimatorEKF // FIXME: Rename to something more specific.
{
public:
  EstimatorContinuousDiscrete();
  EstimatorContinuousDiscrete(bool use_params);

protected:
  virtual void estimate(const Input & input, Output & output);
private:

  /**
   * @brief The low pass filter alpha value used on the gyro.
   */
  float alpha_;
  /**
   * @brief The low pass filter alpha value used on the barometer.
   */
  float alpha1_;

  /**
   * @brief The value of the low pass filtered gyroscope measurement.
   */
  float lpf_gyro_x_;
  /**
   * @brief The value of the low pass filtered gyroscope measurement.
   */
  float lpf_gyro_y_;
  /**
   * @brief The value of the low pass filtered gyroscope measurement.
   */
  float lpf_gyro_z_;
  /**
   * @brief The value of the low pass filtered static pressure sensor (barometer).
   */
  float lpf_static_;
  
  /**
   * @brief This function calculates the derivatives of the state. This is dictated by the dynamics of
   * the system.
   *
   * @param state The state of the system. 
   * @param inputs The inputs to the estimator. Can be something like IMU measurements.
   */
  Eigen::VectorXf multirotor_dynamics(const Eigen::VectorXf& state, const Eigen::VectorXf& inputs);
  /**
   * @brief This is a reference to the multirotor_dynamics function, this is created by the std::bind.
   * This offers a minimum time penalty when passed into a function.
   */
  std::function<Eigen::VectorXf(const Eigen::VectorXf, const Eigen::VectorXf)> multirotor_dynamics_model;

  /**
   * @brief Calculates the jacobian of the system dynamics given the current states and inputs.
   *
   * @param state The state of the system.
   * @param inputs The inputs to the estimator, something like IMU measurements.
   */
  Eigen::MatrixXf multirotor_jacobian(const Eigen::VectorXf& state, const Eigen::VectorXf& inputs);
  /**
   * @brief This is a reference to the multirotor_jacobian function, this is created by the std::bind.
   * This offers a minimum time penalty when passed into a function.
   */
  std::function<Eigen::MatrixXf(const Eigen::VectorXf&, const Eigen::VectorXf&)> multirotor_jacobian_model;

  /**
   * @brief Calculates the jacobian of the inputs to the estimator.
   *
   * @param state The state of the dynamic system.
   * @param inputs Inputs to the estimator.
   */
  Eigen::MatrixXf multirotor_input_jacobian(const Eigen::VectorXf& state, const Eigen::VectorXf& inputs);
  /**
   * @brief This is a reference to the multirotor_input_jacobian function. This incurs minimum time cost
   * when passing into a function.
   */
  std::function<Eigen::MatrixXf(const Eigen::VectorXf&, const Eigen::VectorXf&)> multirotor_input_jacobian_model;

  /**
   * @brief Calculates prediction of the measurements using a model of the sensors.
   *
   * @param state The state of the dynamic system.
   * @param input Inputs to the measurement prediction. Essentially information necessary to the prediction,
   * but is not contained in the state.
   */
  Eigen::VectorXf multirotor_measurement_prediction(const Eigen::VectorXf& state, const Eigen::VectorXf& input);
  /**
   * @brief This is a reference to the multirotor_measurement_prediction function. This incurs minimum time cost
   * when passing into a function.
   */
  std::function<Eigen::VectorXf(const Eigen::VectorXf, const Eigen::VectorXf)> multirotor_measurement_model;
  
  /**
   * @brief Calculates measurement prediction for the fast sensors.
   *
   * @param state The state of the system.
   * @param input Inputs to the measurement prediction. Essentially information necessary to the prediction,
   * but is not contained in the state.
   */
  Eigen::VectorXf multirotor_fast_measurement_prediction(const Eigen::VectorXf& state, const Eigen::VectorXf& input);
  /**
   * @brief This is a reference to the multirotor_fast_measurement_prediction function. This incurs minimum time cost
   * when passing into a function.
   */
  std::function<Eigen::VectorXf(const Eigen::VectorXf, const Eigen::VectorXf)> multirotor_fast_measurement_model;
  
  /**
   * @brief Calculates the jacobian of the measurement model for the fast sensors.
   *
   * @param state State of the system.
   * @param input Any inputs not in the state needed for the system.
   */
  Eigen::MatrixXf multirotor_fast_measurement_jacobian(const Eigen::VectorXf& state, const Eigen::VectorXf& input);
  /**
   * @brief This is a reference to the multirotor_fast_measurement_jacobian function. This incurs minimum time cost
   * when passing into a function.
   */
  std::function<Eigen::MatrixXf(const Eigen::VectorXf, const Eigen::VectorXf)> multirotor_fast_measurement_jacobian_model;

  /**
   * @brief Calculates the measurement jacobian for the measurement model.
   *
   * @param state State of the system.
   * @param input Any necessary inputs not included in the state.
   */
  Eigen::MatrixXf multirotor_measurement_jacobian(const Eigen::VectorXf& state, const Eigen::VectorXf& input);
  /**
   * @brief This is a reference to the multirotor_measurement_jacobian function. This incurs minimum time cost
   * when passing into a function.
   */
  std::function<Eigen::MatrixXf(const Eigen::VectorXf, const Eigen::VectorXf)> multirotor_measurement_jacobian_model;

  /**
   * @brief The state of the system.
   */
  Eigen::VectorXf xhat_; // 12
  /**
   * @brief The covariance of the estimate.
   */
  Eigen::MatrixXf P_;    // 12x12

  /**
   * @brief The process noise for the GPS measurements.
   */
  Eigen::MatrixXf Q_; // 12x12
  /**
   * @brief The process noise from the inputs to the estimator.
   */
  Eigen::MatrixXf Q_g_; // 6x6
  /**
   * @brief The sensor noises for the slower sensors.
   */
  Eigen::MatrixXf R_; // 5x5
  /**
   * @brief The sensor noises for the faster sensors.
   */
  Eigen::MatrixXf R_fast; // 4x4
  
  /**
   * @brief The inclination of the magnetic field at the current location.
   */
  double inclination_;
  /**
   * @brief The declination of the magnetic field at the current location.
   */
  double declination_;

  bool mag_init_;

  void prediction_step(const Input& input);

  void fast_measurement_update_step(const Input& input);

  void gnss_measurement_update_step(const Input& input);

  void update_magnetometer_model(const Input& input);

  Eigen::Vector3f calculate_inertial_magnetic_field(const Eigen::VectorXf& state, const float& declination, const float& inclination);
  Eigen::Matrix3f R(const Eigen::Vector3f& Theta);
  Eigen::Matrix3f S(const Eigen::Vector3f& Theta);
  Eigen::Matrix3f del_R_Theta_y_accel_del_Theta(const Eigen::Vector3f& Theta, const Eigen::Vector3f& accel);
  Eigen::Matrix3f del_S_Theta_del_Theta(const Eigen::Vector3f& Theta, const Eigen::Vector3f& biases, const Eigen::Vector3f& gyro);
  Eigen::Matrix3f del_R_Theta_y_mag_del_Theta(const Eigen::Vector3f& Theta, const Eigen::Vector3f& inertial_mag);

  // TODO: not used
  // ASK: What should we do long term?
  /**
   * @brief The threshold where an gps update is worth updating.
   */
  float gate_threshold_ = 9.21; // chi2(q = .01, df = 2)

  /**
   * @brief This function binds references to the functions used in the ekf.
   */
  void bind_functions();

  /**
   * @brief This declares each parameter as a parameter so that the ROS2 parameter system can recognize each parameter.
   * It also sets the default parameter, which will then be overridden by a launch script.
   */
  void declare_parameters();

  /**
   * @brief Initializes some variables that depend on ROS2 parameters
  */
  void update_measurement_model_parameters();

  /**
   * @brief Initializes the process noise matrices with the ROS2 parameters
   */
  void initialize_process_noises();

  /**
   * @brief Initializes the state covariance matrix with the ROS2 parameters
   */
  void initialize_state_covariances();

  void check_estimate(const Input& input);
}; 

} // namespace roscopter

#endif // ESTIMATOR_CONTINUOUS_DISCRETE_H
