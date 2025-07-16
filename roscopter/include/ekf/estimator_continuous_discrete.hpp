#ifndef ESTIMATOR_CONTINUOUS_DISCRETE_H // FIXME: redefine the header guards when you rename the ekf.
#define ESTIMATOR_CONTINUOUS_DISCRETE_H

#include <math.h>

#include <Eigen/Geometry>
#include <yaml-cpp/yaml.h>
#include "geomag.h"

#include <cmath>
#include <functional>
#include <tuple>

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
  virtual void estimate(const Input & input, Output & output) override;
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
  DynamicModelFuncRef multirotor_dynamics_model;

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
  JacobianFuncRef multirotor_jacobian_model;

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
  JacobianFuncRef multirotor_input_jacobian_model;

  /**
   * @brief Calculates prediction of the measurements using a model of the sensors.
   *
   * @param state The state of the dynamic system.
   * @param input Inputs to the measurement prediction. Essentially information necessary to the prediction,
   * but is not contained in the state.
   */
  Eigen::VectorXf multirotor_gnss_measurement_prediction(const Eigen::VectorXf& state, const Eigen::VectorXf& input);
  /**
   * @brief This is a reference to the multirotor_measurement_prediction function. This incurs minimum time cost
   * when passing into a function.
   */
   MeasurementModelFuncRef multirotor_gnss_measurement_model;
  
  /**
   * @brief Calculates the measurement jacobian for the measurement model.
   *
   * @param state State of the system.
   * @param input Any necessary inputs not included in the state.
   */
  Eigen::MatrixXf multirotor_gnss_measurement_jacobian(const Eigen::VectorXf& state, const Eigen::VectorXf& input);
  /**
   * @brief This is a reference to the multirotor_measurement_jacobian function. This incurs minimum time cost
   * when passing into a function.
   */
  JacobianFuncRef multirotor_gnss_measurement_jacobian_model;
  
  // TODO: Fill in DOXYGEN
  Eigen::MatrixXf multirotor_gnss_measurement_sensor_noise();

  Eigen::Matrix<float, 3,3> del_R_Theta_T_g_del_Theta(const Eigen::Vector3f& Theta, const double& gravity);

  Eigen::Matrix<float, 3,3> del_R_Theta_v_del_Theta(const Eigen::Vector3f& Theta, const Eigen::Vector3f& vels);

  double gravity_ = 9.81;
  
  SensorNoiseFuncRef multirotor_gnss_measurement_sensor_noise_model;
  
  /**
   * @brief Calculates measurement prediction for the mag.
   *
   * @param state The state of the system.
   * @param input Inputs to the measurement prediction. Essentially information necessary to the prediction,
   * but is not contained in the state.
   */
  Eigen::VectorXf multirotor_mag_measurement_prediction(const Eigen::VectorXf& state, const Eigen::VectorXf& input);
  /**
   * @brief This is a reference to the multirotor_mag_measurement_prediction function. This incurs minimum time cost
   * when passing into a function.
   */
  MeasurementModelFuncRef multirotor_mag_measurement_model;
  
  /**
   * @brief Calculates the jacobian of the measurement model for the magnetometer.
   *
   * @param state State of the system.
   * @param input Any inputs not in the state needed for the system.
   */
  Eigen::MatrixXf multirotor_mag_measurement_jacobian(const Eigen::VectorXf& state, const Eigen::VectorXf& input);
  /**
   * @brief This is a reference to the multirotor_mag_measurement_jacobian function. This incurs minimum time cost
   * when passing into a function.
   */
  JacobianFuncRef multirotor_mag_measurement_jacobian_model;
  
  // TODO: Fill in DOXYGEN
  Eigen::MatrixXf multirotor_mag_measurement_sensor_noise();
  
  SensorNoiseFuncRef multirotor_mag_measurement_sensor_noise_model;
  
  /**
   * @brief Calculates measurement prediction for the baro.
   *
   * @param state The state of the system.
   * @param input Inputs to the measurement prediction. Essentially information necessary to the prediction,
   * but is not contained in the state.
   */
  Eigen::VectorXf multirotor_baro_measurement_prediction(const Eigen::VectorXf& state, const Eigen::VectorXf& input);
  /**
   * @brief This is a reference to the multirotor_baro_measurement_prediction function. This incurs minimum time cost
   * when passing into a function.
   */
  MeasurementModelFuncRef multirotor_baro_measurement_model;
  
  /**
   * @brief Calculates the jacobian of the measurement model for the barometer.
   *
   * @param state State of the system.
   * @param input Any inputs not in the state needed for the system.
   */
  Eigen::MatrixXf multirotor_baro_measurement_jacobian(const Eigen::VectorXf& state, const Eigen::VectorXf& input);
  /**
   * @brief This is a reference to the multirotor_baro_measurement_jacobian function. This incurs minimum time cost
   * when passing into a function.
   */
  JacobianFuncRef multirotor_baro_measurement_jacobian_model;
  
  Eigen::MatrixXf multirotor_baro_measurement_sensor_noise();
  
  SensorNoiseFuncRef multirotor_baro_measurement_sensor_noise_model;

  static constexpr int num_states = 12;

  /**
   * @brief The state of the system.
   */
  Eigen::Vector<float, num_states> xhat_;
  /**
   * @brief The covariance of the estimate.
   */
  Eigen::Matrix<float, num_states, num_states> P_;

  /**
   * @brief The process noise for state propagation.
   */
  Eigen::Matrix<float, num_states, num_states> Q_;
  
  /**
   * @brief There are 6 estimator inputs by default. accel_x, accel_y, accel_z, omega_x, omega_y and omega_z.
   */
  static constexpr int num_estimator_inputs = 6;

  /**
   * @brief The process noise from the inputs to the estimator, accelerations (3) and angular velocities (3).
   * The first 3 rows are for the accels, and the second 3 for the angular velocities.
   */
  Eigen::Matrix<float, num_estimator_inputs, num_estimator_inputs> Q_inputs_;

  /**
   * @brief There are 6 gnss measurements by default. Lat, lon, alt, v_n, v_e and v_d.
   */
  static constexpr int num_gnss_measurements = 6;

  /**
   * @brief The sensor noises for the GNSS measurements. The first three rows are for the positional measurements.
   * The last three rows are for the velocity measurements.
   */
  Eigen::Matrix<float, num_gnss_measurements, num_gnss_measurements> R_gnss_;
  
  /**
   * @brief There are 3 mag measurements by default. m_x, m_y and m_z.
   */
  static constexpr int num_mag_measurements = 3;

  /**
   * @brief The sensor noises for the magnetometer.
   */
  Eigen::Matrix<float, num_mag_measurements, num_mag_measurements> R_mag_;
  
  /**
   * @brief There is one barometer measurement by default. P (pressure).
   */
  static constexpr int num_baro_measurements = 1;
  
  /**
   * @brief The sensor noises for the barometer.
   */
  Eigen::Matrix<float, num_baro_measurements, num_baro_measurements> R_baro_;
  
  /**
   * @brief The calculated inclination of the magnetic field at the current location.
   */
  double inclination_;

  /**
   * @brief The calculated declination of the magnetic field at the current location.
   */
  double declination_;

  /**
   * @brief Run the prediction step of the estimation algorithm.
   */
  void prediction_step(const Input& input);

  void mag_measurement_update_step(const Input& input);

  void baro_measurement_update_step(const Input& input);

  void gnss_measurement_update_step(const Input& input);

  void calc_mag_field_properties(const Input& input);

  Eigen::Vector3f calculate_inertial_magnetic_field(const float& declination, const float& inclination);
  Eigen::Matrix3f R(const Eigen::Vector3f& Theta);
  Eigen::Matrix3f S(const Eigen::Vector3f& Theta);
  Eigen::Matrix3f del_R_Theta_y_accel_del_Theta(const Eigen::Vector3f& Theta, const Eigen::Vector3f& accel);
  Eigen::Matrix3f del_S_Theta_del_Theta(const Eigen::Vector3f& Theta, const Eigen::Vector3f& biases, const Eigen::Vector3f& gyro);
  Eigen::Matrix3f del_R_Theta_y_mag_del_Theta(const Eigen::Vector3f& Theta, const Eigen::Vector3f& inertial_mag);
  Eigen::Matrix3f del_R_Theta_T_y_mag_del_Theta(const Eigen::Vector3f& Theta, const Eigen::Vector3f& mag);
  Eigen::Matrix<float, 3,4> del_R_Theta_inc_y_mag_del_Theta(const Eigen::Vector3f& Theta, const double& inclination,  const double& declination);

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
