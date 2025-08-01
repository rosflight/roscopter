#include "ekf/estimator_continuous_discrete.hpp"
#include "ekf/estimator_ros.hpp"
#include "ekf/geomag.h"

namespace roscopter
{

// ======== UTILITY FUNCTIONS ========
float radians(float degrees) { return M_PI * degrees / 180.0; }

double wrap_within_180(double fixed_heading, double wrapped_heading)
{
  // wrapped_heading - number_of_times_to_wrap * 2pi
  return wrapped_heading - floor((wrapped_heading - fixed_heading) / (2 * M_PI) + 0.5) * 2 * M_PI;
}

Eigen::Matrix3f skew_matrix(Eigen::Vector3f vec)
{

  Eigen::Matrix3f skew_symmetric_matrix;

  skew_symmetric_matrix << 0.0, -vec(2), vec(1),
                           vec(2), 0.0, -vec(0),
                           -vec(1), vec(1), 0.0;

  return skew_symmetric_matrix;
}

// ======== CONSTRUCTOR ========
EstimatorContinuousDiscrete::EstimatorContinuousDiscrete()
    : EstimatorEKF()
    , xhat_(Eigen::Vector<float, num_states>::Zero())
    , P_(Eigen::Matrix<float, num_states, num_states>::Identity())
    , Q_(Eigen::Matrix<float, num_states, num_states>::Identity())
    , Q_inputs_(Eigen::Matrix<float, num_estimator_inputs, num_estimator_inputs>::Identity())
    , R_gnss_(Eigen::Matrix<float, num_gnss_measurements, num_gnss_measurements>::Zero())
    , R_mag_(Eigen::Matrix<float,num_mag_measurements, num_mag_measurements>::Zero())
    , R_baro_(Eigen::Matrix<float,num_baro_measurements, num_baro_measurements>::Zero())
{
  // This binds the various functions for the measurement and dynamic models and their jacobians,
  // to a reference that is efficient to pass to the functions used to do the estimation.
  bind_functions(); 

  load_magnetic_model();
  // Indicates whether inclination_ and declination_ have been updated.
  mag_init_ = false;
  
  // Indicates whether the state has been initalized.
  state_init_ = false;

  // Initialize the baro low pass filter.
  lpf_static_ = 0.0;

  // Declare and set parameters with the ROS2 system
  declare_parameters();
  params_.set_parameters();

  // Initialize estimator from ROS2 parameters
  initialize_process_noises();
  initialize_state_covariances();
  update_measurement_model_parameters();
}

// ======== INIT STATE ========
void EstimatorContinuousDiscrete::init_state(const Input & input)
{
  if (mag_init_) {
    float heading = atan2f(input.mag_y, input.mag_x);
    heading -= declination_;
    xhat_(8) = heading;
    state_init_ = true;
    return;
  }
}

// ======== MAIN ESTIMATION LOOP ========
void EstimatorContinuousDiscrete::estimate(const Input & input, Output & output)
{
  if (!state_init_) {
    calc_mag_field_properties(input); // Finds inclination_ and declination_.
    init_state(input);
    return; 
  }

  if (is_parameter_changed()) {
    update_estimation_params();
  }

  prediction_step(input);
  
  // Measurement updates.
  mag_measurement_update_step(input);
  baro_measurement_update_step(input);
  gnss_measurement_update_step(input);

  check_estimate(input);
  
  // Low pass filter gyros to estimate angular rates ASK: Should we lpf this?
  lpf_gyro_x_ = alpha_gyro_ * lpf_gyro_x_ + (1 - alpha_gyro_) * input.gyro_x;
  lpf_gyro_y_ = alpha_gyro_ * lpf_gyro_y_ + (1 - alpha_gyro_) * input.gyro_y;
  lpf_gyro_z_ = alpha_gyro_ * lpf_gyro_z_ + (1 - alpha_gyro_) * input.gyro_z;
  
  Eigen::Vector3f mag;
  mag << input.mag_x, input.mag_y, input.mag_z;
  mag /= mag.norm();
  
  // Stuff the output
  output.pn = xhat_(0);
  output.pe = xhat_(1);
  output.pd = xhat_(2);
  output.vn = xhat_(3);
  output.ve = xhat_(4);
  output.vd = xhat_(5);
  output.p = lpf_gyro_x_ - xhat_(9);
  output.q = lpf_gyro_y_ - xhat_(10);
  output.r = lpf_gyro_z_ - xhat_(11);
  output.phi = xhat_(6);
  output.theta = xhat_(7);
  output.psi = xhat_(8);
  output.bx = xhat_(9);
  output.by = xhat_(10);
  output.bz = xhat_(11);

  // Convert Euler angles to quaternion
  double psi2 = output.psi/2;
  double theta2 = output.theta/2;
  double phi2 = output.phi/2;
  double qw = cosf(psi2)*cosf(theta2)*cosf(phi2) + sinf(psi2)*sinf(theta2)*sinf(phi2);
  double qx = cosf(psi2)*cosf(theta2)*sinf(phi2) - sinf(psi2)*sinf(theta2)*cosf(phi2);
  double qy = cosf(psi2)*sinf(theta2)*cosf(phi2) + sinf(psi2)*cosf(theta2)*sinf(phi2);
  double qz = sinf(psi2)*cosf(theta2)*cosf(phi2) - cosf(psi2)*sinf(theta2)*sinf(phi2);
  output.quat = Eigen::Quaternionf(qw, qx, qy, qz);
  output.quat_valid = true;
}

// ======== ESTIMATION LOOP STEPS ========
void EstimatorContinuousDiscrete::prediction_step(const Input& input)
{
  double frequency = params_.get_double("estimator_update_frequency");
  double Ts = 1.0 / frequency;

  Eigen::Vector<float, num_estimator_inputs> imu_measurements; // These are the input to the propagation.
  imu_measurements << input.accel_x, input.accel_y, input.accel_z,
                      input.gyro_x, input.gyro_y, input.gyro_z;

  std::tie(P_, xhat_) = propagate_model(xhat_, multirotor_dynamics_model, multirotor_jacobian_model,
                                        imu_measurements, multirotor_input_jacobian_model, P_, Q_,
                                        Q_inputs_, Ts);

  // Wrap RPY estimates.
  xhat_(6) = wrap_within_180(0.0, xhat_(6));
  xhat_(7) = wrap_within_180(0.0, xhat_(7));
  xhat_(8) = wrap_within_180(0.0, xhat_(8));
}

void EstimatorContinuousDiscrete::mag_measurement_update_step(const Input& input)
{
  // Only update when have new mag and the magnetometer models have been found.
  if (!new_mag_ || !mag_init_) {
    return;
  }

  bool convert_to_gauss = params_.get_bool("convert_to_gauss");

  Eigen::Vector3f mag_readings;
  mag_readings << input.mag_x, input.mag_y, input.mag_z;
  if (convert_to_gauss) {
    mag_readings *= 10'000.;
  }
  
  Eigen::Vector<float, num_mag_measurements> y_mag;
  y_mag << mag_readings/mag_readings.norm(); // TODO: fix this normailization. This should be a filtered value or based on geomag.

  Eigen::Vector<float, 2> mag_info;
  mag_info << radians(declination_), radians(inclination_);
  
  // Use gammas to enforce consider states, or partial consider states. See Parial-Update Schmidt-Kalman Filter, Kevin Brink 2017.
  Eigen::Vector<float, num_states> gammas = Eigen::Vector<float, num_states>::Zero();

  std::tie(P_, xhat_) = partial_measurement_update(xhat_, mag_info, multirotor_mag_measurement_model, y_mag, multirotor_mag_measurement_jacobian_model, multirotor_mag_measurement_sensor_noise_model, P_, gammas);

  new_mag_ = false;
}

void EstimatorContinuousDiscrete::baro_measurement_update_step(const Input& input) {
  
  // Only update when have new baro.
  if (!new_baro_) {
    return;
  }

  lpf_static_ = alpha_baro_ * lpf_static_ + (1 - alpha_baro_) * input.static_pres;

  Eigen::Vector<float, num_baro_measurements> y_baro;
  y_baro << lpf_static_;

  Eigen::Vector<float, 1> _;

  std::tie(P_, xhat_) = measurement_update(xhat_, _, multirotor_baro_measurement_model, y_baro,
                                             multirotor_baro_measurement_jacobian_model, multirotor_baro_measurement_sensor_noise_model, P_);
  new_baro_ = false;
}

void EstimatorContinuousDiscrete::gnss_measurement_update_step(const Input& input)
{
  Eigen::Vector<float, 1> _; // This is used when no inputs are needed.

  // Only update if new GPS information is available.
  if (input.gps_new) {
    //wrap course measurement
    float gps_course = fmodf(input.gps_course, radians(360.0f));
    gps_course = wrap_within_180(xhat_(3), gps_course);
    
    // Measurements for the positional states.
    Eigen::Vector<float, num_gnss_measurements> y_gps;
    y_gps << input.gps_n, input.gps_e, -input.gps_h, input.gps_vn, input.gps_ve, input.gps_vd;
  
    std::tie(P_, xhat_) = measurement_update(xhat_, _, multirotor_gnss_measurement_model, y_gps,
                                             multirotor_gnss_measurement_jacobian_model, multirotor_gnss_measurement_sensor_noise_model, P_);
  }
}

// ======== PREDICITON STEP EQUATIONS ========
// These are passed by reference to the predition step.
Eigen::VectorXf EstimatorContinuousDiscrete::multirotor_dynamics(const Eigen::VectorXf& state, const Eigen::VectorXf& inputs)
{

  double gravity = params_.get_double("gravity");
  
  // Unpack states and inputs.
  Eigen::Vector3f vels = state.block<3,1>(3,0);
  Eigen::Vector3f Theta = state.block<3,1>(6,0); // Theta is the vector of the euler angles.
  Eigen::Vector3f biases = state.block<3,1>(9,0);

  Eigen::Vector3f y_accel = inputs.block<3,1>(0,0);
  Eigen::Vector3f y_gyro = inputs.block<3,1>(3,0);
  
  // Calculate the derivatives of the states (see Chapter 14 of the UAVbook)
  Eigen::Vector3f velocity_dot = R(Theta).transpose()*gravity*Eigen::Vector3f::UnitZ() + y_accel + vels.cross(y_gyro- biases);
  Eigen::Vector3f euler_angles_dot = S(Theta)*(y_gyro - biases);
  
  // Stuff the derivative vector with the respective derivatives.
  Eigen::Vector<float, num_states> f = Eigen::Vector<float, num_states>::Zero();
  f << R(Theta)*vels, velocity_dot, euler_angles_dot;

  return f;
}

Eigen::MatrixXf EstimatorContinuousDiscrete::multirotor_jacobian(const Eigen::VectorXf& state, const Eigen::VectorXf& inputs)
{
  Eigen::Vector3f accel = inputs.block<3,1>(0,0);
  Eigen::Vector3f gyro = inputs.block<3,1>(3,0);
  
  Eigen::Vector3f vels = state.block<3,1>(3,0);
  Eigen::Vector3f Theta = state.block<3,1>(6,0);
  Eigen::Vector3f biases = state.block<3,1>(9,0);

  Eigen::Matrix<float, num_states, num_states> A = Eigen::Matrix<float, num_states, num_states>::Zero();

  // Identity matrix.
  A.block<3,3>(0,3) = R(Theta);
  A.block<3,3>(0,6) = del_R_Theta_v_del_Theta(Theta, vels);

  // -(y_gyro - bias)^x
  A.block<3,3>(3,3) = -skew_matrix(gyro - biases);
  
  // del R(Theta)*y_accel / del Theta
  A.block<3,3>(3,6) = del_R_Theta_T_g_del_Theta(Theta, gravity_);
  
  // -vel^x
  A.block<3,3>(3,9) = -skew_matrix(vels);

  // del S(Theta)(y_gyro - b) / del Theta
  A.block<3,3>(6,6) = del_S_Theta_del_Theta(Theta, biases, gyro);

  A.block<3,3>(6,9) = -S(Theta);
   
return A;
}

Eigen::MatrixXf EstimatorContinuousDiscrete::multirotor_input_jacobian(const Eigen::VectorXf& state, const Eigen::VectorXf& inputs)
{
  // This uses both the accel and gyro. The associated jacobians have been combined.
  Eigen::Matrix<float, num_states, 6> G = Eigen::Matrix<float, num_states, num_estimator_inputs>::Zero();
  
  Eigen::Vector3f Theta = state.block<3,1>(6,0);
  Eigen::Vector3f vels = state.block<3,1>(3,0);
  
  G.block<3,3>(3,0) = -Eigen::Matrix3f::Identity();

  G.block<3,3>(3,3) = -skew_matrix(vels);
  G.block<3,3>(6,3) = -S(Theta);

  return G;
}

// ======== MAG MEAUREMENT STEP EQUATIONS========
// These are passed by reference to the mag measurement update step.
Eigen::VectorXf EstimatorContinuousDiscrete::multirotor_mag_measurement_prediction(const Eigen::VectorXf& state, const Eigen::VectorXf& input)
{
  float declination = input(0);

  float inclination = input(1);
  
  Eigen::Vector3f Theta = state.block<3,1>(6,0);

  Eigen::Vector<float, num_mag_measurements> h = Eigen::Vector<float, num_mag_measurements>::Zero();

  Eigen::Vector3f inertial_mag_readings = calculate_inertial_magnetic_field(declination, inclination);

  // Rotate the magnetometer readings into the body frame.
  Eigen::Vector3f predicted_mag_readings = R(Theta).transpose()*inertial_mag_readings;
  predicted_mag_readings /= predicted_mag_readings.norm();

  // Predicted magnetometer measurement in each body axis.
  h.block<3,1>(0,0) = predicted_mag_readings;

  return h;
}

Eigen::MatrixXf EstimatorContinuousDiscrete::multirotor_mag_measurement_jacobian(const Eigen::VectorXf& state, const Eigen::VectorXf& input)
{
  Eigen::Vector3f Theta = state.block<3,1>(6,0);

  float declination = input(0);

  float inclination = input(1);

  Eigen::Vector3f inertial_mag = calculate_inertial_magnetic_field(declination, inclination); 

  Eigen::Matrix<float, 3, 3> R_theta_mag_jac = del_R_Theta_T_y_mag_del_Theta(Theta, inertial_mag);

  Eigen::Matrix<float, num_mag_measurements, num_states> C = Eigen::Matrix<float, num_mag_measurements, num_states>::Zero();
  
  // Magnetometer update
  C.block<3,3>(0,6) = R_theta_mag_jac.block<3,3>(0,0); 

  return C;
}

Eigen::MatrixXf EstimatorContinuousDiscrete::multirotor_mag_measurement_sensor_noise()
{
  Eigen::Matrix<float, num_mag_measurements, num_mag_measurements> R;

  R = R_mag_;

  return R;
}

// ======== BARO MEAUREMENT STEP EQUATIONS========
// These are passed by reference to the baro measurement update step.
Eigen::VectorXf EstimatorContinuousDiscrete::multirotor_baro_measurement_prediction(const Eigen::VectorXf& state, const Eigen::VectorXf& input)
{
  float rho = params_.get_double("rho");
  float gravity = params_.get_double("gravity");

  Eigen::Vector<float, num_baro_measurements> h = Eigen::Vector<float, num_baro_measurements>::Zero();

  // Predicted static pressure measurement
  h(0) = -rho*gravity*state(2);

  return h;
}

Eigen::MatrixXf EstimatorContinuousDiscrete::multirotor_baro_measurement_jacobian(const Eigen::VectorXf& state, const Eigen::VectorXf& input)
{
  float rho = params_.get_double("rho");
  float gravity = params_.get_double("gravity");

  Eigen::Matrix<float, num_baro_measurements, num_states> C = Eigen::Matrix<float, num_baro_measurements, num_states>::Zero();

  // Static pressure
  C(0,2) = -rho*gravity;

  return C;
}

Eigen::MatrixXf EstimatorContinuousDiscrete::multirotor_baro_measurement_sensor_noise()
{
  Eigen::Matrix<float, num_baro_measurements, num_baro_measurements> R;

  R = R_baro_;

  return R;
}

// ======== GNSS MEAUREMENT STEP EQUATIONS========
// These are passed by reference to the GNSS measurement update step.
Eigen::VectorXf EstimatorContinuousDiscrete::multirotor_gnss_measurement_prediction(const Eigen::VectorXf& state, const Eigen::VectorXf& input)
{
  Eigen::Vector<float, num_gnss_measurements> h = Eigen::Vector<float, num_gnss_measurements>::Zero();

  // North position
  h(0) = state(0);

  // East position
  h(1) = state(1);
  
  // Down position
  h(2) = state(2);

  // Rotate body vels into the intertial frame.
  
  Eigen::Vector3f inertial_vels;
  
  Eigen::Vector3f vels = state.block<3,1>(3,0);
  Eigen::Vector3f Theta = state.block<3,1>(6,0);

  inertial_vels = R(Theta) * vels;
  
  // Vel north
  h(3) = inertial_vels(0);

  // Vel east
  h(4) = inertial_vels(1);
  
  // Vel down
  h(5) = inertial_vels(2);
  
  // To add a new measurement, simply use the state and any input you need as another entry to h. Be sure to update the measurement jacobian C.
   
  return h;
}

Eigen::MatrixXf EstimatorContinuousDiscrete::multirotor_gnss_measurement_jacobian(const Eigen::VectorXf& state, const Eigen::VectorXf& input)
{
  Eigen::Matrix<float, num_gnss_measurements, num_states> C = Eigen::Matrix<float, num_gnss_measurements, num_states>::Zero();
  Eigen::Vector3f vels = state.block<3,1>(3,0);
  Eigen::Vector3f Theta = state.block<3,1>(6,0);
  
  // GPS north
  C(0,0) = 1;

  // GPS east
  C(1,1) = 1;
  
  // GPS down
  C(2,2) = 1;

  // GPS velocities effect on velocity
  C.block<3,3>(3,3) = R(Theta);
  
  // GPS velocities effect on Theta
  C.block<3,3>(3,6) = del_R_Theta_v_del_Theta(Theta, vels);

  // To add a new measurement use the inputs and the state to add another row to the matrix C. Be sure to update the measurment prediction vector h.

  return C;
}

Eigen::MatrixXf EstimatorContinuousDiscrete::multirotor_gnss_measurement_sensor_noise()
{
  Eigen::Matrix<float, num_gnss_measurements, num_gnss_measurements> R;

  R = R_gnss_;

  return R;
}

// ======== MEASUREMENT UPDATE HELPER FUNCTIONS========
Eigen::Matrix3f EstimatorContinuousDiscrete::R(const Eigen::Vector3f& Theta)
{
  // Finds rotation matrix from the inertial frame to the body frame using
  // RPY angles in the vector Theta.
  float phi = Theta(0);
  float theta = Theta(1);
  float psi = Theta(2);

  Eigen::Matrix3f R_theta;
  R_theta << cosf(psi)*cosf(theta), sinf(phi)*sinf(theta)*cosf(psi) - sinf(psi)*cosf(phi), sinf(phi)*sinf(psi) + sinf(theta)*cosf(phi)*cosf(psi),
             sinf(psi)*cosf(theta), sinf(phi)*sinf(psi)*sinf(theta) + cosf(phi)*cosf(psi), - sinf(phi)*cosf(psi) + sinf(psi)*sinf(theta)*cosf(phi),
             -sinf(theta), sinf(phi)*cosf(theta), cosf(phi)*cosf(theta);
  return R_theta;
}

Eigen::Matrix3f EstimatorContinuousDiscrete::S(const Eigen::Vector3f& Theta)
{
  // Finds transition matrix using RPY angles on the vector Theta. See chapter 14 of the UAVbook.
  float phi = Theta(0);
  float theta = Theta(1);

  Eigen::Matrix3f S_theta;
  S_theta << 1.0, sinf(phi)*tanf(theta), cosf(phi)*tanf(theta),
             0.0, cosf(phi), -sinf(phi),
             0.0, sinf(phi)/cosf(theta), cosf(phi)/cosf(theta);
  return S_theta;
}

Eigen::Matrix3f EstimatorContinuousDiscrete::del_S_Theta_del_Theta(const Eigen::Vector3f& Theta, const Eigen::Vector3f& biases,
                                                                   const Eigen::Vector3f& gyro)
{
  float bias_y = biases(1);
  float bias_z = biases(2);
  
  float gyro_y = gyro(1);
  float gyro_z = gyro(2);

  float phi = Theta(0);
  float theta = Theta(1);

  Eigen::Matrix3f S_Theta_jacobian;
  S_Theta_jacobian << 
  ((- bias_y + gyro_y)*cosf(phi) + (bias_z - gyro_z)*sinf(phi))*tanf(theta),
  ((- bias_y + gyro_y)*sinf(phi) + (- bias_z + gyro_z)*cosf(phi))/(cosf(theta)*cosf(theta)),
  0.0,
  (bias_y - gyro_y)*sinf(phi) + (bias_z - gyro_z)*cosf(phi),
  0.0,
  0.0,
  ((- bias_y + gyro_y)*cosf(phi) + (bias_z - gyro_z)*sinf(phi))*(1.0/cosf(theta)), 
  ((- bias_y + gyro_y)*sinf(phi) + (- bias_z + gyro_z)*cosf(phi))*tanf(theta)*(1.0/cosf(theta)), 
  0.0;

  return S_Theta_jacobian;
}

Eigen::Matrix<float, 3,3> EstimatorContinuousDiscrete::del_R_Theta_T_g_del_Theta(const Eigen::Vector3f& Theta, const double& gravity)
{
  float phi = Theta(0);
  float theta = Theta(1);

  Eigen::Matrix<float, 3, 3> R_theta_T_g_jac;

  R_theta_T_g_jac << 0.0, -gravity*cos(theta), 0.0,
                     gravity*cos(phi)*cos(theta), -gravity*sin(phi)*sin(theta), 0.0,
                     -gravity*sin(phi)*cos(theta), -gravity*sin(theta)*cos(phi), 0.0;

  return R_theta_T_g_jac;
}

Eigen::Matrix<float, 3,3> EstimatorContinuousDiscrete::del_R_Theta_v_del_Theta(const Eigen::Vector3f& Theta, const Eigen::Vector3f& vels)
{
  float phi = Theta(0);
  float theta = Theta(1);
  float psi = Theta(2);

  float v_n = vels(0);
  float v_e = vels(1);
  float v_d = vels(2); 

  Eigen::Matrix<float, 3, 3> R_theta_v_jac;

  R_theta_v_jac << v_e*(sin(phi)*sin(psi) + sin(theta)*cos(phi)*cos(psi)) + v_d*(-sin(phi)*sin(theta)*cos(psi) + sin(psi)*cos(phi)),
                   (-v_n*sin(theta) + v_e*sin(phi)*cos(theta) + v_d*cos(phi)*cos(theta))*cos(psi),
                   -v_n*sin(psi)*cos(theta) + v_e*(-sin(phi)*sin(psi)*sin(theta) - cos(phi)*cos(psi)) + v_d*(sin(phi)*cos(psi) - sin(psi)*sin(theta)*cos(phi)), 
                   v_e*(-sin(phi)*cos(psi) + sin(psi)*sin(theta)*cos(phi)) + v_d*(-sin(phi)*sin(psi)*sin(theta) - cos(phi)*cos(psi)),
                   (-v_n*sin(theta) + v_e*sin(phi)*cos(theta) + v_d*cos(phi)*cos(theta))*sin(psi),
                   v_n*cos(psi)*cos(theta) + v_e*(sin(phi)*sin(theta)*cos(psi) - sin(psi)*cos(phi)) + v_d*(sin(phi)*sin(psi) + sin(theta)*cos(phi)*cos(psi)), 
                   (v_e*cos(phi) - v_d*sin(phi))*cos(theta),
                   -v_n*cos(theta) - v_e*sin(phi)*sin(theta) - v_d*sin(theta)*cos(phi),
                   0.0; 

  return R_theta_v_jac;
}

Eigen::Matrix3f EstimatorContinuousDiscrete::del_R_Theta_T_y_mag_del_Theta(const Eigen::Vector3f& Theta, const Eigen::Vector3f& mag)
{
  float phi = Theta(0);
  float theta = Theta(1);
  float psi = Theta(2);

  float m_x = mag(0);
  float m_y = mag(1);
  float m_z = mag(2);

  Eigen::Matrix3f R_theta_T_mag_jac;

  R_theta_T_mag_jac << 0, - m_x * sinf(theta) * cosf(psi) - m_y * sinf(psi) * sinf(theta) - m_z * cosf(theta), - m_x * sinf(psi) * cosf(theta) + m_y * cosf(psi) * cosf(theta), m_x * (sinf(phi) * sinf(psi) + sinf(theta) * cosf(phi) * cosf(psi)) + m_y * (- sinf(phi) * cosf(psi) + sinf(psi) * sinf(theta) * cosf(phi)) + m_z * cosf(phi) * cosf(theta), m_x * sinf(phi) * cosf(psi) * cosf(theta) + m_y * sinf(phi) * sinf(psi) * cosf(theta) - m_z * sinf(phi) * sinf(theta), m_x * (- sinf(phi) * sinf(psi) * sinf(theta) - cosf(phi) * cosf(psi)) + m_y * (sinf(phi) * sinf(theta) * cosf(psi) - sinf(psi) * cosf(phi)), m_x * (- sinf(phi) * sinf(theta) * cosf(psi) + sinf(psi) * cosf(phi)) + m_y * (- sinf(phi) * sinf(psi) * sinf(theta) - cosf(phi) * cosf(psi)) - m_z * sinf(phi) * cosf(theta), m_x * cosf(phi) * cosf(psi) * cosf(theta) + m_y * sinf(psi) * cosf(phi) * cosf(theta) - m_z * sinf(theta) * cosf(phi), m_x * (sinf(phi) * cosf(psi) - sinf(psi) * sinf(theta) * cosf(phi)) + m_y * (sinf(phi) * sinf(psi) + sinf(theta) * cosf(phi) * cosf(psi));

  return R_theta_T_mag_jac;
}

// ======== MISC HELPER FUNCTIONS========
Eigen::Vector3f EstimatorContinuousDiscrete::calculate_inertial_magnetic_field(const float& declination, const float& inclination)
{
  // The full intesity of the magnetic field is in the x axis in the magnetic frame.
  Eigen::Vector3f mag_x = Eigen::Vector3f::UnitX();

  Eigen::Matrix3f mag_inclination_rotation;
  mag_inclination_rotation = Eigen::AngleAxisf(inclination, Eigen::Vector3f::UnitY());
  
  Eigen::Matrix3f mag_declination_rotation;
  mag_declination_rotation = Eigen::AngleAxisf(declination, Eigen::Vector3f::UnitZ());
  
  // Find the magnetic field intenisty described in the inertial frame by rotating the frame.
  Eigen::Matrix3f mag_rotation = mag_declination_rotation*mag_inclination_rotation;
  Eigen::Vector3f inertial_mag_readings = mag_rotation*mag_x;

  return inertial_mag_readings/inertial_mag_readings.norm();
}


void EstimatorContinuousDiscrete::calc_mag_field_properties(const Input& input)
{
  float inclination = params_.get_double("inclination");
  float declination = params_.get_double("declination");

  if ((0. <= inclination && inclination <= 90.) &&
      (-90. <= declination && declination <= 90.)) {
    inclination_ = inclination;
    declination_ = declination;
    mag_init_ = true;
  }

  if (mag_init_ || !has_fix_ || !input.gps_new) {
    return;
  }

  double total_intensity; // nanoTesla (unused)
  double grid_variation; // Only useful for arctic or antarctic navigation (unused).
  
  // Take the current year and then add a decimal for the current day. USE GPS TIME.
  // This is a rough interpolation for speed. This will be accurate +- 1 day which is a time decimal change of ~0.0027
  // therefore this method isn't completely accurate. Other sources of error will be much larger.
  // TODO: Change the gps day to pull out yday instead of the month and day
  float decimal_month = input.gps_month + input.gps_day/31.0;
  float decimal_year = input.gps_year + decimal_month/12.0;
  
  int mag_success = geomag_calc(input.gps_alt/1000.0,
                                input.gps_lat,
                                input.gps_lon,
                                decimal_year,
                                &declination_,
                                &inclination_,
                                &total_intensity,
                                &grid_variation); 

  if (mag_success == -1) {
    RCLCPP_ERROR(this->get_logger(), "Something went wrong while calculating inclination and declination.");
    RCLCPP_ERROR(this->get_logger(), "Inclination and Declination not set, estimation will likely be poor.");
  }
  else {
    mag_init_ = true;
  }
}

void EstimatorContinuousDiscrete::check_estimate(const Input& input)
{
  double gps_n_lim = params_.get_double("gps_n_lim");
  double gps_e_lim = params_.get_double("gps_e_lim");

  bool problem = false;
  int prob_index = 0;
  for (auto state : xhat_) {
    if (!std::isfinite(state)) {
      if (!problem) {
        problem = true;
      }
      switch (prob_index) {
        case 0:
          xhat_(0) = input.gps_n;
          break;
        case 1:
          xhat_(1) = input.gps_e;
          break;
        case 2:
          xhat_(2) = -input.gps_alt;
          break;
        case 3:
          xhat_(3) = input.gps_vn;
          break;
        case 4:
          xhat_(4) = input.gps_ve;
          break;
        case 5:
          xhat_(5) = input.gps_vd;
          break;
        default:
          xhat_(prob_index) = 0.0;
          break;
      }
      initialize_state_covariances();
    }
    prob_index++;
    if (problem) {
      RCLCPP_WARN(this->get_logger(), "Estimator reinitialized due to non-finite state %d",
                  prob_index);
      problem = false;
    }
  }

  if (xhat_(0) > gps_n_lim || xhat_(0) < -gps_n_lim) {
    RCLCPP_WARN(this->get_logger(), "gps n limit reached");
    xhat_(0) = input.gps_n;
  }
  if (xhat_(1) > gps_e_lim || xhat_(1) < -gps_e_lim) {
    RCLCPP_WARN(this->get_logger(), "gps e limit reached");
    xhat_(1) = input.gps_e;
  }
}

// ======== INITIALIZATION FUNCTIONS ========
void EstimatorContinuousDiscrete::initialize_state_covariances() 
{
  double pos_n_initial_cov = params_.get_double("pos_n_initial_cov");
  double pos_e_initial_cov = params_.get_double("pos_e_initial_cov");
  double pos_d_initial_cov = params_.get_double("pos_d_initial_cov");
  double vn_initial_cov = params_.get_double("vn_initial_cov");
  double ve_initial_cov = params_.get_double("ve_initial_cov");
  double vd_initial_cov = params_.get_double("vd_initial_cov");
  double phi_initial_cov = params_.get_double("phi_initial_cov");
  double theta_initial_cov = params_.get_double("theta_initial_cov");
  double psi_initial_cov = params_.get_double("psi_initial_cov");
  double bias_x_initial_cov = params_.get_double("bias_x_initial_cov");
  double bias_y_initial_cov = params_.get_double("bias_y_initial_cov");
  double bias_z_initial_cov = params_.get_double("bias_z_initial_cov");
  
  P_ = Eigen::MatrixXf::Identity(num_states, num_states);
  P_(0, 0) = pos_n_initial_cov;
  P_(1, 1) = pos_e_initial_cov;
  P_(2, 2) = pos_d_initial_cov;
  P_(3, 3) = vn_initial_cov;
  P_(4, 4) = ve_initial_cov;
  P_(5, 5) = vd_initial_cov;
  P_(6, 6) = phi_initial_cov; 
  P_(7, 7) = theta_initial_cov; 
  P_(8, 8) = psi_initial_cov; 
  P_(9, 9) = bias_x_initial_cov; 
  P_(10, 10) = bias_y_initial_cov; 
  P_(11, 11) = bias_z_initial_cov; 
}

void EstimatorContinuousDiscrete::initialize_process_noises()  // TODO: Use a parameter update callback to dynamically change this.
{
  double gyro_process_noise = params_.get_double("gyro_process_noise");
  double accel_process_noise = params_.get_double("accel_process_noise");
  double position_process_noise = params_.get_double("pos_process_noise");
  double roll_process_noise = params_.get_double("roll_process_noise");
  double pitch_process_noise = params_.get_double("pitch_process_noise");
  double yaw_process_noise = params_.get_double("yaw_process_noise");
  double alt_process_noise = params_.get_double("alt_process_noise");
  double velocity_process_noise = params_.get_double("vel_horizontal_process_noise");
  double velocity_vertical_process_noise = params_.get_double("vel_horizontal_process_noise");
  double bias_process_noise = params_.get_double("bias_process_noise");
  
  Eigen::Vector<float, num_estimator_inputs> estimator_input_process_noises = Eigen::Vector<float, num_estimator_inputs>::Zero();
  estimator_input_process_noises << pow(accel_process_noise,2), pow(accel_process_noise,2), pow(accel_process_noise,2),
                        pow(radians(gyro_process_noise), 2), pow(radians(gyro_process_noise), 2), pow(radians(gyro_process_noise), 2);

  Q_inputs_ = Eigen::DiagonalMatrix<float,num_estimator_inputs>(estimator_input_process_noises);

  Q_(0,0) = position_process_noise;
  Q_(1,1) = position_process_noise;
  Q_(2,2) = alt_process_noise;
  Q_(3,3) = velocity_process_noise;
  Q_(4,4) = velocity_process_noise;
  Q_(5,5) = velocity_vertical_process_noise;
  Q_(6,6) = roll_process_noise;
  Q_(7,7) = pitch_process_noise;
  Q_(8,8) = yaw_process_noise;
  Q_(9,9) = bias_process_noise;
  Q_(10,10) = bias_process_noise;
  Q_(11,11) = bias_process_noise;
}

void EstimatorContinuousDiscrete::update_measurement_model_parameters() // TODO: Use a parameter update callback to dynamically change this.
{ 
  // For readability, declare the parameters used in the function here
  double sigma_n_gps = params_.get_double("sigma_n_gps");
  double sigma_e_gps = params_.get_double("sigma_e_gps");
  double sigma_h_gps = params_.get_double("sigma_h_gps");
  double sigma_vn_gps = params_.get_double("sigma_vn_gps");
  double sigma_ve_gps = params_.get_double("sigma_ve_gps");
  double sigma_vd_gps = params_.get_double("sigma_vd_gps");
  double sigma_static_press = params_.get_double("sigma_static_press");
  double sigma_mag = params_.get_double("sigma_mag");
  double frequency = params_.get_double("estimator_update_frequency");
  double Ts = 1.0 / frequency;
  float gyro_cutoff_freq = params_.get_double("gyro_cutoff_freq");
  float baro_cutoff_freq = params_.get_double("baro_cutoff_freq");

  R_gnss_(0, 0) = powf(sigma_n_gps, 2);
  R_gnss_(1, 1) = powf(sigma_e_gps, 2);
  R_gnss_(2, 2) = powf(sigma_h_gps, 2);
  R_gnss_(3, 3) = powf(sigma_vn_gps, 2);
  R_gnss_(4, 4) = powf(sigma_ve_gps, 2);
  R_gnss_(5, 5) = powf(sigma_vd_gps, 2);
  
  R_baro_(0,0) = powf(sigma_static_press,2);

  R_mag_(0,0) = powf(sigma_mag,2);
  R_mag_(1,1) = powf(sigma_mag,2);
  R_mag_(2,2) = powf(sigma_mag,2);
  
  // Calculate low pass filter alpha values.
  alpha_gyro_ = exp(-2.*M_PI*gyro_cutoff_freq * Ts);
  alpha_baro_ = exp(-2.*M_PI*baro_cutoff_freq * Ts);
}

void EstimatorContinuousDiscrete::declare_parameters()
{
  
  // Sensor uncertainties
  params_.declare_double("sigma_n_gps", .01);
  params_.declare_double("sigma_e_gps", .01);
  params_.declare_double("sigma_h_gps", .03);
  params_.declare_double("sigma_vn_gps", .007);
  params_.declare_double("sigma_ve_gps", .007);
  params_.declare_double("sigma_vd_gps", .01);
  params_.declare_double("sigma_static_press", 10.0);
  params_.declare_double("sigma_mag", 0.04);
  params_.declare_double("sigma_accel", .025 * 9.81);

  // Low pass filter parameters
  params_.declare_double("gyro_cutoff_freq", 20.0);
  params_.declare_double("baro_cutoff_freq", 1.25);
  
  // Proccess noises
  params_.declare_double("roll_process_noise", 1000*powf(0.0001,2));
  params_.declare_double("pitch_process_noise", 1000*powf(0.0001,2));
  params_.declare_double("yaw_process_noise", 1000*powf(0.0001,2));
  params_.declare_double("gyro_process_noise", 0.13);
  params_.declare_double("accel_process_noise", 0.24525);
  params_.declare_double("pos_process_noise", 1000*powf(0.00003,2));
  params_.declare_double("alt_process_noise", 1000*0.000001);
  params_.declare_double("vel_horizontal_process_noise", 1000*powf(0.0001,2)); 
  params_.declare_double("vel_vertical_process_noise", 1000*powf(0.0001,2));
  params_.declare_double("bias_process_noise", 0.0000001*0.0000001);
  
  // Initial covariances
  params_.declare_double("pos_n_initial_cov", .0001);
  params_.declare_double("pos_e_initial_cov", .0001);
  params_.declare_double("pos_d_initial_cov", .0001);
  params_.declare_double("vn_initial_cov", .0001);
  params_.declare_double("ve_initial_cov", .0001);
  params_.declare_double("vd_initial_cov", .0001);
  params_.declare_double("phi_initial_cov", 0.005);
  params_.declare_double("theta_initial_cov", 0.005);
  params_.declare_double("psi_initial_cov", 1.0);
  params_.declare_double("bias_x_initial_cov", 0.0001);
  params_.declare_double("bias_y_initial_cov", 0.0001);
  params_.declare_double("bias_z_initial_cov", 0.0001);
  
  // Conversion flags
  params_.declare_bool("convert_to_gauss", true);
  
  // Magnetic Field Parameters -- Set to -1000.0 to make estimator
  // find values using the WMM given GPS. If a reasonable number 
  // is given for the parameter, it will be used. 
  params_.declare_double("inclination", NOT_IN_USE);
  params_.declare_double("declination", NOT_IN_USE);

  // Saturations limits
  params_.declare_double("max_estimated_phi", 85.0); // Deg
  params_.declare_double("max_estimated_theta", 80.0); // Deg
  params_.declare_double("gps_n_lim", 10000.);
  params_.declare_double("gps_e_lim", 10000.); 
}

bool EstimatorContinuousDiscrete::is_parameter_changed()
{
  if (parameter_changed) {
    parameter_changed = false;
    // TODO: Check if the parameter changed was relavent. Will require structural changes. Like putting names in a dictionary.
    return true;
  }
  return false;
}

void EstimatorContinuousDiscrete::update_estimation_params()
{
  initialize_process_noises();
  update_measurement_model_parameters();
}

void EstimatorContinuousDiscrete::bind_functions()
{
  // Used to declutter the code.
  using namespace std::placeholders;
  using This = EstimatorContinuousDiscrete;

  // This creates references to the functions that are necessary estimate. This means we can pass them to the EKF class's functions.
  // std::bind creates a forwarding reference to a function. So when we pass the binding object to another method, that method can call the
  // original function.
  multirotor_dynamics_model = std::bind(&This::multirotor_dynamics, this, _1, _2);
  multirotor_jacobian_model = std::bind(&This::multirotor_jacobian, this, _1, _2);
  multirotor_input_jacobian_model = std::bind(&This::multirotor_input_jacobian, this, _1, _2);

  multirotor_gnss_measurement_model = std::bind(&This::multirotor_gnss_measurement_prediction, this, _1, _2);
  multirotor_gnss_measurement_jacobian_model = std::bind(&This::multirotor_gnss_measurement_jacobian, this, _1, _2);
  multirotor_gnss_measurement_sensor_noise_model = std::bind(&This::multirotor_gnss_measurement_sensor_noise, this);

  multirotor_mag_measurement_model = std::bind(&This::multirotor_mag_measurement_prediction, this, _1, _2);
  multirotor_mag_measurement_jacobian_model = std::bind(&This::multirotor_mag_measurement_jacobian, this, _1, _2);
  multirotor_mag_measurement_sensor_noise_model = std::bind(&This::multirotor_mag_measurement_sensor_noise, this);

  multirotor_baro_measurement_model = std::bind(&This::multirotor_baro_measurement_prediction, this, _1, _2);
  multirotor_baro_measurement_jacobian_model = std::bind(&This::multirotor_baro_measurement_jacobian, this, _1, _2);
  multirotor_baro_measurement_sensor_noise_model = std::bind(&This::multirotor_baro_measurement_sensor_noise, this);
}

} // namespace roscopter
