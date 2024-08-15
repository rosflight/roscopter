#include <cmath>
#include <functional>
#include <rclcpp/logging.hpp>
#include <tuple>

#include "ekf/estimator_continuous_discrete.hpp"
#include "Eigen/src/Core/Matrix.h"
#include "ekf/estimator_ros.hpp"
#include "ekf/geomag.h"

// TODO:
// Get rid of all MatrixX, only use deterministic sizes.
// Can we get everything on the stack????

namespace roscopter
{

float radians(float degrees) { return M_PI * degrees / 180.0; }

double wrap_within_180(double fixed_heading, double wrapped_heading)
{
  // wrapped_heading - number_of_times_to_wrap * 2pi
  return wrapped_heading - floor((wrapped_heading - fixed_heading) / (2 * M_PI) + 0.5) * 2 * M_PI;
}

EstimatorContinuousDiscrete::EstimatorContinuousDiscrete()
    : EstimatorEKF()
    , xhat_(Eigen::VectorXf::Zero(12))
    , P_(Eigen::MatrixXf::Identity(12, 12))
    , Q_(Eigen::MatrixXf::Identity(12, 12))
    , Q_g_(Eigen::MatrixXf::Identity(6, 6))
    , R_(Eigen::MatrixXf::Zero(4, 4))
    , R_fast(Eigen::MatrixXf::Zero(4, 4))
{

  bind_functions(); // TODO: Document what the _models are.

  lpf_static_ = 0.0;
  lpf_diff_ = 0.0;

  alpha_ = 0.0f;

  geomag_init();

  // Declare and set parameters with the ROS2 system
  declare_parameters();
  params_.set_parameters();

  // Initialize covariance matrices from ROS2 parameters
  initialize_uncertainties();

  // Inits R matrix and alpha values with ROS2 parameters
  update_measurement_model_parameters();

  N_ = params_.get_int("num_propagation_steps");
}

void EstimatorContinuousDiscrete::initialize_state_covariances() {
  double pos_n_initial_cov = params_.get_double("pos_n_initial_cov");
  double pos_e_initial_cov = params_.get_double("pos_e_initial_cov");
  double pos_d_initial_cov = params_.get_double("pos_d_initial_cov");
  double vn_initial_cov = params_.get_double("vn_initial_cov");
  double ve_initial_cov = params_.get_double("ve_initial_cov");
  double vd_initial_cov = params_.get_double("vd_initial_cov");
  double phi_initial_cov = params_.get_double("phi_initial_cov");
  double theta_initial_cov = params_.get_double("theta_initial_cov");
  double psi_initial_cov = params_.get_double("psi_initial_cov");
  double bias_x_initial_cov = 0.0; //params_.get_double("bias_x_initial_cov"); FIXME: figure out what to actually do here.
  double bias_y_initial_cov = 0.0; //params_.get_double("bias_y_initial_cov");
  double bias_z_initial_cov = 0.0; //params_.get_double("bias_z_initial_cov");

  P_ = Eigen::MatrixXf::Identity(12, 12); 
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

void EstimatorContinuousDiscrete::initialize_uncertainties() {
  double gyro_process_noise = params_.get_double("gyro_process_noise");
  double accel_process_noise = params_.get_double("accel_process_noise");
  double position_process_noise = params_.get_double("pos_process_noise");
  double roll_process_noise = params_.get_double("roll_process_noise");
  double pitch_process_noise = params_.get_double("pitch_process_noise");
  double yaw_process_noise = params_.get_double("yaw_process_noise");
  double alt_process_noise = params_.get_double("alt_process_noise");
  double velocity_process_noise = params_.get_double("vel_horizontal_process_noise");
  double velocity_vertical_process_noise = params_.get_double("vel_horizontal_process_noise");

  Eigen::VectorXf imu_process_noises;
  imu_process_noises = Eigen::VectorXf::Zero(6);
  imu_process_noises << pow(accel_process_noise,2), pow(accel_process_noise,2), pow(accel_process_noise,2),
                        pow(radians(gyro_process_noise), 2), pow(radians(gyro_process_noise), 2), pow(radians(gyro_process_noise), 2);

  Q_g_ = Eigen::DiagonalMatrix<float,6>(imu_process_noises);

  Q_ = Q_*position_process_noise; // FIXME: make this more explicit
  Q_(2,2) = alt_process_noise;
  Q_(3,3) = velocity_process_noise;
  Q_(4,4) = velocity_process_noise;
  Q_(5,5) = velocity_vertical_process_noise;
  Q_(6,6) = roll_process_noise;
  Q_(7,7) = pitch_process_noise;
  Q_(8,8) = yaw_process_noise;
  Q_(9,9) = 0.0001; // FIXME: add bias params
  Q_(10,10) = 0.0001;
  Q_(11,11) = 0.0001;

  initialize_state_covariances();
}

void EstimatorContinuousDiscrete::update_measurement_model_parameters()
{
  // For readability, declare the parameters used in the function here
  double sigma_n_gps = params_.get_double("sigma_n_gps");
  double sigma_e_gps = params_.get_double("sigma_e_gps");
  double sigma_static_press = params_.get_double("sigma_static_press");
  double sigma_Vg_gps = params_.get_double("sigma_Vg_gps");
  double sigma_course_gps = params_.get_double("sigma_course_gps");
  double sigma_mag = params_.get_double("sigma_mag");
  double frequency = params_.get_double("estimator_update_frequency");
  double Ts = 1.0 / frequency;
  float lpf_a = params_.get_double("lpf_a");
  float lpf_a1 = params_.get_double("lpf_a1");

  R_(0, 0) = powf(sigma_n_gps, 2);
  R_(1, 1) = powf(sigma_e_gps, 2);
  R_(2, 2) = powf(0.005, 2);
  R_(3, 3) = powf(0.005, 2); // FIXME: add vel down
  
  R_fast(0,0) = powf(sigma_static_press,2);
  R_fast(1,1) = powf(sigma_mag,2);
  R_fast(2,2) = powf(sigma_mag,2);
  R_fast(3,3) = powf(sigma_mag,2);

  alpha_ = exp(-lpf_a * Ts);
  alpha1_ = exp(-lpf_a1 * Ts);
}

void EstimatorContinuousDiscrete::estimate(const Input & input, Output & output)
{
  // For readability, declare the parameters here
  double rho = params_.get_double("rho");
  double gravity = params_.get_double("gravity");
  double frequency = params_.get_double("estimator_update_frequency");
  double gps_n_lim = params_.get_double("gps_n_lim");
  double gps_e_lim = params_.get_double("gps_e_lim");
  double Ts = 1.0 / frequency;

  // Inits R matrix and alpha values with ROS2 parameters
  update_measurement_model_parameters();

  // low pass filter gyros to estimate angular rates // TODO: Do we need to do this?
  lpf_gyro_x_ = alpha_ * lpf_gyro_x_ + (1 - alpha_) * input.gyro_x;
  lpf_gyro_y_ = alpha_ * lpf_gyro_y_ + (1 - alpha_) * input.gyro_y;
  lpf_gyro_z_ = alpha_ * lpf_gyro_z_ + (1 - alpha_) * input.gyro_z;

  float phat = input.gyro_x;
  float qhat = input.gyro_y;
  float rhat = input.gyro_z;

  // Use magenetometer measures to produce a heading measurement.
  
  double total_intensity; // nanoTesla (unused) // FIXME: abstract into a function, make it run only once.
  double grid_variation; // Only useful for arctic or antarctic navigation (unused).
  
  // Take the current year and then add a decimal for the current day. USE GPS TIME.
  // This is a rough interpolation for speed. This will be accurate +- a day which is a time decimal change of ~0.0027
  // therefore this method isn't completely accurate. Other sources of error will be much larger.
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
    RCLCPP_ERROR(this->get_logger(), "Something went wrong while calculating inclanation and declination.");
  }

  // ESTIMATION
  // Prediction step
  Eigen::Vector<float, 6> imu_measurements; // These are the input to the prediction step
  imu_measurements << input.accel_x, input.accel_y, input.accel_z, input.gyro_x, input.gyro_y, input.gyro_z;

  std::tie(P_, xhat_) = propagate_model(xhat_, multirotor_dynamics_model, multirotor_jacobian_model, imu_measurements, multirotor_input_jacobian_model, P_, Q_, Q_g_, Ts);
  
  // Measurement updates.
  Eigen::VectorXf _(1); // This is used when no inputs are needed.
  
  Eigen::Vector3f mag_readings;
  mag_readings << input.mag_x/10000., input.mag_y/10000., input.mag_z/10000; // TODO: Add param for converting gauss.
  
  Eigen::Vector<float, 4> y_fast;
  lpf_static_ = alpha1_ * lpf_static_ + (1 - alpha1_) * input.static_pres; // TODO: Should we nix this?
  y_fast << lpf_static_, mag_readings;

  xhat_(8) = wrap_within_180(0.0, xhat_(8));

  // Only update when have new baro and new mag.
  if (new_baro_) { // TODO: should these be split up? 
    Eigen::Vector2f mag_info;
    mag_info << radians(declination_), radians(inclination_);

    std::tie(P_, xhat_) = measurement_update(xhat_, mag_info, multirotor_fast_measurement_model, y_fast, multirotor_fast_measurement_jacobian_model, R_fast, P_);

    new_baro_ = false;
  }

  // Only update if new GPS information is available.
  if (input.gps_new) {

    Eigen::VectorXf C = Eigen::VectorXf::Zero(12);
    C(5) = 1;
    std::tie(P_, xhat_) = single_measurement_update(input.gps_vd, xhat_(5), 0.01, C, xhat_, P_); // FIXME: put this in with the rest of the measurement update.

    //wrap course measurement
    float gps_course = fmodf(input.gps_course, radians(360.0f));
    gps_course = wrap_within_180(xhat_(3), gps_course);
    
    // Measurements for the postional states.
    Eigen::Vector<float, 4> y_pos;
    y_pos << input.gps_n, input.gps_e, input.gps_vn, input.gps_ve; //, mag_true_heading;
    
    // Update the state and covariance with based on the predicted and actual measurements.
    std::tie(P_, xhat_) = measurement_update(xhat_, _, multirotor_measurement_model, y_pos, multirotor_measurement_jacobian_model, R_, P_);
    // xhat_(9) = xhat_(10) = xhat_(11) = 0.0;
    if (xhat_(0) > gps_n_lim || xhat_(0) < -gps_n_lim) {
      RCLCPP_WARN(this->get_logger(), "gps n limit reached");
      xhat_(0) = input.gps_n;
    }
    if (xhat_(1) > gps_e_lim || xhat_(1) < -gps_e_lim) {
      RCLCPP_WARN(this->get_logger(), "gps e limit reached");
      xhat_(1) = input.gps_e;
    }
  }

  // bool problem = false; // FIXME: put this back but with the correct states.
  // int prob_index;
  // for (int i = 0; i < 7; i++) {
  //   if (!std::isfinite(xhat_(i))) {
  //     if (!problem) {
  //       problem = true;
  //       prob_index = i;
  //     }
  //     switch (i) {
  //       case 0:
  //         xhat_(i) = input.gps_n;
  //         break;
  //       case 1:
  //         xhat_(i) = input.gps_e;
  //         break;
  //       case 2:
  //         xhat_(i) = input.gps_Vg;
  //         break;
  //       case 3:
  //         xhat_(i) = input.gps_course;
  //         break;
  //       case 6:
  //         xhat_(i) = input.gps_course;
  //         break;
  //       default:
  //         xhat_(i) = 0;
  //     }
  //
  //     initialize_state_covariances();
  //   }
  // }
  // if (problem) {
  //   RCLCPP_WARN(this->get_logger(), "position estimator reinitialized due to non-finite state %d",
  //               prob_index);
  // }
  
  float pnhat = xhat_(0);
  float pehat = xhat_(1);
  float pdhat = xhat_(2);
  float vnhat = xhat_(3);
  float vehat = xhat_(4);
  float vdhat = xhat_(5);
  float phihat = xhat_(6);
  float thetahat = xhat_(7);
  float psihat = xhat_(8);
  float bxhat = xhat_(9);
  float byhat = xhat_(10);
  float bzhat = xhat_(11);

  output.pn = pnhat;
  output.pe = pehat;
  output.pd = pdhat;
  output.vn = vnhat;
  output.ve = vehat;
  output.vd = vdhat;
  output.p = phat;
  output.q = qhat;
  output.r = rhat;
  output.phi = phihat;
  output.theta = thetahat;
  output.psi = psihat;
  output.bx = bxhat;
  output.by = byhat;
  output.bz = bzhat;
}

Eigen::VectorXf EstimatorContinuousDiscrete::multirotor_dynamics(const Eigen::VectorXf& state, const Eigen::VectorXf& measurements)
{

  double gravity = params_.get_double("gravity");

  float phi = state(6);
  float theta = state(7);
  float psi = state(8);
  float bias_x = state(9);
  float bias_y = state(10);
  float bias_z = state(11);
  
  Eigen::Vector3f biases;
  biases << bias_x, bias_y, bias_z;
  
  float accel_x = measurements(0);
  float accel_y = measurements(1);
  float accel_z = measurements(2);
  float gyro_x = measurements(3);
  float gyro_y = measurements(4);
  float gyro_z = measurements(5);
  
  Eigen::Vector3f y_accel;
  y_accel << accel_x, accel_y, accel_z;
  
  Eigen::Vector3f y_gyro;
  y_gyro << gyro_x, gyro_y, gyro_z;

  Eigen::Vector3f e_3;
  e_3 << 0,0,1.0;

  Eigen::Matrix3f R_theta;
  R_theta << cosf(psi)*cosf(theta), sinf(phi)*sinf(theta)*cosf(psi) - sinf(psi)*cosf(phi), sinf(phi)*sinf(psi) + sinf(theta)*cosf(phi)*cosf(psi),
             sinf(psi)*cosf(theta), sinf(phi)*sinf(psi)*sinf(theta) + cosf(phi)*cosf(psi), - sinf(phi)*cosf(psi) + sinf(psi)*sinf(theta)*cosf(phi),
             -sinf(theta), sinf(phi)*cosf(theta), cosf(phi)*cosf(theta);
  
  Eigen::Vector3f velocity_dot = gravity*e_3 + R_theta*y_accel;

  Eigen::Matrix3f S_theta;
  S_theta << 1.0, sinf(phi)*tanf(theta), cosf(phi)*tanf(theta),
             0.0, cosf(phi), -sinf(phi),
             0.0, sinf(phi)/cosf(theta), cosf(phi)/cosf(theta);

  Eigen::Vector3f euler_angles_dot = S_theta*(y_gyro - biases);
  
  Eigen::VectorXf f;
  f = Eigen::VectorXf::Zero(12);

  f(0) = state(3);
  f(1) = state(4);
  f(2) = state(5);
  f(3) = velocity_dot(0);
  f(4) = velocity_dot(1);
  f(5) = velocity_dot(2);
  f(6) = euler_angles_dot(0);
  f(7) = euler_angles_dot(1);
  f(8) = euler_angles_dot(2);

  return f;
}

Eigen::MatrixXf EstimatorContinuousDiscrete::multirotor_jacobian(const Eigen::VectorXf& state, const Eigen::VectorXf& measurements)
{
  float accel_x = measurements(0);
  float accel_y = measurements(1);
  float accel_z = measurements(2);
  float gyro_x = measurements(3);
  float gyro_y = measurements(4);
  float gyro_z = measurements(5);

  float phi = state(6);
  float theta = state(7);
  float psi = state(8);

  float bias_x = state(9);
  float bias_y = state(10);
  float bias_z = state(11);

  Eigen::MatrixXf A;
  A = Eigen::MatrixXf::Zero(12, 12);

  // Identity matrix.
  A(0,3) = 1;
  A(1,4) = 1;
  A(2,5) = 1;
  
  // del R(Theta)*y_accel / del Theta
  A(3,6) = accel_y*(sinf(phi)*sinf(psi) + sinf(theta)*cosf(phi)*cosf(psi)) - accel_z *(sinf(phi)*sinf(theta)*cosf(psi) - sinf(psi)*cosf(phi));
  A(3,7) = (- accel_x*sin(theta) + accel_y*sin(phi)*cos(theta) + accel_z*cos(phi)*cos(theta))*cos(psi);
  A(3,8) = - accel_x*sinf(psi)*cosf(theta) - accel_y*(sinf(phi)*sinf(psi)*sinf(theta) + cosf(phi)*cosf(psi)) + accel_z*(sinf(phi)*cosf(psi) - sinf(psi)*sinf(theta)*cosf(phi));
  A(4,6) = - accel_y*(sinf(phi)*cosf(psi) - sinf(psi)*sinf(theta)*cosf(phi)) - accel_z*(sinf(phi)*sinf(psi)*sinf(theta) + cosf(phi)*cosf(psi));
  A(4,7) = (- accel_x*sinf(theta) + accel_y*sinf(phi)*cosf(theta) + accel_z*cosf(phi)*cosf(theta))*sinf(psi); 
  A(4,8) = accel_x*cosf(psi)*cosf(theta) + accel_y*(sinf(phi)*sinf(theta)*cosf(psi) - sinf(psi)*cosf(phi)) + accel_z*(sinf(phi)*sinf(psi) + sinf(theta)*cosf(phi)*cosf(psi));
  A(5,6) = (accel_y*cosf(phi) - accel_z*sinf(phi))*cosf(theta);
  A(5,7) =  - accel_x*cosf(theta) - accel_y*sinf(phi)*sinf(theta) - accel_z*sinf(theta)*cosf(phi);
  A(5,8) = 0.0;

  // del S(Theta)(y_gyro - b) / del Theta
  A(6,6) = ((- bias_y + gyro_y)*cosf(phi) + (bias_z - gyro_z)*sinf(phi))*tanf(theta);
  A(6,7) = ((- bias_y + gyro_y)*sinf(phi) + (- bias_z + gyro_z)*cosf(phi))/(cosf(theta)*cosf(theta));
  A(6,8) = 0.0;
  A(7,6) = (bias_y - gyro_y)*sinf(phi) + (bias_z - gyro_z)*cosf(phi);
  A(7,7) = 0.0;
  A(7,8) = 0.0;
  A(8,6) = ((- bias_y + gyro_y)*cosf(phi) + (bias_z - gyro_z)*sinf(phi))*(1.0/cosf(theta)); 
  A(8,7) = ((- bias_y + gyro_y)*sinf(phi) + (- bias_z + gyro_z)*cosf(phi))*tanf(theta)*(1.0/cosf(theta)); 
  A(8,8) = 0.0; 

  // -S(theta)
  A(6, 9) = -1.0;
  A(6, 10) = -sinf(phi)*tanf(theta);
  A(6, 11) = -cosf(phi)*tanf(theta);
  A(7, 9) = 0.0;
  A(7, 10) = -cosf(phi);
  A(7, 11) = sinf(phi);
  A(8, 9) = 0.0;
  A(8, 10) = -sinf(phi)/cosf(theta);
  A(8, 11) = -cosf(phi)/cosf(theta);

  return A;
}

Eigen::MatrixXf EstimatorContinuousDiscrete::multirotor_input_jacobian(const Eigen::VectorXf& state, const Eigen::VectorXf& inputs)
{

  // This uses both the accel and gyro. The associated jacobians have been combined.
  
  float phi = state(6);
  float theta = state(7);
  float psi = state(8);
  
  Eigen::MatrixXf G;
  G = Eigen::MatrixXf::Zero(12, 6);

  Eigen::Matrix3f R_theta;
  R_theta << cosf(psi)*cosf(theta), sinf(phi)*sinf(theta)*cosf(psi) - sinf(psi)*cosf(phi), sinf(phi)*sinf(psi) + sinf(theta)*cosf(phi)*cosf(psi),
             sinf(psi)*cosf(theta), sinf(phi)*sinf(psi)*sinf(theta) + cosf(phi)*cosf(psi), - sinf(phi)*cosf(psi) + sinf(psi)*sinf(theta)*cosf(phi),
             -sinf(theta), sinf(phi)*cosf(theta), cosf(phi)*cosf(theta);

  Eigen::Matrix3f S_theta;
  S_theta << 1.0, sinf(phi)*tanf(theta), cosf(phi)*tanf(theta),
             0.0, cosf(phi), -sinf(phi),
             0.0, sinf(phi)/cosf(theta), cosf(phi)/cosf(theta);

  G(3,0) = -R_theta(0,0); // TODO: Swap to block assignments.
  G(3,1) = -R_theta(0,1);
  G(3,2) = -R_theta(0,2);
  G(4,0) = -R_theta(1,0);
  G(4,1) = -R_theta(1,1);
  G(4,2) = -R_theta(1,2);
  G(5,0) = -R_theta(2,0);
  G(5,1) = -R_theta(2,1);
  G(5,2) = -R_theta(2,2);

  G(6,3) = -S_theta(0,0);
  G(6,4) = -S_theta(0,1);
  G(6,5) = -S_theta(0,2);
  G(7,3) = -S_theta(1,0);
  G(7,4) = -S_theta(1,1);
  G(7,5) = -S_theta(1,2);
  G(8,3) = -S_theta(2,0);
  G(8,4) = -S_theta(2,1);
  G(8,5) = -S_theta(2,2);

  return G;
}

Eigen::VectorXf EstimatorContinuousDiscrete::multirotor_fast_measurement_prediction(const Eigen::VectorXf& state, const Eigen::VectorXf& input)
{
  float rho = params_.get_double("rho");
  float gravity = params_.get_double("gravity");

  float declination = input(0);
  float inclination = input(1);
  
  float phi = state(6);
  float theta = state(7);
  float psi = state(8);

  Eigen::VectorXf h = Eigen::VectorXf::Zero(4);

  // A magnetometer reading when pointed exactly along the inclination
  // and declination is completely along the body x axis.
  
  Eigen::Vector3f body_x = Eigen::Vector3f::UnitX(); // TODO: put into its own function.

  Eigen::Matrix3f mag_inclination_rotation;
  mag_inclination_rotation = Eigen::AngleAxisf(inclination, Eigen::Vector3f::UnitY());
  
  Eigen::Matrix3f mag_declination_rotation;
  mag_inclination_rotation = Eigen::AngleAxisf(declination, Eigen::Vector3f::UnitZ());

  Eigen::Matrix3f mag_rotation = mag_inclination_rotation*mag_declination_rotation;
  
  Eigen::Matrix3f R_theta;
  R_theta << cosf(psi)*cosf(theta),   sinf(phi)*sinf(theta)*cosf(psi) - sinf(psi)*cosf(phi),  sinf(phi)*sinf(psi) + sinf(theta)*cosf(phi)*cosf(psi),
             sinf(psi)*cosf(theta),   sinf(phi)*sinf(psi)*sinf(theta) + cosf(phi)*cosf(psi),  -sinf(phi)*cosf(psi) + sinf(psi)*sinf(theta)*cosf(phi),
             -sinf(theta),            sinf(phi)*cosf(theta),                                  cosf(phi)*cosf(theta);
  
  // Rotate the magnetometer readings into the intertial frame and then into the body frame.
  Eigen::Vector3f predicted_mag_readings = R_theta*mag_rotation*body_x;
  predicted_mag_readings /= predicted_mag_readings.norm();
  
  // Predicted static pressure measurements
  h(0) = -rho*gravity*state(2);

  // Predicted magnetometer measurement in each body axis. // FIXME: Switch to block update.
  h(1) = predicted_mag_readings(0);
  h(2) = predicted_mag_readings(1);
  h(3) = predicted_mag_readings(2);

  return h;
}

Eigen::VectorXf EstimatorContinuousDiscrete::multirotor_measurement_prediction(const Eigen::VectorXf& state, const Eigen::VectorXf& input)
{
  float v_n = state(3);
  float v_e = state(4);
  
  Eigen::VectorXf h = Eigen::VectorXf::Zero(4);

  // GPS north
  h(0) = state(0);

  // GPS east
  h(1) = state(1);
  
  // GPS vel north
  h(2) = v_n;

  // GPS vel east
  h(3) = v_e;
  
  // To add a new measurement, simply use the state and any input you need as another entry to h. Be sure to update the measurement jacobian C.
   
  return h;
}

Eigen::MatrixXf EstimatorContinuousDiscrete::multirotor_fast_measurement_jacobian(const Eigen::VectorXf& state, const Eigen::VectorXf& input)
{
  float rho = params_.get_double("rho");
  float gravity = params_.get_double("gravity");
  
  float phi = state(6);
  float theta = state(7);
  float psi = state(8);

  float declination = input(0);
  float inclination = input(1);
  
  Eigen::Vector3f body_x = Eigen::Vector3f::UnitX();

  Eigen::Matrix3f mag_inclination_rotation;
  mag_inclination_rotation = Eigen::AngleAxisf(inclination, Eigen::Vector3f::UnitY());
  
  Eigen::Matrix3f mag_declination_rotation;
  mag_inclination_rotation = Eigen::AngleAxisf(declination, Eigen::Vector3f::UnitZ());

  Eigen::Matrix3f mag_rotation = mag_inclination_rotation*mag_declination_rotation;
  Eigen::Vector3f intertial_mag_readings = mag_rotation*body_x;

  float m_x = intertial_mag_readings(0); // These are the mag measures in the inertial frame.
  float m_y = intertial_mag_readings(1);
  float m_z = intertial_mag_readings(2);

  Eigen::Matrix3f R_theta_mag_jac;
  R_theta_mag_jac << m_y*(sinf(phi)*sinf(psi) + sinf(theta)*cosf(phi)*cosf(psi)) - m_z*(sinf(phi)*sinf(theta)*cosf(psi) - sinf(psi)*cosf(phi)), (- m_x*sinf(theta) + m_y*sinf(phi)*cosf(theta) + m_z*cosf(phi)*cosf(theta)) * cosf(psi), - m_x*sinf(psi)*cosf(theta) - m_y*(sinf(phi)*sinf(psi)*sinf(theta) + cosf(phi)*cosf(psi)) + m_z*(sinf(phi)*cosf(psi) - sinf(psi)*sinf(theta)*cosf(phi)), - m_y*(sinf(phi)*cosf(psi) - sinf(psi)*sinf(theta)*cosf(phi)) - m_z*(sinf(phi)*sinf(psi)*sinf(theta) + cosf(phi)*cosf(psi)), (- m_x*sinf(theta) + m_y*sinf(phi)*cosf(theta) + m_z*cosf(phi)*cosf(theta))*sinf(psi), m_x*cosf(psi)*cosf(theta) + m_y*(sinf(phi)*sinf(theta)*cosf(psi) - sinf(psi)*cosf(phi)) + m_z*(sinf(phi)*sinf(psi) + sinf(theta)*cosf(phi)*cosf(psi)), (m_y*cosf(phi) - m_z*sinf(phi))*cosf(theta), - m_x*cosf(theta) - m_y*sinf(phi)*sinf(theta) - m_z*sinf(theta)*cosf(phi), 0;

  Eigen::MatrixXf C = Eigen::MatrixXf::Zero(4,12);

  // Static pressure
  C(0,2) = -rho*gravity;
  
  // Magnetometer update
  C.block<3,3>(1,6) = R_theta_mag_jac;

  return C;
}

Eigen::MatrixXf EstimatorContinuousDiscrete::multirotor_measurement_jacobian(const Eigen::VectorXf& state, const Eigen::VectorXf& input)
{
  Eigen::MatrixXf C = Eigen::MatrixXf::Zero(4,12);
  
  // GPS north
  C(0,0) = 1;

  // GPS east
  C(1,1) = 1;

  // GPS velocities
  C(2,3) = 1;
  C(3,4) = 1;

  // To add a new measurement use the inputs and the state to add another row to the matrix C. Be sure to update the measurment prediction vector h.

  return C;
}

void EstimatorContinuousDiscrete::declare_parameters()
{ // FIXME: Get rid of extraneous parameters.
  // Add a param file to pull from.
  params_.declare_double("sigma_n_gps", .01);
  params_.declare_double("sigma_e_gps", .01);
  params_.declare_double("sigma_static_press", 1.0);
  params_.declare_double("sigma_Vg_gps", .00005);
  params_.declare_double("sigma_course_gps", .005 / 20);
  params_.declare_double("sigma_mag", 0.05); 
  params_.declare_double("sigma_accel", .0025 * 9.81);
  params_.declare_double("sigma_heading", 0.01);
  params_.declare_double("lpf_a", 50.0);
  params_.declare_double("lpf_a1", 8.0);
  params_.declare_double("gps_n_lim", 10000.);
  params_.declare_double("gps_e_lim", 10000.);

  params_.declare_double("roll_process_noise", 0.0000000001);   // Radians?, should be already squared
  params_.declare_double("pitch_process_noise", 0.000000000001);   // Radians?, already squared
  params_.declare_double("yaw_process_noise", 0.000000001);   // Radians?, already squared
  params_.declare_double("gyro_process_noise", 0.13);   // Deg, not squared
  params_.declare_double("accel_process_noise", 024525);   // m/s^2 not squared
  params_.declare_double("pos_process_noise", 0.0000001);   // already squared
  params_.declare_double("alt_process_noise", 0.001);   // already squared
  params_.declare_double("vel_horizontal_process_noise", 0.0001);   // already squared
  params_.declare_double("vel_vertical_process_noise", 0.00001);   // already squared

  params_.declare_double("pos_n_initial_cov", 100.);
  params_.declare_double("pos_e_initial_cov", 100.);
  params_.declare_double("pos_d_initial_cov", 100.);
  params_.declare_double("vn_initial_cov", 40000.);
  params_.declare_double("ve_initial_cov", 40000.);
  params_.declare_double("vd_initial_cov", 40000.);
  params_.declare_double("phi_initial_cov", 0.005);
  params_.declare_double("theta_initial_cov", 0.005);
  params_.declare_double("psi_initial_cov", 0.01);
  params_.declare_double("bias_x_initial_cov", 0.01);
  params_.declare_double("bias_y_initial_cov", 0.01);
  params_.declare_double("bias_z_initial_cov", 0.01);

  params_.declare_int("num_propagation_steps", 1);

  params_.declare_double("max_estimated_phi", 85.0); // Deg
  params_.declare_double("max_estimated_theta", 80.0); // Deg
  params_.declare_double("estimator_max_buffer", 3.0);   // Deg
}

void EstimatorContinuousDiscrete::bind_functions()
{
  // This creates references to the functions that are necessary estimate. This means we can pass them to the EKF class's functions.
  // std::bind creates a forwarding reference to a function. So when we pass the binding object to another method, that method can call the
  // original function.
  multirotor_dynamics_model = std::bind(&EstimatorContinuousDiscrete::multirotor_dynamics, this, std::placeholders::_1, std::placeholders::_2);
  multirotor_jacobian_model = std::bind(&EstimatorContinuousDiscrete::multirotor_jacobian, this, std::placeholders::_1, std::placeholders::_2);
  multirotor_input_jacobian_model = std::bind(&EstimatorContinuousDiscrete::multirotor_input_jacobian, this, std::placeholders::_1, std::placeholders::_2);
  multirotor_measurement_model = std::bind(&EstimatorContinuousDiscrete::multirotor_measurement_prediction, this, std::placeholders::_1, std::placeholders::_2);
  multirotor_fast_measurement_model = std::bind(&EstimatorContinuousDiscrete::multirotor_fast_measurement_prediction, this, std::placeholders::_1, std::placeholders::_2);
  multirotor_measurement_jacobian_model = std::bind(&EstimatorContinuousDiscrete::multirotor_measurement_jacobian, this, std::placeholders::_1, std::placeholders::_2);
  multirotor_fast_measurement_jacobian_model = std::bind(&EstimatorContinuousDiscrete::multirotor_fast_measurement_jacobian, this, std::placeholders::_1, std::placeholders::_2);

}

} // namespace roscopter
