#include <cmath>
#include <functional>
#include <tuple>

#include "ekf/estimator_continuous_discrete.hpp"
#include "Eigen/src/Core/Matrix.h"
#include "ekf/estimator_ros.hpp"

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
    , xhat_(Eigen::VectorXf::Zero(7))
    , P_(Eigen::MatrixXf::Identity(7, 7))
    , R_accel_(Eigen::Matrix3f::Identity())
    , Q_(Eigen::MatrixXf::Identity(7, 7))
    , R_(Eigen::MatrixXf::Zero(7, 7))
    , f_(7)
    , A_(7, 7)
    , C_(7)
    , L_(7)
{

  bind_functions(); // TODO: Document what the _models are.

  phat_ = 0;
  qhat_ = 0;
  rhat_ = 0;
  phihat_ = 0;
  thetahat_ = 0;
  psihat_ = 0;
  Vwhat_ = 0;

  lpf_static_ = 0.0;
  lpf_diff_ = 0.0;

  alpha_ = 0.0f;

  // Declare and set parameters with the ROS2 system
  declare_parameters();
  params_.set_parameters();

  // Initialize covariance matrices from ROS2 parameters
  initialize_uncertainties();

  // Inits R matrix and alpha values with ROS2 parameters
  update_measurement_model_parameters();

  N_ = params_.get_int("num_propagation_steps");
}

EstimatorContinuousDiscrete::EstimatorContinuousDiscrete(bool use_params) : EstimatorContinuousDiscrete()
{
  double init_lat = params_.get_double("init_lat");
  double init_long = params_.get_double("init_lon");
  double init_alt = params_.get_double("init_alt");
  double init_static = params_.get_double("baro_calibration_val");

  RCLCPP_INFO_STREAM(this->get_logger(), "Using seeded estimator values.");
  RCLCPP_INFO_STREAM(this->get_logger(), "Seeded initial latitude: " << init_lat);
  RCLCPP_INFO_STREAM(this->get_logger(), "Seeded initial longitude: " << init_long);
  RCLCPP_INFO_STREAM(this->get_logger(), "Seeded initial altitude: " << init_alt);
  RCLCPP_INFO_STREAM(this->get_logger(), "Seeded barometer calibration value: " << init_static);

  gps_init_ = true;
  init_lat_ = init_lat;
  init_lon_ = init_long;
  init_alt_ = init_alt;

  baro_init_ = true;
  init_static_ = init_static;
}

void EstimatorContinuousDiscrete::initialize_state_covariances() {
  double pos_n_initial_cov = params_.get_double("pos_n_initial_cov");
  double pos_e_initial_cov = params_.get_double("pos_e_initial_cov");
  double vg_initial_cov = params_.get_double("vg_initial_cov");
  double chi_initial_cov = params_.get_double("chi_initial_cov");
  double wind_n_initial_cov = params_.get_double("wind_n_initial_cov");
  double wind_e_initial_cov = params_.get_double("wind_e_initial_cov");
  double psi_initial_cov = params_.get_double("psi_initial_cov");

  P_ = Eigen::MatrixXf::Identity(7, 7);
  P_(0, 0) = pos_n_initial_cov;
  P_(1, 1) = pos_e_initial_cov;
  P_(2, 2) = vg_initial_cov;
  P_(3, 3) = radians(chi_initial_cov);
  P_(4, 4) = wind_n_initial_cov;
  P_(5, 5) = wind_e_initial_cov;
  P_(6, 6) = radians(psi_initial_cov);
}

void EstimatorContinuousDiscrete::initialize_uncertainties() {
  double roll_process_noise = params_.get_double("roll_process_noise");
  double pitch_process_noise = params_.get_double("pitch_process_noise");
  double gyro_process_noise = params_.get_double("gyro_process_noise");
  double position_process_noise = params_.get_double("pos_process_noise");
  double attitude_initial_cov = params_.get_double("attitude_initial_cov");

  Q_g_ *= pow(radians(gyro_process_noise), 2);

  Q_ *= position_process_noise;

  initialize_state_covariances();
}

void EstimatorContinuousDiscrete::update_measurement_model_parameters()
{
  // For readability, declare the parameters used in the function here
  double sigma_n_gps = params_.get_double("sigma_n_gps");
  double sigma_e_gps = params_.get_double("sigma_e_gps");
  double sigma_Vg_gps = params_.get_double("sigma_Vg_gps");
  double sigma_course_gps = params_.get_double("sigma_course_gps");
  double sigma_accel = params_.get_double("sigma_accel");
  double sigma_pseudo_wind_n = params_.get_double("sigma_pseudo_wind_n");
  double sigma_pseudo_wind_e = params_.get_double("sigma_pseudo_wind_e");
  double sigma_heading = params_.get_double("sigma_heading");
  double frequency = params_.get_double("estimator_update_frequency");
  double Ts = 1.0 / frequency;
  float lpf_a = params_.get_double("lpf_a");
  float lpf_a1 = params_.get_double("lpf_a1");

  R_accel_ = Eigen::Matrix3f::Identity() * pow(sigma_accel, 2);

  R_(0, 0) = powf(sigma_n_gps, 2);
  R_(1, 1) = powf(sigma_e_gps, 2);
  R_(2, 2) = powf(sigma_Vg_gps, 2);
  R_(3, 3) = powf(sigma_course_gps, 2);
  R_(4, 4) = sigma_pseudo_wind_n;
  R_(5, 5) = sigma_pseudo_wind_e;
  R_(6, 6) = sigma_heading;

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

  // low pass filter gyros to estimate angular rates
  lpf_gyro_x_ = alpha_ * lpf_gyro_x_ + (1 - alpha_) * input.gyro_x;
  lpf_gyro_y_ = alpha_ * lpf_gyro_y_ + (1 - alpha_) * input.gyro_y;
  lpf_gyro_z_ = alpha_ * lpf_gyro_z_ + (1 - alpha_) * input.gyro_z;

  float phat = lpf_gyro_x_;
  float qhat = lpf_gyro_y_;
  float rhat = lpf_gyro_z_;
  
  // These states will allow us to propagate our state model for pitch and roll.
  Eigen::Vector3f angular_rates;
  angular_rates << phat, qhat, rhat;

  // low pass filter static pressure sensor and invert to esimate altitude
  lpf_static_ = alpha1_ * lpf_static_ + (1 - alpha1_) * input.static_pres;
  float hhat = lpf_static_ / rho / gravity;

  if (input.static_pres == 0.0
      || baro_init_ == false) { // Catch the edge case for if pressure measured is zero.
    hhat = 0.0;
  }

  // low pass filter accelerometers
  lpf_accel_x_ = alpha_ * lpf_accel_x_ + (1 - alpha_) * input.accel_x;
  lpf_accel_y_ = alpha_ * lpf_accel_y_ + (1 - alpha_) * input.accel_y;
  lpf_accel_z_ = alpha_ * lpf_accel_z_ + (1 - alpha_) * input.accel_z;

  Eigen::VectorXf imu_measurements;
  imu_measurements = Eigen::VectorXf::Zero(6);
  imu_measurements << input.accel_x, input.accel_y, input.accel_z, input.gyro_x, input.gyro_y, input.gyro_z;
  
  // ESTIMATION
  // Prediction step
  std::tie(P_, xhat_) = propagate_model(xhat_, multirotor_dynamics_model, multirotor_jacobian_model, imu_measurements, multirotor_input_jacobian_model, P_, Q_, Eigen::MatrixXf::Zero(6, 6), Ts); // TODO: replace 6x6 with the true 6x6 input process uncertainty.
  
    // Check wrapping of the heading and course.
  xhat_(8) = wrap_within_180(0.0, xhat_(8));
  if (xhat_(8) > radians(180.0f) || xhat_(8) < radians(-180.0f)) {
    RCLCPP_WARN(this->get_logger(), "Psi estimate not wrapped from -pi to pi");
    xhat_(8) = 0;
  }

  // Measurement updates.
  // Only update if new GPS information is available.
  if (input.gps_new) {
    Eigen::VectorXf pos_curr_state_info(1);

    //wrap course measurement
    float gps_course = fmodf(input.gps_course, radians(360.0f));
    gps_course = wrap_within_180(xhat_(3), gps_course);
    
    // Measurements for the postional states.
    Eigen::Vector<float, 6> y_pos;
    y_pos << input.gps_n, input.gps_e, input.gps_Vg, gps_course, 0.0, 0.0;
    
    // Update the state and covariance with based on the predicted and actual measurements.
    std::tie(P_, xhat_) = measurement_update(xhat_, pos_curr_state_info, multirotor_measurement_model, y_pos, multirotor_measurement_jacobian_model, R_, P_);

    if (xhat_(0) > gps_n_lim || xhat_(0) < -gps_n_lim) {
      RCLCPP_WARN(this->get_logger(), "gps n limit reached");
      xhat_(0) = input.gps_n;
    }
    if (xhat_(1) > gps_e_lim || xhat_(1) < -gps_e_lim) {
      RCLCPP_WARN(this->get_logger(), "gps e limit reached");
      xhat_(1) = input.gps_e;
    }
  }

  bool problem = false;
  int prob_index;
  for (int i = 0; i < 7; i++) {
    if (!std::isfinite(xhat_(i))) {
      if (!problem) {
        problem = true;
        prob_index = i;
      }
      switch (i) {
        case 0:
          xhat_(i) = input.gps_n;
          break;
        case 1:
          xhat_(i) = input.gps_e;
          break;
        case 2:
          xhat_(i) = input.gps_Vg;
          break;
        case 3:
          xhat_(i) = input.gps_course;
          break;
        case 6:
          xhat_(i) = input.gps_course;
          break;
        default:
          xhat_(i) = 0;
      }

      initialize_state_covariances();
    }
  }
  if (problem) {
    RCLCPP_WARN(this->get_logger(), "position estimator reinitialized due to non-finite state %d",
                prob_index);
  }
  if (xhat_(6) - xhat_(3) > radians(360.0f) || xhat_(6) - xhat_(3) < radians(-360.0f)) {
    xhat_(6) = fmodf(xhat_(6), 2.0 * M_PI);
  }

  float pnhat = xhat_(0);
  float pehat = xhat_(1);
  float Vghat = xhat_(2);
  float chihat = xhat_(3);
  float wnhat = xhat_(4);
  float wehat = xhat_(5);
  float psihat = xhat_(6);

  output.pn = pnhat;
  output.pe = pehat;
  output.h = hhat;
  output.alpha = 0;
  output.beta = 0;
  output.phi = phihat_;
  output.theta = thetahat_;
  output.chi = chihat;
  output.p = phat;
  output.q = qhat;
  output.r = rhat;
  output.Vg = Vghat;
  output.wn = wnhat;
  output.we = wehat;
  output.psi = psihat;
}

Eigen::VectorXf EstimatorContinuousDiscrete::multirotor_dynamics(const Eigen::VectorXf& state, const Eigen::VectorXf& measurements)
{

  double gravity = params_.get_double("gravity");

  float Vg = state(2);
  float chi = state(3);
  float wn = state(4);
  float we = state(5);
  float psi = state(6);

  float p = measurements(0);
  float q = measurements(1);
  float r = measurements(2);
  float phi = measurements(3);
  float theta = measurements(4);
  float va = measurements(5);

  float psidot = (q * sinf(phi) + r * cosf(phi)) / cosf(theta);

  float Vgdot = va / Vg * psidot * (we * cosf(psi) - wn * sinf(psi));
  
  Eigen::VectorXf f;
  f = Eigen::VectorXf::Zero(7);

  f(0) = state(2) * cosf(state(3));
  f(1) = state(2) * sinf(state(3));
  f(2) = Vgdot;
  f(3) = gravity / state(2) * tanf(phi) * cosf(chi - psi);
  f(6) = psidot;

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
  A(3,6) = accel_y*(sinf(phi)*sinf(psi) + sinf(theta)*cos(phi)*cos(psi)) - accel_z *(sinf(phi)*sinf(theta)*cos(psi - sinf(psi)*cos(phi)));
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
  
  Eigen::MatrixXf G;
  G = Eigen::MatrixXf::Zero(7, 7);

  return G;
}

Eigen::VectorXf EstimatorContinuousDiscrete::multirotor_measurement_prediction(const Eigen::VectorXf& state, const Eigen::VectorXf& input)
{
  float va = input(0);
  
  Eigen::VectorXf h = Eigen::VectorXf::Zero(6);

  // GPS north
  h(0) = state(0);

  // GPS east
  h(1) = state(1);

  // GPS ground speed
  h(2) = state(2);
  
  // GPS course
  h(3) = state(3);
  
  // Pseudo Measurement north
  h(4) = va * cosf(state(6)) + state(4) - state(2) * cosf(state(3));
  
  // Pseudo Measurement east
  h(5) = va * sinf(state(6)) + state(5) - state(2) * sinf(state(3));

  // To add a new measurement, simply use the state and any input you need as another entry to h. Be sure to update the measurement jacobian C.
   
  return h;
}

Eigen::MatrixXf EstimatorContinuousDiscrete::multirotor_measurement_jacobian(const Eigen::VectorXf& state, const Eigen::VectorXf& input)
{
  float va = input(0);

  Eigen::MatrixXf C = Eigen::MatrixXf::Zero(6,7);
  
  // GPS north
  C(0,0) = 1;

  // GPS east
  C(1,1) = 1;
  
  // GPS ground speed
  C(2,2) = 1;

  // GPS course
  C(3,3) = 1;
  
  // Pseudo Measurement north
  C(4,2) = -cos(state(3));
  C(4,3) = state(2) * sinf(state(3));
  C(4,4) = 1;
  C(4,6) = -va * sinf(state(6));

  // Pseudo Measurement east
  C(5, 2) = -sin(state(3));
  C(5, 3) = -state(2) * cosf(state(3));
  C(5, 5) = 1;
  C(5, 6) = va * cosf(state(6));

  // To add a new measurement use the inputs and the state to add another row to the matrix C. Be sure to update the measurment prediction vector h.

  return C;
}

void EstimatorContinuousDiscrete::declare_parameters()
{
  params_.declare_double("sigma_n_gps", .01);
  params_.declare_double("sigma_e_gps", .01);
  params_.declare_double("sigma_Vg_gps", .005);
  params_.declare_double("sigma_course_gps", .005 / 20);
  params_.declare_double("sigma_accel", .0025 * 9.81);
  params_.declare_double("sigma_pseudo_wind_n", 0.01);
  params_.declare_double("sigma_pseudo_wind_e", 0.01);
  params_.declare_double("sigma_heading", 0.01);
  params_.declare_double("lpf_a", 50.0);
  params_.declare_double("lpf_a1", 8.0);
  params_.declare_double("gps_n_lim", 10000.);
  params_.declare_double("gps_e_lim", 10000.);

  params_.declare_double("roll_process_noise", 0.0001);   // Radians?, should be already squared
  params_.declare_double("pitch_process_noise", 0.0000001);   // Radians?, already squared
  params_.declare_double("gyro_process_noise", 0.13);   // Deg, not squared
  params_.declare_double("pos_process_noise", 0.1);   // already squared

  params_.declare_double("attitude_initial_cov", 5.0); // Deg, not squared
  params_.declare_double("pos_n_initial_cov", 0.03);
  params_.declare_double("pos_e_initial_cov", 0.03);
  params_.declare_double("vg_initial_cov", 0.01);
  params_.declare_double("chi_initial_cov", 5.0);  // Deg
  params_.declare_double("wind_n_initial_cov", 0.04);
  params_.declare_double("wind_e_initial_cov", 0.04);
  params_.declare_double("psi_initial_cov", 5.0);  // Deg

  params_.declare_int("num_propagation_steps", 10);

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
  multirotor_measurement_jacobian_model = std::bind(&EstimatorContinuousDiscrete::multirotor_measurement_jacobian, this, std::placeholders::_1, std::placeholders::_2);

}

} // namespace roscopter
