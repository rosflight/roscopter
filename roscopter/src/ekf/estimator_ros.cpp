#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iomanip>

#include "ekf/estimator_ros.hpp"
#include "ekf/estimator_continuous_discrete.hpp"

namespace roscopter
{

EstimatorROS::EstimatorROS()
    : Node("estimator_ros"), params_(this), params_initialized_(false)
{
  vehicle_state_pub_ = this->create_publisher<roscopter_msgs::msg::State>("estimated_state", 10);
  true_mag_pub_ = this->create_publisher<sensor_msgs::msg::MagneticField>("rotated_mag", 10);

  gnss_fix_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
    gnss_fix_topic_, 10, std::bind(&EstimatorROS::gnssFixCallback, this, std::placeholders::_1));
  gnss_vel_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
    gnss_vel_topic_, 10, std::bind(&EstimatorROS::gnssVelCallback, this, std::placeholders::_1));
  imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
    imu_topic_, 10, std::bind(&EstimatorROS::imuCallback, this, std::placeholders::_1));
  baro_sub_ = this->create_subscription<rosflight_msgs::msg::Barometer>(
    baro_topic_, 10, std::bind(&EstimatorROS::baroAltCallback, this, std::placeholders::_1));
  status_sub_ = this->create_subscription<rosflight_msgs::msg::Status>(
    status_topic_, 10, std::bind(&EstimatorROS::statusCallback, this, std::placeholders::_1));
  magnetometer_sub_ = this->create_subscription<sensor_msgs::msg::MagneticField>(
    magnetometer_topic_, 10, std::bind(&EstimatorROS::magnetometerCallback, this, std::placeholders::_1));
  true_state_sub_ = this->create_subscription<roscopter_msgs::msg::State>(
    "state", 10, std::bind(&EstimatorROS::trueStateCallback, this, std::placeholders::_1));
  comp_filt_sub_ = this->create_subscription<geometry_msgs::msg::Vector3Stamped>(
    "attitude/euler", 10, std::bind(&EstimatorROS::compFiltCallback, this, std::placeholders::_1));

  init_static_ = 0;
  baro_count_ = 0;
  armed_first_time_ = false;
  baro_init_ = false;
  gps_init_ = false;

  // Set the parameter callback, for when parameters are changed.
  parameter_callback_handle_ = this->add_on_set_parameters_callback(
    std::bind(&EstimatorROS::parametersCallback, this, std::placeholders::_1));

  // Declare and set parameters with the ROS2 system
  declare_parameters();
  params_.set_parameters();

  params_initialized_ = true;

  set_timer();
}

void EstimatorROS::declare_parameters()
{
  params_.declare_double("estimator_update_frequency", 390.0);
  params_.declare_double("rho", 1.225);
  params_.declare_double("gravity", 9.8);
  params_.declare_double("gps_ground_speed_threshold", 0.3);  // TODO: this is a magic number. What is it determined from?
  params_.declare_double("baro_measurement_gate", 1.35);  // TODO: this is a magic number. What is it determined from?
  params_.declare_double("airspeed_measurement_gate", 5.0);  // TODO: this is a magic number. What is it determined from?
  params_.declare_int("baro_calibration_count", 100);  // TODO: this is a magic number. What is it determined from?
  params_.declare_double("baro_calibration_val", 0.0);
  params_.declare_double("init_lat", 0.0);
  params_.declare_double("init_lon", 0.0);
  params_.declare_double("init_alt", 0.0);
}

void EstimatorROS::set_timer() {
  double frequency = params_.get_double("estimator_update_frequency");

  update_period_ = std::chrono::microseconds(static_cast<long long>(1.0 / frequency * 1'000'000));
  update_timer_ = this->create_wall_timer(update_period_, std::bind(&EstimatorROS::update, this));
}

rcl_interfaces::msg::SetParametersResult 
EstimatorROS::parametersCallback(const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";

  bool success = params_.set_parameters_callback(parameters);
  if (!success)
  {
    result.successful = false;
    result.reason =
      "One of the parameters given is not a parameter of the estimator node.";
  }

  // Check to see if the timer period was changed. If it was, recreate the timer with the new period
  if (params_initialized_ && success) {
    std::chrono::microseconds curr_period = std::chrono::microseconds(static_cast<long long>(1.0 / params_.get_double("estimator_update_frequency") * 1'000'000));
    if (update_period_ != curr_period) {
      update_timer_->cancel();
      set_timer();
    }

  }

  return result;
}

void EstimatorROS::update()
{
  Output output;

  if (armed_first_time_) {
    estimate(input_, output);
  } else {
    output.pn = output.pe = output.pd = 0;
    output.phi = output.theta = output.psi = 0;
    output.vn = output.ve = output.vd = 0;
    output.p = output.q = output.r = 0;
    output.Vg = 0;
    output.bx = output.by = output.bz = 0;
  }

  input_.gps_new = false;

  // TODO: create state publisher.
  
  roscopter_msgs::msg::State msg = roscopter_msgs::msg::State();

  msg.position[0] = output.pn;
  msg.position[1] = output.pe;
  msg.position[2] = output.pd;
  msg.v_n = output.vn;
  msg.v_e = output.ve;
  msg.v_d = output.vd;
  msg.phi = output.phi;
  msg.theta = output.theta;
  msg.psi = output.psi;
  msg.p = output.p;
  msg.q = output.q;
  msg.r = output.r;
  msg.bx = output.bx;
  msg.by = output.by;
  msg.bz = output.bz;
  
  vehicle_state_pub_->publish(msg);
}

void EstimatorROS::gnssFixCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
{
  bool has_fix = msg->status.status
    >= sensor_msgs::msg::NavSatStatus::STATUS_FIX; // Higher values refer to augmented fixes
  if (!has_fix || !std::isfinite(msg->latitude)) {
    input_.gps_new = false;
    return;
  }
  if (!gps_init_ && has_fix) {
    gps_init_ = true;
    init_alt_ = msg->altitude;
    init_lat_ = msg->latitude;
    init_lon_ = msg->longitude;
    // saveParameter("init_lat", init_lat_);
    // saveParameter("init_lon", init_lon_);
    // saveParameter("init_alt", init_alt_);
  } else {
    input_.gps_lat = msg->latitude;
    input_.gps_lon = msg->longitude;
    input_.gps_alt = msg->altitude;
    input_.gps_year = 2024; // FIXME: add the right code to have this be accurate.
    input_.gps_month = 7;
    input_.gps_day = 18;
    input_.gps_n = EARTH_RADIUS * (msg->latitude - init_lat_) * M_PI / 180.0;
    input_.gps_e =
      EARTH_RADIUS * cos(init_lat_ * M_PI / 180.0) * (msg->longitude - init_lon_) * M_PI / 180.0;
    input_.gps_h = msg->altitude - init_alt_;
    input_.gps_new = true;
  }
}

void EstimatorROS::trueStateCallback(const roscopter_msgs::msg::State::SharedPtr msg){

  true_state_ = *msg;

}

void EstimatorROS::compFiltCallback(const geometry_msgs::msg::Vector3Stamped::SharedPtr msg){
  comp_filt_ = *msg;
}

void EstimatorROS::gnssVelCallback(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
{
  // Rename parameter here for clarity
  double ground_speed_threshold = params_.get_double("gps_ground_speed_threshold");

  double v_n = msg->twist.linear.x;
  double v_e = msg->twist.linear.y;
  double v_d = msg->twist.linear.z;
  //  double v_d = msg->twist.linear.z; // This variable was unused.
  double ground_speed = sqrt(v_n * v_n + v_e * v_e);
  double course =
    atan2(v_e, v_n); //Does this need to be in a specific range? All uses seem to accept anything.
  input_.gps_Vg = ground_speed;
  input_.gps_vn = v_n;
  input_.gps_ve = v_e;
  input_.gps_vd = v_d;
  if (ground_speed > ground_speed_threshold)
    input_.gps_course = course;
}

void EstimatorROS::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  input_.accel_x = msg->linear_acceleration.x;
  input_.accel_y = msg->linear_acceleration.y;
  input_.accel_z = msg->linear_acceleration.z;

  input_.gyro_x = msg->angular_velocity.x;
  input_.gyro_y = msg->angular_velocity.y;
  input_.gyro_z = msg->angular_velocity.z;
}

void EstimatorROS::baroAltCallback(const rosflight_msgs::msg::Barometer::SharedPtr msg)
{
  // For readability, declare the parameters here
  double rho = params_.get_double("rho");
  double gravity = params_.get_double("gravity");
  double gate_gain_constant = params_.get_double("baro_measurement_gate");
  double baro_calib_count = params_.get_int("baro_calibration_count");

  new_baro_ = true;

  if (armed_first_time_ && !baro_init_) {
    if (baro_count_ < baro_calib_count) {
      init_static_ += msg->pressure;
      init_static_vector_.push_back(msg->pressure);
      input_.static_pres = 0;
      baro_count_ += 1;
    } else {
      init_static_ = std::accumulate(init_static_vector_.begin(), init_static_vector_.end(), 0.0)
        / init_static_vector_.size();
      baro_init_ = true;
      // saveParameter("baro_calibration_val", init_static_);

      //Check that it got a good calibration.
      std::sort(init_static_vector_.begin(), init_static_vector_.end());
      float q1 = (init_static_vector_[24] + init_static_vector_[25]) / 2.0;
      float q3 = (init_static_vector_[74] + init_static_vector_[75]) / 2.0;
      float IQR = q3 - q1;
      float upper_bound = q3 + 2.0 * IQR;
      float lower_bound = q1 - 2.0 * IQR;
      for (int i = 0; i < baro_calib_count; i++) {
        if (init_static_vector_[i] > upper_bound) {
          baro_init_ = false;
          baro_count_ = 0;
          init_static_vector_.clear();
          RCLCPP_WARN(this->get_logger(), "Bad baro calibration. Recalibrating");
          break;
        } else if (init_static_vector_[i] < lower_bound) {
          baro_init_ = false;
          baro_count_ = 0;
          init_static_vector_.clear();
          RCLCPP_WARN(this->get_logger(), "Bad baro calibration. Recalibrating");
          break;
        }
      }
    }
  } else {
    float static_pres_old = input_.static_pres;
    input_.static_pres = -msg->pressure + init_static_;

    float gate_gain = gate_gain_constant * rho * gravity;
    if (input_.static_pres < static_pres_old - gate_gain) {
      input_.static_pres = static_pres_old - gate_gain;
    } else if (input_.static_pres > static_pres_old + gate_gain) {
      input_.static_pres = static_pres_old + gate_gain;
    }
  }
}

void EstimatorROS::magnetometerCallback(const sensor_msgs::msg::MagneticField::SharedPtr msg)
{
  input_.mag_x = msg->magnetic_field.x;
  input_.mag_y = msg->magnetic_field.y;
  input_.mag_z = msg->magnetic_field.z;
}

void EstimatorROS::statusCallback(const rosflight_msgs::msg::Status::SharedPtr msg)
{
  if (!armed_first_time_ && msg->armed) armed_first_time_ = true;
}

} // namespace roscopter
