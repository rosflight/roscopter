#ifndef ESTIMATOR_EKF_H
#define ESTIMATOR_EKF_H

#include <cassert>
#include <math.h>
#include <tuple>

#include <Eigen/Geometry>
#include <yaml-cpp/yaml.h>

#include "estimator_ros.hpp"

// These declutter code, and incur a minimum time cost when passing these types to functions.
using DynamicModelFuncRef = std::function<Eigen::VectorXf(const Eigen::VectorXf, const Eigen::VectorXf)>;
using MeasurementModelFuncRef = std::function<Eigen::VectorXf(const Eigen::VectorXf, const Eigen::VectorXf)>;
using JacobianFuncRef = std::function<Eigen::MatrixXf(const Eigen::VectorXf, const Eigen::VectorXf)>;

// TODO: make sure this does not allocate mem on the heap.

namespace roscopter
{

class EstimatorEKF : public EstimatorROS
{
public:
  EstimatorEKF();

protected:
  std::tuple<Eigen::MatrixXf, Eigen::VectorXf> measurement_update(Eigen::VectorXf x,
                                                                  Eigen::VectorXf inputs,
                                                                  MeasurementModelFuncRef measurement_model,
                                                                  Eigen::VectorXf y,
                                                                  JacobianFuncRef measurement_jacobian,
                                                                  Eigen::MatrixXf R,
                                                                  Eigen::MatrixXf P);

  std::tuple<Eigen::MatrixXf, Eigen::VectorXf> propagate_model(Eigen::VectorXf x,
                                                               DynamicModelFuncRef dynamic_model,
                                                               JacobianFuncRef jacobian,
                                                               Eigen::VectorXf inputs,
                                                               JacobianFuncRef input_jacobian,
                                                               Eigen::MatrixXf P,
                                                               Eigen::MatrixXf Q,
                                                               Eigen::MatrixXf Q_g,
                                                               float Ts);          

  std::tuple<Eigen::MatrixXf, Eigen::VectorXf> single_measurement_update(float measurement,
                                                                         float mesurement_prediction,
                                                                         float measurement_uncertainty,
                                                                         Eigen::VectorXf measurement_jacobian,
                                                                         Eigen::VectorXf x,
                                                                         Eigen::MatrixXf P);
  
  std::tuple<Eigen::MatrixXf, Eigen::VectorXf> partial_measurement_update(Eigen::VectorXf x,
                                                                  Eigen::VectorXf inputs,
                                                                  MeasurementModelFuncRef measurement_model,
                                                                  Eigen::VectorXf y,
                                                                  JacobianFuncRef measurement_jacobian,
                                                                  Eigen::MatrixXf R,
                                                                  Eigen::MatrixXf P,
                                                                  Eigen::VectorXf gammas);

private:
  virtual void estimate(const Input & input, Output & output) override = 0;
}; 

} // namespace roscopter

#endif // ESTIMATOR_EKF_H
