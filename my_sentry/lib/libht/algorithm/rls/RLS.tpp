#pragma once

#include <cmath>
#include <limits>

namespace RLS
{
template <std::size_t dim>
  requires(dim > 0)
RLS<dim>::RLS(float delta, float lambda)
    : lambda_(lambda),
      delta_(delta),
      defaultParamsVector_(Eigen::Matrix<float, dim, 1>::Zero()),
      is_param_valid_(isConfigValid(delta, lambda))
{
  this->reset();
}

template <std::size_t dim>
  requires(dim > 0)
RLS<dim>::RLS(float delta, float lambda, const Eigen::Matrix<float, dim, 1>& initParam)
    : RLS(delta, lambda)
{
  if (!initParam.allFinite())
  {
    is_param_valid_ = false;
    this->reset();
    return;
  }

  defaultParamsVector_ = initParam;
  this->reset();
}

template <std::size_t dim>
  requires(dim > 0)
void RLS<dim>::reset()
{
  transMatrix_ = Eigen::Matrix<float, dim, dim>::Zero();
  if (is_param_valid_)
  {
    transMatrix_.diagonal().setConstant(delta_);
  }
  gainVector_ = Eigen::Matrix<float, dim, 1>::Zero();
  paramsVector_ = defaultParamsVector_;
  output_ = 0.0f;
}

template <std::size_t dim>
  requires(dim > 0)
ErrorCode RLS<dim>::update(const Eigen::Matrix<float, dim, 1>& sampleVector,
                           float actualOutput)
{
  if (!is_param_valid_ || !sampleVector.allFinite() || !std::isfinite(actualOutput))
  {
    return ErrorCode::ARG_ERR;
  }

  // 1. 计算增益向量 (Gain Vector)
  // 分母 denominator = lambda + phi^T * P * phi
  const Eigen::Matrix<float, dim, 1> covariance_times_sample =
      transMatrix_ * sampleVector;
  const float denominator = lambda_ + sampleVector.dot(covariance_times_sample);
  if (!std::isfinite(denominator) || denominator <= std::numeric_limits<float>::min())
  {
    return ErrorCode::OUT_OF_RANGE;
  }

  const Eigen::Matrix<float, dim, 1> next_gain = covariance_times_sample / denominator;

  // 2. 更新参数向量 (Parameter Vector)
  // 预测误差 error = y - phi^T * theta_old
  const float prediction_error = actualOutput - sampleVector.dot(paramsVector_);
  const Eigen::Matrix<float, dim, 1> next_params =
      paramsVector_ + next_gain * prediction_error;
  const float next_output = sampleVector.dot(next_params);

  // 3. 更新协方差矩阵 (Covariance Matrix)
  // P_new = (P_old - K * phi^T * P_old) / lambda
  Eigen::Matrix<float, dim, dim> next_covariance =
      (transMatrix_ - next_gain * sampleVector.transpose() * transMatrix_) / lambda_;
  next_covariance = (0.5f * (next_covariance + next_covariance.transpose())).eval();

  if (!std::isfinite(prediction_error) || !next_gain.allFinite() ||
      !next_params.allFinite() || !std::isfinite(next_output) ||
      !next_covariance.allFinite())
  {
    return ErrorCode::OUT_OF_RANGE;
  }

  gainVector_ = next_gain;
  paramsVector_ = next_params;
  output_ = next_output;
  transMatrix_ = next_covariance;
  return ErrorCode::OK;
}

template <std::size_t dim>
  requires(dim > 0)
ErrorCode RLS<dim>::setParamVector(const Eigen::Matrix<float, dim, 1>& updatedParams)
{
  if (!updatedParams.allFinite())
  {
    is_param_valid_ = false;
    return ErrorCode::ARG_ERR;
  }

  defaultParamsVector_ = updatedParams;
  is_param_valid_ = isConfigValid(delta_, lambda_);
  this->reset();
  return is_param_valid_ ? ErrorCode::OK : ErrorCode::ARG_ERR;
}

template <std::size_t dim>
  requires(dim > 0)
bool RLS<dim>::isConfigValid(float delta, float lambda)
{
  return std::isfinite(delta) && delta > 0.0f && std::isfinite(lambda) && lambda > 0.0f &&
         lambda <= 1.0f;
}
}  // namespace RLS
