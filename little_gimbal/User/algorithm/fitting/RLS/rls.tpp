#pragma once

#include "rls.hpp"

namespace algorithm::fitting::rls {
template <std::size_t dim>
RLS<dim>::RLS(float delta, float lambda)
    : dimension_(dim)
    , lambda_(lambda)
    , delta_(delta)
    , defaultParamsVector_(Eigen::Matrix<float, dim, 1>::Zero())
    , output_(0.0f)
{
    assert(lambda_ > 0.0f && lambda_ <= 1.0f);
    assert(delta_ > 0.0f);
    this->reset();
}

template <std::size_t dim>
RLS<dim>::RLS(float delta, float lambda, const Eigen::Matrix<float, dim, 1>& initParam)
    : RLS(delta, lambda)
{
    defaultParamsVector_ = initParam;
    paramsVector_ = defaultParamsVector_;
}

template <std::size_t dim>
void RLS<dim>::reset()
{
    transMatrix_ = Eigen::Matrix<float, dim, dim>::Identity() * delta_;
    gainVector_ = Eigen::Matrix<float, dim, 1>::Zero();
    paramsVector_ = defaultParamsVector_;
    output_ = 0.0f;
}

template <std::size_t dim>
const Eigen::Matrix<float, dim, 1>& RLS<dim>::update(const Eigen::Matrix<float, dim, 1>& sampleVector, float actualOutput)
{
    // 1. 计算增益向量 (Gain Vector)
    // 分母 denominator = lambda + phi^T * P * phi
    float denominator = lambda_ + (sampleVector.transpose() * transMatrix_ * sampleVector)(0, 0);
    gainVector_ = (transMatrix_ * sampleVector) / denominator;

    // 2. 更新参数向量 (Parameter Vector)
    // 预测误差 error = y - phi^T * theta_old
    float predictionError = actualOutput - (sampleVector.transpose() * paramsVector_)(0, 0);
    paramsVector_ += gainVector_ * predictionError;
    output_ = (sampleVector.transpose() * paramsVector_)(0, 0);

    // 3. 更新协方差矩阵 (Covariance Matrix)
    // P_new = (P_old - K * phi^T * P_old) / lambda
    transMatrix_ = (transMatrix_ - gainVector_ * sampleVector.transpose() * transMatrix_) / lambda_;

    return paramsVector_;
}

template <std::size_t dim>
void RLS<dim>::setParamVector(const Eigen::Matrix<float, dim, 1>& updatedParams)
{
    paramsVector_ = updatedParams;
    defaultParamsVector_ = updatedParams;
}
}
