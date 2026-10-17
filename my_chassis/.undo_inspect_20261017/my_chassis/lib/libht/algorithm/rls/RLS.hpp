#pragma once

#include <Eigen/Eigen>
#include <cstddef>

#include "error_code.hpp"

namespace RLS
{
using ErrorCode = Algorithm::ErrorCode;

template <std::size_t dim>
  requires(dim > 0)
class RLS
{
 public:
  /**
   * @brief Delete the default constructor
   */
  RLS() = delete;

  /**
   * @brief The constructor
   * @param delta Initial covariance scale
   * @param lambda Forgetting factor
   */
  RLS(float delta, float lambda);

  RLS(float delta, float lambda, const Eigen::Matrix<float, dim, 1>& initParam);

  /**
   * @brief Reset the RLS module
   * @retval None
   */
  void reset();

  /**
   * @brief 执行递归最小二乘法 (RLS) 更新迭代
   * @details
   * 实现 RLS 更新算法：
   * 1. 计算增益向量: K = (P * phi) / (lambda + phi^T * P * phi)
   * 2. 更新参数估计: theta = theta + K * (y - phi^T * theta)
   * 3. 更新协方差矩阵: P = (P - K * phi^T * P) / lambda
   *
   * @param sampleVector 输入回归向量 (phi) [n x 1]
   * @param actualOutput 实际系统输出观测值 (y)
   * @retval ErrorCode 更新结果
   */
  ErrorCode update(const Eigen::Matrix<float, dim, 1>& sampleVector, float actualOutput);

  /**
   * @brief Set the default regression parameters and reinitialize the estimator
   * @param updatedParams
   * @retval ErrorCode 参数设置结果
   */
  ErrorCode setParamVector(const Eigen::Matrix<float, dim, 1>& updatedParams);

  /**
   * @brief The getter function of the params vector
   * @param None
   * @retval paramsVector
   */
  const Eigen::Matrix<float, dim, 1>& getParamsVector() const { return paramsVector_; }

  const Eigen::Matrix<float, dim, dim>& getCovarianceMatrix() const
  {
    return transMatrix_;
  }

  /**
   * @brief The getter function of the output vector
   * @param None
   * @retval The estimated output of the RLS module
   */
  float getOutput() const { return output_; }

 private:
  static bool isConfigValid(float delta, float lambda);

  /* RLS 调优参数 */
  float lambda_;  // 遗忘因子 (0 < lambda <= 1). 决定旧数据被遗忘的速度. 越接近 1
                  // 适应越慢，但稳定性越高.
  float delta_;   // 协方差矩阵 P 的初始化值. P_0 = delta * I. delta
                  // 越大初始收敛越快，但可能产生超调.

  /* RLS 状态矩阵 */
  Eigen::Matrix<float, dim, dim> transMatrix_;        // 协方差矩阵 (P)
  Eigen::Matrix<float, dim, 1> gainVector_;           // 卡尔曼增益向量 (K)
  Eigen::Matrix<float, dim, 1> paramsVector_;         // 参数估计向量 (theta)
  Eigen::Matrix<float, dim, 1> defaultParamsVector_;  // 重置用的默认参数
  float output_ = 0.0f;
  bool is_param_valid_ = false;
};
}  // namespace RLS

#include "RLS.tpp"
