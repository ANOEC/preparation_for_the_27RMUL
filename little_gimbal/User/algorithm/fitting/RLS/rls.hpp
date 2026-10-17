#pragma once

#include <Eigen/Eigen>
#include <cassert>
#include <cstdint>
#include <cstddef>

namespace algorithm::fitting::rls {
template <std::size_t dim>
class RLS {
public:
    /**
     * @brief Delete the default constructor
     */
    RLS() = delete;

    /**
     * @brief The constructor
     * @param delta_ The intialized non-singular value of the transfer matrix
     * @param lambda_ The forgotten index
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
     * @retval paramsVector 更新后的参数估计值 (theta)
     */
    const Eigen::Matrix<float, dim, 1>& update(const Eigen::Matrix<float, dim, 1>& sampleVector, float actualOutput);

    /**
     * @brief Set the default regression parameters
     * @param updatedParams
     * @retval None
     */
    void setParamVector(const Eigen::Matrix<float, dim, 1>& updatedParams);

    /**
     * @brief The getter function of the params vector
     * @param None
     * @retval paramsVector
     */
    const Eigen::Matrix<float, dim, 1>& getParamsVector() const { return paramsVector_; }

    /**
     * @brief The getter function of the output vector
     * @param None
     * @retval The estimated / filterd output of the RLS module
     */
    float getOutput() const { return output_; }

private:
    /* RLS 调优参数 */
    uint32_t dimension_; // 参数空间的维度
    float lambda_; // 遗忘因子 (0 < lambda <= 1). 决定旧数据被遗忘的速度. 越接近 1 适应越慢，但稳定性越高.
    float delta_; // 协方差矩阵 P 的初始化值. P_0 = delta * I. delta 越大初始收敛越快，但可能产生超调.

    /* RLS 状态矩阵 */
    Eigen::Matrix<float, dim, dim> transMatrix_; // 协方差矩阵 (P)
    Eigen::Matrix<float, dim, 1> gainVector_; // 卡尔曼增益向量 (K)
    Eigen::Matrix<float, dim, 1> paramsVector_; // 参数估计向量 (theta)
    Eigen::Matrix<float, dim, 1> defaultParamsVector_; // 重置用的默认参数
    float output_;
};

}

#include "rls.tpp"
