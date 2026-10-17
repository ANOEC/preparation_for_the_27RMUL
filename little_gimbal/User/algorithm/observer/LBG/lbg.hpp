#pragma once

#include <Eigen/Dense>
#include <cmath>
#include <functional>

namespace algorithm::observer::lbg {

/**
 * @tparam Nx 状态量维度 (State Dimension)
 * @tparam Nu 控制量维度 (Input Dimension)
 * @tparam Ny 观测量维度 (Output Dimension)
 */
template <size_t Nx, size_t Nu, size_t Ny>
class LBG {
public:
    using StateVec = Eigen::Matrix<float, Nx, 1>;
    using InputVec = Eigen::Matrix<float, Nu, 1>;
    using MeasVec = Eigen::Matrix<float, Ny, 1>;
    using GainMat = Eigen::Matrix<float, Nx, Ny>;

    // 预测模型: f(x, u) -> x_next
    using SystemFunc = std::function<StateVec(const StateVec&, const InputVec&)>;
    // 观测模型: h(x) -> y_pred
    using MeasFunc = std::function<MeasVec(const StateVec&)>;

public:
    LBG() = default;

    LBG(SystemFunc f, MeasFunc h, const GainMat& L, const StateVec& x0 = StateVec::Zero())
        : system_func_(f)
        , meas_func_(h)
        , L_(L)
        , x_hat_(x0)
    {
    }

    void set_observer(SystemFunc f, MeasFunc h, const GainMat& L, const StateVec& x0 = StateVec::Zero())
    {
        system_func_ = f;
        meas_func_ = h;
        L_ = L;
        x_hat_ = x0;
    }

    /**
     * @brief 观测器更新
     * @param u 当前控制输入
     * @param y 当前传感器测量值
     */
    const StateVec& update(const InputVec& u, const MeasVec& y)
    {
        if (system_func_ == nullptr || meas_func_ == nullptr) {
            x_hat_ = StateVec::Zero();
            return x_hat_;
        }

        // 1. 预测 (Predict): x_pred = f(x_hat, u)
        StateVec x_pred = system_func_(x_hat_, u);

        // 2. 观测估计 (Estimate Measurement): y_hat = h(x_pred)
        MeasVec y_hat = meas_func_(x_pred);

        // 3. 校正 (Correct): x_new = x_pred + L * (y - y_hat)
        x_hat_ = x_pred + L_ * (y - y_hat);

        return x_hat_;
    }

    const StateVec& get_state() const { return x_hat_; }

    void set_gain(const GainMat& L) { L_ = L; }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
private:
    SystemFunc system_func_;
    MeasFunc meas_func_;
    GainMat L_;
    StateVec x_hat_;
};
}