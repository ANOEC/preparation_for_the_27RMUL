#pragma once

#include "Eigen/Core"
#include <Eigen/Dense>
#include <type_traits>

namespace algorithm::filter::kf {

/**
 * @tparam Nx 状态维度 (State Dimension)
 * @tparam Nu 控制维度 (Control Dimension)
 * @tparam Ny 观测维度 (Measurement Dimension)
 */
template <int Nx, int Nu, int Ny, bool USE_JOSEPH_FORM = false>
class KF {
public:
    using StateVec = Eigen::Matrix<float, Nx, 1>;
    using ControlVec = Eigen::Matrix<float, Nu, 1>;
    using MeasVec = Eigen::Matrix<float, Ny, 1>;

    using Matxx = Eigen::Matrix<float, Nx, Nx>; // F, P, Q
    using Matxy = Eigen::Matrix<float, Nx, Ny>; // K
    using Matyx = Eigen::Matrix<float, Ny, Nx>; // H
    using Matyy = Eigen::Matrix<float, Ny, Ny>; // S, R
    using Matxu = Eigen::Matrix<float, Nx, Nu>; // B

public:
    KF();

    KF(const StateVec& x0, const Matxx& P0);

    void set_matrices(const Matxx& F, const Matxu& B, const Matyx& H, const Matxx& Q, const Matyy& R);

    virtual void predict(const ControlVec& u);

    virtual void prior_constraint();

    virtual bool update_K();

    virtual void optimal_estimate(const MeasVec& z);

    virtual void posterior_constraint();

    const StateVec& state() const { return x_; }
    const Matxx& covariance() const { return P_; }

    void set_F(const Matxx& F) { F_ = F; }
    void set_H(const Matyx& H) { H_ = H; }
    void set_Q(const Matxx& Q) { Q_ = Q; }
    void set_R(const Matyy& R) { R_ = R; }
    void set_B(const Matxu& B) { B_ = B; }

protected:
    // 状态向量与协方差
    StateVec x_;
    Matxx P_;

    // 系统矩阵
    Matxx F_; // 状态转移矩阵
    Matxu B_; // 控制输入矩阵
    Matyx H_; // 观测矩阵

    // 噪声矩阵
    Matxx Q_; // 过程噪声协方差
    Matyy R_; // 测量噪声协方差

    Matxy K_;

    Matxx I_; // 单位矩阵

    // 用于中间计算的矩阵，避免重复分配内存
    Matxy M_;
    Matyy S_;
    Matyx Kt_;
    Matxx FP_;

    struct JosephCache {
        Matxx IKH_;
        Matxx IKHP_;
        decltype(K_) KR_;
    };

    struct EmptyCache { };

    [[no_unique_address]] std::conditional_t<
        USE_JOSEPH_FORM,
        JosephCache,
        EmptyCache> j_cache_;
};

template <int Nx, int Ny, bool USE_JOSEPH_FORM>
class KF<Nx, 0, Ny, USE_JOSEPH_FORM> {
public:
    using StateVec = Eigen::Matrix<float, Nx, 1>;
    using MeasVec = Eigen::Matrix<float, Ny, 1>;

    using Matxx = Eigen::Matrix<float, Nx, Nx>; // F, P, Q
    using Matxy = Eigen::Matrix<float, Nx, Ny>; // K
    using Matyx = Eigen::Matrix<float, Ny, Nx>; // H
    using Matyy = Eigen::Matrix<float, Ny, Ny>; // S, R

    KF();

    KF(const StateVec& x0, const Matxx& P0);

    void set_matrices(const Matxx& F, const Matyx& H, const Matxx& Q, const Matyy& R);

    virtual void predict();

    virtual void prior_constraint();

    virtual bool update_K();

    virtual void optimal_estimate(const MeasVec& z);

    virtual void posterior_constraint();

    const StateVec& state() const { return x_; }
    const Matxx& covariance() const { return P_; }

    void set_F(const Matxx& F) { F_ = F; }
    void set_H(const Matyx& H) { H_ = H; }
    void set_Q(const Matxx& Q) { Q_ = Q; }
    void set_R(const Matyy& R) { R_ = R; }

protected:
    // 状态向量与协方差
    StateVec x_;
    Matxx P_;

    // 系统矩阵
    Matxx F_; // 状态转移矩阵
    Matyx H_; // 观测矩阵

    // 噪声矩阵
    Matxx Q_; // 过程噪声协方差
    Matyy R_; // 测量噪声协方差

    Matxy K_;
    Matxx I_; // 单位矩阵

    Matxy M_;
    Matyy S_;
    Matyx Kt_;
    Matxx FP_;

    struct JosephCache {
        Matxx IKH_;
        Matxx IKHP_;
        decltype(K_) KR_;
    };

    struct EmptyCache { };

    [[no_unique_address]] std::conditional_t<
        USE_JOSEPH_FORM,
        JosephCache,
        EmptyCache> j_cache_;

    StateVec x_add_temp_;
};
}

#include "kf.tpp"
