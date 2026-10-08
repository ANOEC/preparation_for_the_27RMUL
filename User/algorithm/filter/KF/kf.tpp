#pragma once

#include "kf.hpp"

namespace algorithm::filter::kf {

template <int Nx, int Nu, int Ny, bool USE_JOSEPH_FORM>
KF<Nx, Nu, Ny, USE_JOSEPH_FORM>::KF()
{
    x_.setZero();
    P_.setIdentity();
    F_.setIdentity();
    Q_.setZero();
    H_.setZero();
    R_.setIdentity();
    B_.setZero();
    I_.setIdentity();
}

template <int Nx, int Nu, int Ny, bool USE_JOSEPH_FORM>
KF<Nx, Nu, Ny, USE_JOSEPH_FORM>::KF(const StateVec& x0, const Matxx& P0)
    : x_(x0)
    , P_(P0)
{
    F_.setIdentity();
    Q_.setZero();
    H_.setZero();
    R_.setIdentity();
    B_.setZero();
    I_.setIdentity();
}

/**
 * @brief 设置系统矩阵
 */
template <int Nx, int Nu, int Ny, bool USE_JOSEPH_FORM>
void KF<Nx, Nu, Ny, USE_JOSEPH_FORM>::set_matrices(const Matxx& F, const Matxu& B, const Matyx& H, const Matxx& Q, const Matyy& R)
{
    F_ = F;
    B_ = B;
    H_ = H;
    Q_ = Q;
    R_ = R;
}

template <int Nx, int Nu, int Ny, bool USE_JOSEPH_FORM>
void KF<Nx, Nu, Ny, USE_JOSEPH_FORM>::predict(const ControlVec& u)
{
    // x_pred = F * x + B * u
    StateVec x_next;
    x_next.noalias() = F_ * x_;
    x_next.noalias() += B_ * u;
    x_ = x_next;
}

template <int Nx, int Nu, int Ny, bool USE_JOSEPH_FORM>
void KF<Nx, Nu, Ny, USE_JOSEPH_FORM>::prior_constraint()
{
    // P_pred = F * P * F^T + Q
    FP_.noalias() = F_ * P_;
    P_ = Q_;
    P_.noalias() += FP_ * F_.transpose();
}

template <int Nx, int Nu, int Ny, bool USE_JOSEPH_FORM>
bool KF<Nx, Nu, Ny, USE_JOSEPH_FORM>::update_K()
{
    M_.noalias() = P_ * H_.transpose();
    S_ = R_;
    S_.noalias() += H_ * M_;

    // 确保S是对称正定的，增加数值稳定性
    S_ = 0.5f * (S_ + S_.transpose());
    S_.diagonal().array() += 1e-6f;

    auto llt_solver = S_.llt();
    if (llt_solver.info() != Eigen::Success) {
        return false;
    }

    Kt_ = llt_solver.solve(M_.transpose());
    K_.noalias() = Kt_.transpose();
    return true;
}

template <int Nx, int Nu, int Ny, bool USE_JOSEPH_FORM>
void KF<Nx, Nu, Ny, USE_JOSEPH_FORM>::optimal_estimate(const MeasVec& z)
{
    MeasVec y = z - H_ * x_;
    x_.noalias() += K_ * y;
}

template <int Nx, int Nu, int Ny, bool USE_JOSEPH_FORM>
void KF<Nx, Nu, Ny, USE_JOSEPH_FORM>::posterior_constraint()
{
    if constexpr (USE_JOSEPH_FORM) {
        j_cache_.IKH_.noalias() = I_ - K_ * H_;

        j_cache_.IKHP_.noalias() = j_cache_.IKH_ * P_;
        j_cache_.KR_.noalias() = K_ * R_;

        P_.noalias() = j_cache_.IKHP_ * j_cache_.IKH_.transpose();
        P_.noalias() += j_cache_.KR_ * K_.transpose();
    } else {
        P_.noalias() -= K_ * M_.transpose();
        P_ = 0.5f * (P_ + P_.transpose());
    }
}

template <int Nx, int Ny, bool USE_JOSEPH_FORM>
KF<Nx, 0, Ny, USE_JOSEPH_FORM>::KF()
{
    x_.setZero();
    P_.setIdentity();
    F_.setIdentity();
    Q_.setZero();
    H_.setZero();
    R_.setIdentity();
    I_.setIdentity();
}

template <int Nx, int Ny, bool USE_JOSEPH_FORM>
KF<Nx, 0, Ny, USE_JOSEPH_FORM>::KF(const StateVec& x0, const Matxx& P0)
    : x_(x0)
    , P_(P0)
{
    F_.setIdentity();
    Q_.setZero();
    H_.setZero();
    R_.setIdentity();
    I_.setIdentity();
}

template <int Nx, int Ny, bool USE_JOSEPH_FORM>
void KF<Nx, 0, Ny, USE_JOSEPH_FORM>::set_matrices(const Matxx& F, const Matyx& H, const Matxx& Q, const Matyy& R)
{
    F_ = F;
    H_ = H;
    Q_ = Q;
    R_ = R;
}

template <int Nx, int Ny, bool USE_JOSEPH_FORM>
void KF<Nx, 0, Ny, USE_JOSEPH_FORM>::predict()
{
    // x_pred = F * x
    StateVec x_next;
    x_next.noalias() = F_ * x_;
    x_ = x_next;
}

template <int Nx, int Ny, bool USE_JOSEPH_FORM>
void KF<Nx, 0, Ny, USE_JOSEPH_FORM>::prior_constraint()
{
    FP_.noalias() = F_ * P_;
    P_ = Q_;
    P_.noalias() += FP_ * F_.transpose();
}

template <int Nx, int Ny, bool USE_JOSEPH_FORM>
bool KF<Nx, 0, Ny, USE_JOSEPH_FORM>::update_K()
{
    M_.noalias() = P_ * H_.transpose();
    S_ = R_;
    S_.noalias() += H_ * M_;

    // 确保S是对称正定的，增加数值稳定性
    S_ = 0.5f * (S_ + S_.transpose());
    S_.diagonal().array() += 1e-6f;

    auto llt_solver = S_.llt();
    if (llt_solver.info() != Eigen::Success) {
        return false;
    }

    Kt_ = llt_solver.solve(M_.transpose());
    K_.noalias() = Kt_.transpose();
    return true;
}

template <int Nx, int Ny, bool USE_JOSEPH_FORM>
void KF<Nx, 0, Ny, USE_JOSEPH_FORM>::optimal_estimate(const MeasVec& z)
{
    MeasVec y = z - H_ * x_;
    x_.noalias() += K_ * y;
}

template <int Nx, int Ny, bool USE_JOSEPH_FORM>
void KF<Nx, 0, Ny, USE_JOSEPH_FORM>::posterior_constraint()
{
    if constexpr (USE_JOSEPH_FORM) {
        j_cache_.IKH_.noalias() = I_ - K_ * H_;

        j_cache_.IKHP_.noalias() = j_cache_.IKH_ * P_;
        j_cache_.KR_.noalias() = K_ * R_;

        P_.noalias() = j_cache_.IKHP_ * j_cache_.IKH_.transpose();
        P_.noalias() += j_cache_.KR_ * K_.transpose();
    } else {
        P_.noalias() -= K_ * M_.transpose();
        P_ = 0.5f * (P_ + P_.transpose());
    }
}
}