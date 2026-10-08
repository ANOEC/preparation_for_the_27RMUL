#pragma once

#include "lqr.hpp"

namespace algorithm::control::lqr {
template <int Nx, int Nu>
LQR<Nx, Nu>::LQR(const GainMat& K)
    : K_(K)
{
}

/**
 * @brief 计算控制量 u
 * @param x_now 当前状态数组 (长度 Nx)
 * @param x_ref 目标状态数组 (长度 Nx)
 * @param u_out 输出控制量数组 (长度 Nu)
 */
template <int Nx, int Nu>
LQR<Nx, Nu>::InputVec LQR<Nx, Nu>::update(const StateVec& x_now, const StateVec& x_ref)
{
    StateVec error = x_now - x_ref;

    InputVec u_fb = -K_ * error;
    
    return u_fb + u_ff_;
}

template <int Nx, int Nu>
LQR<Nx, Nu>::InputVec LQR<Nx, Nu>::update(const StateVec& x_error)
{
    InputVec u_fb = -K_ * x_error;

    return u_fb + u_ff_;
}
}
