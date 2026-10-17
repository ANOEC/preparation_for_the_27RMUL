#pragma once

#include <Eigen/Core>

namespace algorithm::control::lqr {
template <int Nx, int Nu>
class LQR {
public:
    using StateVec = Eigen::Vector<float, Nx>;
    using InputVec = Eigen::Vector<float, Nu>;
    using GainMat = Eigen::Matrix<float, Nu, Nx>;

    explicit LQR(const GainMat& K_data);

    void set_feedforward(InputVec u_ff) { u_ff_ = u_ff; }
    void set_gain(const GainMat& K) { K_ = K; }

    InputVec update(const StateVec& x_now, const StateVec& x_ref);
    InputVec update(const StateVec& x_error);

protected:
    GainMat K_;
    InputVec u_ff_ = InputVec::Zero();
};
}

#include "lqr.tpp"
