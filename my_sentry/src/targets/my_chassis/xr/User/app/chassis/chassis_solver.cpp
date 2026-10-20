#include "chassis_solver.hpp"

#include <algorithm>
#include <cmath>

namespace app::chassis {
namespace {
constexpr float kEpsilon = 1.0e-6f;
constexpr float kGravity = 9.81f;
constexpr float kPi = 3.14159265358979323846f;

bool solve3x3(float a[3][3], const float b[3], float x[3])
{
    float augmented[3][4] {};
    for (int row = 0; row < 3; ++row) {
        for (int col = 0; col < 3; ++col) augmented[row][col] = a[row][col];
        augmented[row][3] = b[row];
    }
    for (int pivot = 0; pivot < 3; ++pivot) {
        int best = pivot;
        for (int row = pivot + 1; row < 3; ++row) {
            if (std::abs(augmented[row][pivot]) > std::abs(augmented[best][pivot])) best = row;
        }
        if (std::abs(augmented[best][pivot]) < kEpsilon) return false;
        for (int col = pivot; col < 4; ++col) std::swap(augmented[pivot][col], augmented[best][col]);
        const float divisor = augmented[pivot][pivot];
        for (int col = pivot; col < 4; ++col) augmented[pivot][col] /= divisor;
        for (int row = 0; row < 3; ++row) {
            if (row == pivot) continue;
            const float factor = augmented[row][pivot];
            for (int col = pivot; col < 4; ++col) augmented[row][col] -= factor * augmented[pivot][col];
        }
    }
    for (int i = 0; i < 3; ++i) x[i] = augmented[i][3];
    return true;
}
}

ChassisSolver::ChassisSolver(const Config& config) { set_config(config); }

void ChassisSolver::set_config(const Config& config)
{
    config_ = config;
    auto estimator_geometry = config_.kinematics;
    // Estimation and allocation need the unscaled matrix even when command
    // generation enables a maximum wheel speed.
    estimator_geometry.max_wheel_speed = 0.0f;
    kinematics_.set_config(estimator_geometry);
    config_.minimum_wheel_confidence = std::clamp(config_.minimum_wheel_confidence, 0.001f, 1.0f);
    config_.residual_reference_speed = std::max(config_.residual_reference_speed, 0.001f);
    config_.residual_threshold = std::max(config_.residual_threshold,
                                          config_.residual_deadband + 0.001f);
}

void ChassisSolver::reset()
{
    estimate_ = {};
    estimate_.wheel_confidence = {1.0f, 1.0f, 1.0f, 1.0f};
    smooth_omega_ = 0.0f;
    smooth_omega_acceleration_ = 0.0f;
    lock_timer_ = 0.0f;
}

const ChassisSolver::Estimate& ChassisSolver::update(const Feedback& feedback, float dt)
{
    if (dt <= 0.0f || !std::isfinite(dt)) return estimate_;

    ChassisVelocity prediction = estimate_.body_velocity;
    if (feedback.imu.online) {
        const float c = std::cos(config_.imu_yaw);
        const float s = std::sin(config_.imu_yaw);
        const float ax_sensor = c * feedback.imu.acceleration[0] - s * feedback.imu.acceleration[1];
        const float ay_sensor = s * feedback.imu.acceleration[0] + c * feedback.imu.acceleration[1];
        const float raw_omega = feedback.imu.gyro[2];
        const float observer_gain = std::max(config_.gyro_observer_frequency, 0.0f);
        const float alpha = std::clamp(observer_gain * dt, 0.0f, 1.0f);
        const float old_omega = smooth_omega_;
        smooth_omega_ += alpha * (raw_omega - smooth_omega_);
        smooth_omega_acceleration_ = (smooth_omega_ - old_omega) / dt;

        // a_sensor = a_centre + alpha x r + omega x (omega x r).
        const float centripetal_x = -smooth_omega_ * smooth_omega_ * config_.imu_offset_x;
        const float centripetal_y = -smooth_omega_ * smooth_omega_ * config_.imu_offset_y;
        const float tangential_x = -smooth_omega_acceleration_ * config_.imu_offset_y;
        const float tangential_y =  smooth_omega_acceleration_ * config_.imu_offset_x;
        estimate_.body_acceleration.vx = ax_sensor - centripetal_x - tangential_x
                                       + smooth_omega_ * estimate_.body_velocity.vy;
        estimate_.body_acceleration.vy = ay_sensor - centripetal_y - tangential_y
                                       - smooth_omega_ * estimate_.body_velocity.vx;
        estimate_.body_acceleration.wz = smooth_omega_acceleration_;
        prediction.vx += estimate_.body_acceleration.vx * dt;
        prediction.vy += estimate_.body_acceleration.vy * dt;
        prediction.wz = smooth_omega_;
    } else {
        estimate_.body_acceleration = {};
    }

    const WheelVector predicted_wheels = kinematics_.inverse(prediction);
    float minimum_residual = 1.0e9f;
    std::array<float, 4> residual {};
    bool any_wheel = false;
    for (std::size_t i = 0; i < 4; ++i) {
        if (!feedback.wheel_online[i]) {
            estimate_.wheel_confidence[i] = 0.0f;
            continue;
        }
        any_wheel = true;
        residual[i] = std::abs(feedback.wheel_speed[i] - predicted_wheels[i]) /
                      std::max(std::abs(predicted_wheels[i]), config_.residual_reference_speed);
        minimum_residual = std::min(minimum_residual, residual[i]);
    }

    estimate_.slipping = false;
    for (std::size_t i = 0; i < 4; ++i) {
        if (!feedback.wheel_online[i]) continue;
        const float excess = std::max(residual[i] - minimum_residual - config_.residual_deadband, 0.0f);
        const float indicator = std::clamp(excess /
            (config_.residual_threshold - config_.residual_deadband) * config_.slip_sensitivity,
            0.0f, 1.0f);
        estimate_.wheel_confidence[i] = 1.0f -
            (1.0f - config_.minimum_wheel_confidence) * indicator;
        estimate_.slipping = estimate_.slipping || indicator > 0.5f;
    }

    if (any_wheel) {
        // Weighted least squares of H*x = wheel_speed. H is obtained from the
        // same pure kinematics module, keeping geometry in one place.
        float normal[3][3] {};
        float rhs[3] {};
        for (int wheel = 0; wheel < 4; ++wheel) {
            if (!feedback.wheel_online[wheel]) continue;
            ChassisVelocity unit[3] {{1,0,0}, {0,1,0}, {0,0,1}};
            float h[3] {kinematics_.inverse(unit[0])[wheel],
                        kinematics_.inverse(unit[1])[wheel],
                        kinematics_.inverse(unit[2])[wheel]};
            const float weight = estimate_.wheel_confidence[wheel] /
                                 std::max(config_.wheel_measurement_noise, 0.001f);
            for (int row = 0; row < 3; ++row) {
                rhs[row] += weight * h[row] * feedback.wheel_speed[wheel];
                for (int col = 0; col < 3; ++col) normal[row][col] += weight * h[row] * h[col];
            }
        }
        float measured[3] {};
        if (solve3x3(normal, rhs, measured)) {
            const float process = std::max(config_.velocity_process_noise, 0.001f);
            const float blend = feedback.imu.online
                ? std::clamp(process / (process + config_.wheel_measurement_noise), 0.05f, 0.95f)
                : 1.0f;
            prediction.vx += blend * (measured[0] - prediction.vx);
            prediction.vy += blend * (measured[1] - prediction.vy);
            if (!feedback.imu.online) prediction.wz = measured[2];
        }
    }
    estimate_.body_velocity = prediction;
    estimate_.world_velocity = ChassisKinematics::body_to_world(prediction, feedback.yaw_to_world);

    if (config_.automatic_gravity_compensation && feedback.imu.online) {
        const float tilt = std::acos(std::clamp(std::cos(feedback.roll) *
                                                std::cos(feedback.pitch), 0.0f, 1.0f));
        if (tilt >= config_.slope_activation_degrees * kPi / 180.0f) {
            // Oppose the projection of gravity on the chassis XY plane.
            gravity_force_x_ = config_.vehicle_mass * kGravity * std::sin(feedback.pitch);
            gravity_force_y_ = -config_.vehicle_mass * kGravity *
                               std::sin(feedback.roll) * std::cos(feedback.pitch);
        } else {
            gravity_force_x_ = gravity_force_y_ = 0.0f;
        }
    }
    return estimate_;
}

ChassisSolver::WheelVector ChassisSolver::allocate_torque(
    const ChassisVelocity& target_world, const std::array<float, 4>& gravity_weight) const
{
    ChassisVelocity target = target_world;
    target.vx += gravity_force_x_;
    target.vy += gravity_force_y_;

    // Minimum weighted-norm solution: f = W H (H' W H)^-1 target.
    float h[4][3] {};
    const ChassisVelocity unit[3] {{1,0,0}, {0,1,0}, {0,0,1}};
    for (int axis = 0; axis < 3; ++axis) {
        const auto column = kinematics_.inverse(unit[axis]);
        for (int wheel = 0; wheel < 4; ++wheel) h[wheel][axis] = column[wheel];
    }
    float normal[3][3] {};
    for (int wheel = 0; wheel < 4; ++wheel) {
        const float weight = std::clamp(gravity_weight[wheel], 0.05f, 1.0f) *
                             std::max(estimate_.wheel_confidence[wheel], 0.001f);
        for (int row = 0; row < 3; ++row)
            for (int col = 0; col < 3; ++col)
                normal[row][col] += weight * h[wheel][row] * h[wheel][col];
    }
    const float demand[3] {target.vx, target.vy, target.wz};
    float intermediate[3] {};
    WheelVector result {};
    if (!solve3x3(normal, demand, intermediate)) return result;
    for (int wheel = 0; wheel < 4; ++wheel) {
        const float weight = std::clamp(gravity_weight[wheel], 0.05f, 1.0f) *
                             std::max(estimate_.wheel_confidence[wheel], 0.001f);
        for (int axis = 0; axis < 3; ++axis) result[wheel] += weight * h[wheel][axis] * intermediate[axis];
    }
    return result;
}

void ChassisSolver::set_gravity_compensation(float force_x, float force_y)
{
    gravity_force_x_ = force_x;
    gravity_force_y_ = force_y;
}

void ChassisSolver::update_lock_state(const ChassisVelocity& target, float dt)
{
    const float target_norm = std::hypot(std::hypot(target.vx, target.vy), target.wz);
    const float speed = std::hypot(estimate_.body_velocity.vx, estimate_.body_velocity.vy);
    if (target_norm < config_.lock_target_threshold && speed < config_.lock_speed_threshold) {
        lock_timer_ = std::min(lock_timer_ + std::max(dt, 0.0f), config_.lock_delay_seconds);
    } else {
        lock_timer_ = 0.0f;
    }
    estimate_.locked = lock_timer_ >= config_.lock_delay_seconds;
}

} // namespace app::chassis
