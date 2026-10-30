#include "../chassis.hpp"
#include "../chassis_config.hpp"
#include "../reference_coordinates.hpp"

#include <cassert>
#include <cmath>

using namespace app::chassis;

namespace {
bool close(float lhs, float rhs, float tolerance = 1.0e-4f)
{
    return std::abs(lhs - rhs) < tolerance;
}
}

int main()
{
    const auto generated_config = MakeChassisConfig();
    assert(close(generated_config.kinematics.wheel_radius,
                 param::chassis::param.omni_wheel_r));
    assert(close(generated_config.kinematics.wheelbase,
                 param::chassis::param.car_length));
    assert(close(generated_config.kinematics.track_width,
                 param::chassis::param.car_wide));
    assert(close(generated_config.solver.imu_offset_y,
                 param::chassis::imu.offset_y));
    assert(close(generated_config.solver.imu_yaw,
                 param::chassis::imu.horizontal_shift *
                     3.14159265358979323846f / 180.0f));
    assert(close(generated_config.body_controller.x.kp,
                 param::chassis::pid::x.kp));
    assert(close(generated_config.body_controller.y.kp,
                 param::chassis::pid::y.kp));
    assert(close(generated_config.body_controller.yaw.kp,
                 param::chassis::pid::z.kp));
    assert(close(generated_config.body_controller.quick_brake_speed_threshold_x,
                 param::chassis::param.brake_x));
    assert(close(generated_config.body_controller.translation_acceleration_limit,
                 param::chassis::param.translation_acceleration));
    assert(close(generated_config.body_controller.quick_brake_acceleration_limit,
                 param::chassis::param.quick_brake_acceleration));
    assert(close(generated_config.wheels[0].kp,
                 param::chassis::pid::motor1.kp));
    assert(close(generated_config.wheels[0].gear_ratio, 1.0f));
    assert(close(generated_config.wheels[3].output_limit,
                 param::chassis::pid::motor4.max_output));
    assert(close(generated_config.follow_kp,
                 param::chassis::pid::follow.kp));
    assert(close(generated_config.follow_velocity_feedforward,
                 param::chassis::pid::follow.velocity_feedforward));
    assert(close(generated_config.maximum_tilt_for_spin,
                 param::chassis::param.stop_spin_angle));

    ChassisKinematics kinematics;
    const ChassisVelocity commanded {1.2f, -0.4f, 2.0f};
    const auto recovered = kinematics.forward(kinematics.inverse(commanded));
    assert(close(commanded.vx, recovered.vx));
    assert(close(commanded.vy, recovered.vy));
    assert(close(commanded.wz, recovered.wz));

    WheelController wheel;
    WheelController::Feedback offline {};
    assert(wheel.update(10.0f, offline, 0.001f).torque == 0.0f);

    BodyVelocityController::Config body_config {};
    body_config.x.kp = 2.0f;
    body_config.x.output_limit = 10.0f;
    body_config.translation_acceleration_limit = 1.0f;
    body_config.sprint_acceleration_limit = 1.0f;
    body_config.sprint_activation_speed = 100.0f;
    body_config.quick_brake_speed_threshold_x = 2.0f;
    body_config.quick_brake_speed_threshold_y = 2.0f;
    BodyVelocityController body_controller(body_config);
    const auto ramped = body_controller.update(
        ChassisVelocity{1.0f, 0.0f, 0.0f}, ChassisVelocity{}, 0.1f);
    assert(close(ramped.target_velocity.vx, 0.1f));
    assert(close(ramped.force_command.vx, 0.2f));

    body_config.translation_acceleration_limit = 10.0f;
    body_config.sprint_acceleration_limit = 10.0f;
    body_controller.set_config(body_config);
    static_cast<void>(body_controller.update(
        ChassisVelocity{3.0f, 0.0f, 0.0f}, ChassisVelocity{}, 0.3f));
    const auto braking = body_controller.update(
        ChassisVelocity{}, ChassisVelocity{2.5f, 0.0f, 0.0f}, 0.01f);
    assert(braking.quick_braking);
    assert(braking.target_velocity.vx < 2.5f);
    assert(braking.force_command.vx < 0.0f);
    const auto released = body_controller.update(
        ChassisVelocity{}, ChassisVelocity{0.05f, 0.0f, 0.0f}, 0.01f);
    assert(!released.quick_braking);

    PowerControl power;
    PowerControl::Input power_input {};
    power_input.referee_online = true;
    power_input.referee_power_limit = 40.0f;
    power_input.referee_buffer_energy = 60.0f;
    power_input.requested_torque.fill(6.0f);
    power_input.maximum_torque.fill(6.0f);
    power_input.wheel_speed.fill(30.0f);
    const auto limited = power.update(power_input);
    assert(power.status().limiting);
    assert(power.predict_power(limited, power_input.wheel_speed) <=
           power.status().budget + 1.0e-3f);

    Chassis chassis;
    Chassis::Command command {};
    command.mode = ChassisMode::Spin;
    chassis.set_command(command);
    const auto& safe = chassis.update(0.001f);
    assert(safe.safe_output);
    for (float torque : safe.motor_torque) assert(torque == 0.0f);

    Chassis controlled(generated_config);
    Chassis::Feedback feedback {};
    feedback.command_link_online = true;
    feedback.imu.online = true;
    for (auto& wheel_feedback : feedback.wheels) {
        wheel_feedback.online = true;
    }
    Chassis::Command drive {};
    drive.vx = 1.0f;
    drive.mode = ChassisMode::Free;
    drive.world_frame = false;
    controlled.set_feedback(feedback);
    controlled.set_command(drive);
    const auto& controlled_output = controlled.update(0.001f);
    assert(!controlled_output.safe_output);
    assert(controlled_output.target_body_velocity.vx > 0.0f);
    assert(controlled_output.target_body_velocity.vx < drive.vx);
    assert(controlled_output.body_force_command.vx > 0.0f);

    // Follow has no angular dead zone: even a one-degree relative error must
    // create a chassis yaw request.
    controlled.reset();
    Chassis::Command small_follow {};
    small_follow.mode = ChassisMode::Follow;
    small_follow.world_frame = false;
    small_follow.follow_yaw_error =
        1.0f * 3.14159265358979323846f / 180.0f;
    controlled.set_feedback(feedback);
    controlled.set_command(small_follow);
    const auto& small_follow_output = controlled.update(0.001f);
    assert(small_follow_output.target_body_velocity.wz < 0.0f);

    // Positive relative big-yaw velocity requests immediate chassis rotation
    // in the same direction before a sizeable angle error accumulates.
    controlled.reset();
    Chassis::Command moving_follow {};
    moving_follow.mode = ChassisMode::Follow;
    moving_follow.world_frame = false;
    moving_follow.follow_yaw_velocity = 1.0f;
    controlled.set_feedback(feedback);
    controlled.set_command(moving_follow);
    const auto& moving_follow_output = controlled.update(0.001f);
    assert(moving_follow_output.target_body_velocity.wz > 0.0f);

    const float half_pi = 0.5f * 3.14159265358979323846f;
    assert(close(ReferenceFollowYawError(half_pi), -half_pi));

    // GAME uses only the big-yaw offset. A +90-degree big-yaw angle rotates
    // a forward command into +body-Y with the reference matrix.
    const float game_deviation =
        ReferenceTranslationDeviation(half_pi, 0.0f, true);
    const auto game_translation =
        ReferenceTranslationToBody(1.0f, 0.0f, game_deviation);
    assert(close(game_translation.vx, 0.0f));
    assert(close(game_translation.vy, 1.0f));

    // HAND additionally applies the small-yaw command angle. Equal +90-degree
    // angles cancel, so forward remains body-forward.
    const float hand_deviation =
        ReferenceTranslationDeviation(half_pi, half_pi, false);
    const auto hand_translation =
        ReferenceTranslationToBody(1.0f, 0.0f, hand_deviation);
    assert(close(hand_translation.vx, 1.0f));
    assert(close(hand_translation.vy, 0.0f));
}
