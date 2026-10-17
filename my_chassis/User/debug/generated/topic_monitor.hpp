// Generated from topic_monitor.toml. DO NOT EDIT.
// This file is intentionally included twice with different mode macros.

#if defined(USER_APP_TOPIC_MONITOR_MEMBERS)
static constexpr std::size_t MONITOR_COUNT = 13;

Monitor<RemoteData> remote_{"/remote", PrintRemote};
Monitor<ChassisCommand> chassis_command_{"/cmd/chassis", PrintChassisCommand};
Monitor<MotorFeedback> motor_1_{"/fb/motor/chassis_motor1", PrintMotorFeedback};
Monitor<MotorFeedback> motor_2_{"/fb/motor/chassis_motor2", PrintMotorFeedback};
Monitor<MotorFeedback> motor_3_{"/fb/motor/chassis_motor3", PrintMotorFeedback};
Monitor<MotorFeedback> motor_4_{"/fb/motor/chassis_motor4", PrintMotorFeedback};
Monitor<ImuVector> gyro_{"bmi088_gyro", PrintImuVector};
Monitor<ImuVector> accel_{"bmi088_accl", PrintImuVector};
Monitor<VqfOutputData> vqf_output_{"vqf_output", PrintVqfOutput};
Monitor<float> torque_1_{"/torque/chassis_motor1", PrintTorque};
Monitor<float> torque_2_{"/torque/chassis_motor2", PrintTorque};
Monitor<float> torque_3_{"/torque/chassis_motor3", PrintTorque};
Monitor<float> torque_4_{"/torque/chassis_motor4", PrintTorque};

MonitorBase* monitors_[MONITOR_COUNT] = {
    &remote_,
    &chassis_command_,
    &motor_1_,
    &motor_2_,
    &motor_3_,
    &motor_4_,
    &gyro_,
    &accel_,
    &vqf_output_,
    &torque_1_,
    &torque_2_,
    &torque_3_,
    &torque_4_,
};

#elif defined(USER_APP_TOPIC_MONITOR_INSTANTIATIONS)
template class TopicDebugApp::Monitor<TopicDebugApp::RemoteData>;
template class TopicDebugApp::Monitor<TopicDebugApp::ChassisCommand>;
template class TopicDebugApp::Monitor<TopicDebugApp::MotorFeedback>;
template class TopicDebugApp::Monitor<TopicDebugApp::ImuVector>;
template class TopicDebugApp::Monitor<TopicDebugApp::VqfOutputData>;
template class TopicDebugApp::Monitor<float>;
#else
#error "Define a topic monitor generation mode before including this file"
#endif
