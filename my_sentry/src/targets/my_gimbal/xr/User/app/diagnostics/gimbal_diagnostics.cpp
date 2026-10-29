#include "app/diagnostics/gimbal_diagnostics.hpp"

#include <atomic>
#include <cstdint>

#include "DJIMotor.hpp"
#include "DMMotor.hpp"
#include "Details/DT7/DT7.hpp"
#include "VQFModule.hpp"
#include "app/remote/remote.hpp"
#include "libxr.hpp"

namespace UserApp::Diagnostics
{
namespace
{

using RemoteData = Module::Remote::DT7::sRemoteInfo;
using GimbalCommand = UserApp::Remote::GimbalCommand;

constexpr uint32_t kFastFreshMs = 100U;
constexpr uint32_t kRemoteFreshMs = 500U;

template <typename T>
class LatestSample
{
 public:
  void Store(const T& value)
  {
    uint32_t sequence = sequence_.load(std::memory_order_relaxed);
    if ((sequence & 1U) != 0U)
    {
      ++sequence;
    }
    sequence_.store(sequence + 1U, std::memory_order_release);
    value_ = value;
    timestamp_ms_ =
        static_cast<uint32_t>(LibXR::Timebase::GetMilliseconds());
    sequence_.store(sequence + 2U, std::memory_order_release);
  }

  bool Load(T& value, uint32_t& age_ms) const
  {
    for (unsigned int attempt = 0; attempt < 8U; ++attempt)
    {
      const uint32_t before = sequence_.load(std::memory_order_acquire);
      if (before == 0U || (before & 1U) != 0U)
      {
        continue;
      }

      const T copy = value_;
      const uint32_t timestamp = timestamp_ms_;
      const uint32_t after = sequence_.load(std::memory_order_acquire);
      if (before == after)
      {
        value = copy;
        age_ms = static_cast<uint32_t>(LibXR::Timebase::GetMilliseconds()) -
                 timestamp;
        return true;
      }
    }
    return false;
  }

 private:
  mutable std::atomic<uint32_t> sequence_{0U};
  T value_{};
  uint32_t timestamp_ms_ = 0U;
};

const char* StateName(UserApp::Remote::ControlState state)
{
  switch (state)
  {
    case UserApp::Remote::ControlState::OFFLINE:
      return "OFFLINE";
    case UserApp::Remote::ControlState::PROTECT:
      return "PROTECT";
    case UserApp::Remote::ControlState::HAND:
      return "HAND";
    case UserApp::Remote::ControlState::GAME:
      return "GAME";
  }
  return "UNKNOWN";
}

const char* Freshness(bool seen, uint32_t age_ms, uint32_t limit_ms)
{
  if (!seen)
  {
    return "MISSING";
  }
  return age_ms <= limit_ms ? "OK" : "STALE";
}

class GimbalDiagnostics
{
 public:
  explicit GimbalDiagnostics(LibXR::RamFS& ramfs)
      : remote_topic_(LibXR::Topic::CreateTopic<RemoteData>("/remote")),
        command_topic_(
            LibXR::Topic::CreateTopic<GimbalCommand>("/cmd/gimbal")),
        imu_topic_(LibXR::Topic::CreateTopic<VqfOutput<float>>("vqf_output")),
        yaw_feedback_topic_(LibXR::Topic::CreateTopic<DJIMotor::SolvedFeedback>(
            "/fb/motor/yaw")),
        big_yaw_feedback_topic_(
            LibXR::Topic::CreateTopic<DMMotor::SolvedFeedback>(
                "/fb/motor/big_yaw")),
        pitch_feedback_topic_(
            LibXR::Topic::CreateTopic<DMMotor::SolvedFeedback>(
                "/fb/motor/pitch")),
        yaw_torque_topic_(LibXR::Topic::CreateTopic<float>("/torque/yaw")),
        big_yaw_torque_topic_(
            LibXR::Topic::CreateTopic<DMMotor::ControlCommand>(
                "/torque/big_yaw")),
        pitch_torque_topic_(
            LibXR::Topic::CreateTopic<DMMotor::ControlCommand>(
                "/torque/pitch")),
        remote_callback_(LibXR::Topic::Callback::Create(OnRemote, this)),
        command_callback_(LibXR::Topic::Callback::Create(OnCommand, this)),
        imu_callback_(LibXR::Topic::Callback::Create(OnImu, this)),
        yaw_feedback_callback_(
            LibXR::Topic::Callback::Create(OnYawFeedback, this)),
        big_yaw_feedback_callback_(
            LibXR::Topic::Callback::Create(OnBigYawFeedback, this)),
        pitch_feedback_callback_(
            LibXR::Topic::Callback::Create(OnPitchFeedback, this)),
        yaw_torque_callback_(
            LibXR::Topic::Callback::Create(OnYawTorque, this)),
        big_yaw_torque_callback_(
            LibXR::Topic::Callback::Create(OnBigYawTorque, this)),
        pitch_torque_callback_(
            LibXR::Topic::Callback::Create(OnPitchTorque, this)),
        diag_command_(LibXR::RamFS::CreateCommand<GimbalDiagnostics*>(
            "diag", RunDiag, this)),
        remote_command_(LibXR::RamFS::CreateCommand<GimbalDiagnostics*>(
            "remote", RunRemote, this)),
        imu_command_(LibXR::RamFS::CreateCommand<GimbalDiagnostics*>(
            "imu", RunImu, this)),
        motors_command_(LibXR::RamFS::CreateCommand<GimbalDiagnostics*>(
            "motors", RunMotors, this)),
        torque_command_(LibXR::RamFS::CreateCommand<GimbalDiagnostics*>(
            "torque", RunTorque, this))
  {
    remote_topic_.RegisterCallback(remote_callback_);
    command_topic_.RegisterCallback(command_callback_);
    imu_topic_.RegisterCallback(imu_callback_);
    yaw_feedback_topic_.RegisterCallback(yaw_feedback_callback_);
    big_yaw_feedback_topic_.RegisterCallback(big_yaw_feedback_callback_);
    pitch_feedback_topic_.RegisterCallback(pitch_feedback_callback_);
    yaw_torque_topic_.RegisterCallback(yaw_torque_callback_);
    big_yaw_torque_topic_.RegisterCallback(big_yaw_torque_callback_);
    pitch_torque_topic_.RegisterCallback(pitch_torque_callback_);

    ramfs.Add(diag_command_);
    ramfs.Add(remote_command_);
    ramfs.Add(imu_command_);
    ramfs.Add(motors_command_);
    ramfs.Add(torque_command_);
  }

 private:
  static void OnRemote(bool, GimbalDiagnostics* self, const RemoteData& value)
  {
    self->remote_.Store(value);
  }

  static void OnCommand(bool, GimbalDiagnostics* self,
                        const GimbalCommand& value)
  {
    self->command_.Store(value);
  }

  static void OnImu(bool, GimbalDiagnostics* self,
                    const VqfOutput<float>& value)
  {
    self->imu_.Store(value);
  }

  static void OnYawFeedback(bool, GimbalDiagnostics* self,
                            const DJIMotor::SolvedFeedback& value)
  {
    self->yaw_feedback_.Store(value);
  }

  static void OnBigYawFeedback(bool, GimbalDiagnostics* self,
                               const DMMotor::SolvedFeedback& value)
  {
    self->big_yaw_feedback_.Store(value);
  }

  static void OnPitchFeedback(bool, GimbalDiagnostics* self,
                              const DMMotor::SolvedFeedback& value)
  {
    self->pitch_feedback_.Store(value);
  }

  static void OnYawTorque(bool, GimbalDiagnostics* self, const float& value)
  {
    self->yaw_torque_.Store(value);
  }

  static void OnBigYawTorque(bool, GimbalDiagnostics* self,
                             const DMMotor::ControlCommand& value)
  {
    self->big_yaw_torque_.Store(value);
  }

  static void OnPitchTorque(bool, GimbalDiagnostics* self,
                            const DMMotor::ControlCommand& value)
  {
    self->pitch_torque_.Store(value);
  }

  static int RunDiag(GimbalDiagnostics* self, int, char**)
  {
    RemoteData remote{};
    GimbalCommand command{};
    VqfOutput<float> imu{};
    DJIMotor::SolvedFeedback yaw{};
    DMMotor::SolvedFeedback big_yaw{};
    DMMotor::SolvedFeedback pitch{};
    float yaw_torque = 0.0F;
    uint32_t remote_age = 0U;
    uint32_t command_age = 0U;
    uint32_t imu_age = 0U;
    uint32_t yaw_age = 0U;
    uint32_t big_yaw_age = 0U;
    uint32_t pitch_age = 0U;
    uint32_t torque_age = 0U;

    const bool has_remote = self->remote_.Load(remote, remote_age);
    const bool has_command = self->command_.Load(command, command_age);
    const bool has_imu = self->imu_.Load(imu, imu_age);
    const bool has_yaw = self->yaw_feedback_.Load(yaw, yaw_age);
    const bool has_big_yaw =
        self->big_yaw_feedback_.Load(big_yaw, big_yaw_age);
    const bool has_pitch = self->pitch_feedback_.Load(pitch, pitch_age);
    const bool has_torque = self->yaw_torque_.Load(yaw_torque, torque_age);

    LibXR::STDIO::Printf<"remote %-7s age=%lu ms\r\n">(
        Freshness(has_remote, remote_age, kRemoteFreshMs),
        static_cast<unsigned long>(remote_age));
    LibXR::STDIO::Printf<"command %-7s age=%lu state=%s enabled=%d\r\n">(
        Freshness(has_command, command_age, kFastFreshMs),
        static_cast<unsigned long>(command_age), StateName(command.state),
        command.enabled ? 1 : 0);
    LibXR::STDIO::Printf<"imu     %-7s age=%lu ms\r\n">(
        Freshness(has_imu, imu_age, kFastFreshMs),
        static_cast<unsigned long>(imu_age));
    LibXR::STDIO::Printf<"yaw     %-7s age=%lu ms\r\n">(
        Freshness(has_yaw, yaw_age, kFastFreshMs),
        static_cast<unsigned long>(yaw_age));
    LibXR::STDIO::Printf<"big_yaw %-7s age=%lu ms\r\n">(
        Freshness(has_big_yaw, big_yaw_age, kFastFreshMs),
        static_cast<unsigned long>(big_yaw_age));
    LibXR::STDIO::Printf<"pitch   %-7s age=%lu ms\r\n">(
        Freshness(has_pitch, pitch_age, kFastFreshMs),
        static_cast<unsigned long>(pitch_age));
    LibXR::STDIO::Printf<"control %-7s age=%lu yaw_torque=%.3f\r\n">(
        Freshness(has_torque, torque_age, kFastFreshMs),
        static_cast<unsigned long>(torque_age), yaw_torque);

    if (!has_remote || remote_age > kRemoteFreshMs)
    {
      LibXR::STDIO::Printf<"VERDICT: REMOTE OFFLINE; check UART5/SBUS.\r\n">();
    }
    else if (!has_command || command_age > kFastFreshMs)
    {
      LibXR::STDIO::Printf<"VERDICT: remote task is not producing commands.\r\n">();
    }
    else if (!command.enabled)
    {
      LibXR::STDIO::Printf<"VERDICT: PROTECTED; select a valid right-switch mode.\r\n">();
    }
    else if (!has_imu || imu_age > kFastFreshMs)
    {
      LibXR::STDIO::Printf<"VERDICT: IMU/VQF feedback missing.\r\n">();
    }
    else if (!has_yaw || yaw_age > kFastFreshMs)
    {
      LibXR::STDIO::Printf<"VERDICT: YAW motor feedback missing on FDCAN1.\r\n">();
    }
    else if (!has_big_yaw || big_yaw_age > kFastFreshMs)
    {
      LibXR::STDIO::Printf<"VERDICT: BIG_YAW feedback missing on FDCAN3.\r\n">();
    }
    else if (!has_pitch || pitch_age > kFastFreshMs)
    {
      LibXR::STDIO::Printf<"VERDICT: PITCH feedback missing on FDCAN3.\r\n">();
    }
    else if (!has_torque || torque_age > kFastFreshMs)
    {
      LibXR::STDIO::Printf<"VERDICT: gimbal control task is not running.\r\n">();
    }
    else
    {
      LibXR::STDIO::Printf<"VERDICT: inputs healthy; inspect torque and motor state.\r\n">();
    }
    return 0;
  }

  static int RunRemote(GimbalDiagnostics* self, int, char**)
  {
    RemoteData remote{};
    GimbalCommand command{};
    uint32_t remote_age = 0U;
    uint32_t command_age = 0U;
    const bool has_remote = self->remote_.Load(remote, remote_age);
    const bool has_command = self->command_.Load(command, command_age);
    if (!has_remote)
    {
      LibXR::STDIO::Printf<"remote: no SBUS frame received\r\n">();
      return 0;
    }
    LibXR::STDIO::Printf<"remote age=%lu ch=[%d,%d,%d,%d,%d]\r\n">(
        static_cast<unsigned long>(remote_age), remote.rc.ch[0], remote.rc.ch[1],
        remote.rc.ch[2], remote.rc.ch[3], remote.rc.ch[4]);
    LibXR::STDIO::Printf<"switch left=%d right=%d key=0x%04x\r\n">(
        static_cast<int>(remote.rc.s[0]), static_cast<int>(remote.rc.s[1]),
        static_cast<unsigned int>(remote.key.v));
    if (has_command)
    {
      LibXR::STDIO::Printf<"cmd age=%lu state=%s enable=%d yaw=%.5f pitch=%.5f\r\n">(
          static_cast<unsigned long>(command_age), StateName(command.state),
          command.enabled ? 1 : 0, command.yaw_offset_rad,
          command.pitch_offset_rad);
    }
    return 0;
  }

  static int RunImu(GimbalDiagnostics* self, int, char**)
  {
    VqfOutput<float> imu{};
    uint32_t age = 0U;
    if (!self->imu_.Load(imu, age))
    {
      LibXR::STDIO::Printf<"imu: no VQF output received\r\n">();
      return 0;
    }
    const auto& q = imu.orientation_6d;
    const auto& gyro = imu.bias_corrected_angular_velocity_sensor_rad_per_second;
    const auto& accel = imu.acceleration_imu_meters_per_second_squared;
    LibXR::STDIO::Printf<"imu age=%lu q=[%.3f %.3f %.3f %.3f]\r\n">(
        static_cast<unsigned long>(age), q.w, q.x, q.y, q.z);
    LibXR::STDIO::Printf<"gyro=[%.3f %.3f %.3f] rad/s\r\n">(
        gyro.x, gyro.y, gyro.z);
    LibXR::STDIO::Printf<"accel=[%.3f %.3f %.3f] m/s2\r\n">(
        accel.x, accel.y, accel.z);
    return 0;
  }

  static int RunMotors(GimbalDiagnostics* self, int, char**)
  {
    DJIMotor::SolvedFeedback yaw{};
    DMMotor::SolvedFeedback big_yaw{};
    DMMotor::SolvedFeedback pitch{};
    uint32_t yaw_age = 0U;
    uint32_t big_yaw_age = 0U;
    uint32_t pitch_age = 0U;
    const bool has_yaw = self->yaw_feedback_.Load(yaw, yaw_age);
    const bool has_big_yaw =
        self->big_yaw_feedback_.Load(big_yaw, big_yaw_age);
    const bool has_pitch = self->pitch_feedback_.Load(pitch, pitch_age);
    if (has_yaw)
    {
      LibXR::STDIO::Printf<"yaw age=%lu pos=%.3f vel=%.3f tq=%.3f temp=%u\r\n">(
          static_cast<unsigned long>(yaw_age), yaw.position_rad,
          yaw.velocity_rad_per_sec, yaw.torque_nm,
          static_cast<unsigned int>(yaw.temperature_c));
    }
    else
    {
      LibXR::STDIO::Printf<"yaw: no feedback\r\n">();
    }
    PrintDm("big_yaw", has_big_yaw, big_yaw_age, big_yaw);
    PrintDm("pitch", has_pitch, pitch_age, pitch);
    return 0;
  }

  static void PrintDm(const char* name, bool seen, uint32_t age,
                      const DMMotor::SolvedFeedback& value)
  {
    if (!seen)
    {
      LibXR::STDIO::Printf<"%s: no feedback\r\n">(name);
      return;
    }
    LibXR::STDIO::Printf<"%s age=%lu pos=%.3f vel=%.3f tq=%.3f\r\n">(
        name, static_cast<unsigned long>(age),
        value.position_rad, value.velocity_rad_per_sec, value.torque_nm);
    LibXR::STDIO::Printf<"  enabled=%d status=%u mos=%u rotor=%u\r\n">(
        value.enabled ? 1 : 0, static_cast<unsigned int>(value.status),
        static_cast<unsigned int>(value.mos_temperature_c),
        static_cast<unsigned int>(value.rotor_temperature_c));
  }

  static int RunTorque(GimbalDiagnostics* self, int, char**)
  {
    float yaw = 0.0F;
    DMMotor::ControlCommand big_yaw{};
    DMMotor::ControlCommand pitch{};
    uint32_t yaw_age = 0U;
    uint32_t big_yaw_age = 0U;
    uint32_t pitch_age = 0U;
    const bool has_yaw = self->yaw_torque_.Load(yaw, yaw_age);
    const bool has_big_yaw = self->big_yaw_torque_.Load(big_yaw, big_yaw_age);
    const bool has_pitch = self->pitch_torque_.Load(pitch, pitch_age);
    LibXR::STDIO::Printf<"yaw      %s age=%lu torque=%.3f\r\n">(
        Freshness(has_yaw, yaw_age, kFastFreshMs),
        static_cast<unsigned long>(yaw_age), yaw);
    LibXR::STDIO::Printf<"big_yaw  %s age=%lu torque=%.3f kind=%u\r\n">(
        Freshness(has_big_yaw, big_yaw_age, kFastFreshMs),
        static_cast<unsigned long>(big_yaw_age),
        big_yaw.torque_nm,
        static_cast<unsigned int>(big_yaw.kind));
    LibXR::STDIO::Printf<"pitch    %s age=%lu torque=%.3f kind=%u\r\n">(
        Freshness(has_pitch, pitch_age, kFastFreshMs),
        static_cast<unsigned long>(pitch_age),
        pitch.torque_nm,
        static_cast<unsigned int>(pitch.kind));
    return 0;
  }

  LibXR::Topic remote_topic_;
  LibXR::Topic command_topic_;
  LibXR::Topic imu_topic_;
  LibXR::Topic yaw_feedback_topic_;
  LibXR::Topic big_yaw_feedback_topic_;
  LibXR::Topic pitch_feedback_topic_;
  LibXR::Topic yaw_torque_topic_;
  LibXR::Topic big_yaw_torque_topic_;
  LibXR::Topic pitch_torque_topic_;

  LibXR::Topic::Callback remote_callback_;
  LibXR::Topic::Callback command_callback_;
  LibXR::Topic::Callback imu_callback_;
  LibXR::Topic::Callback yaw_feedback_callback_;
  LibXR::Topic::Callback big_yaw_feedback_callback_;
  LibXR::Topic::Callback pitch_feedback_callback_;
  LibXR::Topic::Callback yaw_torque_callback_;
  LibXR::Topic::Callback big_yaw_torque_callback_;
  LibXR::Topic::Callback pitch_torque_callback_;

  LatestSample<RemoteData> remote_;
  LatestSample<GimbalCommand> command_;
  LatestSample<VqfOutput<float>> imu_;
  LatestSample<DJIMotor::SolvedFeedback> yaw_feedback_;
  LatestSample<DMMotor::SolvedFeedback> big_yaw_feedback_;
  LatestSample<DMMotor::SolvedFeedback> pitch_feedback_;
  LatestSample<float> yaw_torque_;
  LatestSample<DMMotor::ControlCommand> big_yaw_torque_;
  LatestSample<DMMotor::ControlCommand> pitch_torque_;

  LibXR::RamFS::File diag_command_;
  LibXR::RamFS::File remote_command_;
  LibXR::RamFS::File imu_command_;
  LibXR::RamFS::File motors_command_;
  LibXR::RamFS::File torque_command_;
};

}  // namespace

void RegisterGimbalDiagnosticCommands(LibXR::RamFS& ramfs)
{
  static GimbalDiagnostics diagnostics(ramfs);
}

}  // namespace UserApp::Diagnostics
