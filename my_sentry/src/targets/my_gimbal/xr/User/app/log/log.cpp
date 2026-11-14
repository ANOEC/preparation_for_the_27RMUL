#include "app/log/log.hpp"

#include <array>
#include <cstddef>
#include <cstdint>

#include "DJIMotor.hpp"
#include "DMMotor.hpp"
#include "RefereeProtocol/RefereeProtocolGenerated.hpp"
#include "RefereeSystem.hpp"
#include "libxr.hpp"
#include "timebase.hpp"
#include "topic.hpp"

namespace UserApp::Log
{
namespace
{
constexpr std::uint32_t kRefereeOnlineTimeoutMs = 1500U;
constexpr std::uint32_t kRefereeLogPeriodMs = 1000U;
constexpr std::uint32_t kFeedbackTimeoutMs = 100U; // 电机反馈超时时间
constexpr std::uint32_t kStartupGraceMs = 1000U; // 上电启动保护时间
constexpr std::uint32_t kToneMs = 100U; // 每一声蜂鸣持续时间
constexpr std::uint32_t kSilenceMs = 100U; // 同一组报警中，两声蜂鸣之间的静音间隔
constexpr std::uint32_t kPatternGapMs = 1000U; // 一组完整报警播放结束后，距离下一组报警的静音间隔
constexpr std::uint32_t kToneFrequencyHz = 2000U; // 蜂鸣频率

LibXR::PWM* buzzer_pwm = nullptr;

struct FeedbackState
{
  std::uint32_t last_receive_ms = 0U;
  bool received = false;
};

class MotorOfflineLogger
{
 public:
  explicit MotorOfflineLogger(LibXR::PWM& pwm)
      : pwm_(pwm),
        pitch_sub_(LibXR::Topic::CreateTopic<DMMotor::SolvedFeedback>(
            "/fb/motor/pitch")),
        yaw_sub_(LibXR::Topic::CreateTopic<DJIMotor::SolvedFeedback>(
            "/fb/motor/yaw")),
        big_yaw_sub_(LibXR::Topic::CreateTopic<DMMotor::SolvedFeedback>(
            "/fb/motor/big_yaw")),
        pluck_sub_(LibXR::Topic::CreateTopic<DJIMotor::SolvedFeedback>(
            "/fb/motor/pluck")),
        fri_left_sub_(LibXR::Topic::CreateTopic<DJIMotor::SolvedFeedback>(
            "/fb/motor/fri_left")),
        fri_right_sub_(LibXR::Topic::CreateTopic<DJIMotor::SolvedFeedback>(
            "/fb/motor/fri_right")),
        startup_ms_(static_cast<std::uint32_t>(LibXR::Timebase::GetMilliseconds()))
  {
    pitch_sub_.StartWaiting();
    yaw_sub_.StartWaiting();
    big_yaw_sub_.StartWaiting();
    pluck_sub_.StartWaiting();
    fri_left_sub_.StartWaiting();
    fri_right_sub_.StartWaiting();
    Silence();
  }

  void Run(std::uint32_t now_ms)
  {
    Poll(pitch_sub_, states_[0], now_ms);
    Poll(yaw_sub_, states_[1], now_ms);
    Poll(big_yaw_sub_, states_[2], now_ms);
    Poll(pluck_sub_, states_[3], now_ms);
    Poll(fri_left_sub_, states_[4], now_ms);
    Poll(fri_right_sub_, states_[5], now_ms);

    if (static_cast<std::uint32_t>(now_ms - startup_ms_) < kStartupGraceMs)
    {
      return;
    }
    UpdateBuzzer(now_ms);
  }

 private:
  template <typename Feedback>
  static void Poll(LibXR::Topic::ASyncSubscriber<Feedback>& subscriber,
                   FeedbackState& state, std::uint32_t now_ms)
  {
    if (!subscriber.Available()) return;
    static_cast<void>(subscriber.GetData());
    state.last_receive_ms = now_ms;
    state.received = true;
    subscriber.StartWaiting();
  }

  [[nodiscard]] bool Offline(std::size_t index, std::uint32_t now_ms) const
  {
    const auto& state = states_[index];
    return !state.received ||
           static_cast<std::uint32_t>(now_ms - state.last_receive_ms) >
               kFeedbackTimeoutMs;
  }

  void UpdateBuzzer(std::uint32_t now_ms)
  {
    if (static_cast<std::int32_t>(now_ms - deadline_ms_) < 0) return;

    if (tone_on_)
    {
      Silence();
      tone_on_ = false;
      if (--pulses_remaining_ == 0U)
      {
        deadline_ms_ = now_ms + kPatternGapMs;
      }
      else
      {
        deadline_ms_ = now_ms + kSilenceMs;
      }
      return;
    }

    if (pulses_remaining_ > 0U)
    {
      Tone();
      tone_on_ = true;
      deadline_ms_ = now_ms + kToneMs;
      return;
    }

    for (std::size_t offset = 1; offset <= states_.size(); ++offset)
    {
      const std::size_t index = (last_motor_ + offset) % states_.size();
      if (!Offline(index, now_ms)) continue;
      last_motor_ = index;
      pulses_remaining_ = static_cast<std::uint8_t>(index + 1U);
      Tone();
      tone_on_ = true;
      deadline_ms_ = now_ms + kToneMs;
      return;
    }
    last_motor_ = states_.size() - 1U;
    deadline_ms_ = now_ms + kFeedbackTimeoutMs;
  }

  void Tone()
  {
    static_cast<void>(pwm_.SetDutyCycle(0.0F));
    static_cast<void>(pwm_.SetConfig({kToneFrequencyHz}));
    static_cast<void>(pwm_.SetDutyCycle(0.5F));
    static_cast<void>(pwm_.Enable());
  }

  void Silence()
  {
    static_cast<void>(pwm_.SetDutyCycle(0.0F));
    static_cast<void>(pwm_.Disable());
  }

  LibXR::PWM& pwm_;
  LibXR::Topic::ASyncSubscriber<DMMotor::SolvedFeedback> pitch_sub_;
  LibXR::Topic::ASyncSubscriber<DJIMotor::SolvedFeedback> yaw_sub_;
  LibXR::Topic::ASyncSubscriber<DMMotor::SolvedFeedback> big_yaw_sub_;
  LibXR::Topic::ASyncSubscriber<DJIMotor::SolvedFeedback> pluck_sub_;
  LibXR::Topic::ASyncSubscriber<DJIMotor::SolvedFeedback> fri_left_sub_;
  LibXR::Topic::ASyncSubscriber<DJIMotor::SolvedFeedback> fri_right_sub_;
  std::array<FeedbackState, 6> states_{};
  std::uint32_t startup_ms_;
  std::uint32_t deadline_ms_ = 0U;
  std::size_t last_motor_ = 5U;
  std::uint8_t pulses_remaining_ = 0U;
  bool tone_on_ = false;
};

class RefereeLogger
{
 public:
  using Statistics = RefereeSystemRegular::Statistics;

  RefereeLogger()
      : statistics_subscriber_(LibXR::Topic::CreateTopic<Statistics>(
            "regular/diagnostics", &referee_domain_)),
        robot_status_subscriber_(
            LibXR::Topic::CreateTopic<RefereeProtocol::RobotStatus>(
                "regular/cmd/0201/rx", &referee_domain_))
  {
    statistics_subscriber_.StartWaiting();
    robot_status_subscriber_.StartWaiting();
  }

  void Run(std::uint32_t now_ms)
  {
    if (statistics_subscriber_.Available())
    {
      statistics_ = statistics_subscriber_.GetData();
      statistics_subscriber_.StartWaiting();
      has_statistics_ = true;
    }
    if (robot_status_subscriber_.Available())
    {
      robot_status_ = robot_status_subscriber_.GetData();
      robot_status_subscriber_.StartWaiting();
      robot_status_ms_ = now_ms;
      has_robot_status_ = true;
    }
    if (static_cast<std::uint32_t>(now_ms - last_log_ms_) <
        kRefereeLogPeriodMs)
      return;
    last_log_ms_ = now_ms;

    std::uint32_t command_0201_count = 0U;
    if (has_statistics_)
    {
      for (std::size_t index = 0; index < statistics_.command_ids.size(); ++index)
      {
        if (statistics_.command_ids[index] == 0x0201U)
        {
          command_0201_count = statistics_.command_counts[index];
          break;
        }
      }
    }
    const bool online = has_robot_status_ &&
        static_cast<std::uint32_t>(now_ms - robot_status_ms_) <=
            kRefereeOnlineTimeoutMs;
    LibXR::STDIO::Printf<
        "[REF] rx=%lu valid=%lu crc8=%lu crc16=%lu len=%lu "
        "cmd0201=%lu online=%u id=%u hp=%u power=%uW\r\n">(
        static_cast<unsigned long>(statistics_.received_bytes),
        static_cast<unsigned long>(statistics_.received_frames),
        static_cast<unsigned long>(statistics_.invalid_header_crc),
        static_cast<unsigned long>(statistics_.invalid_frame_crc),
        static_cast<unsigned long>(statistics_.invalid_lengths),
        static_cast<unsigned long>(command_0201_count), online ? 1U : 0U,
        static_cast<unsigned int>(online ? robot_status_.robot_id : 0U),
        static_cast<unsigned int>(online ? robot_status_.current_hp : 0U),
        static_cast<unsigned int>(
            online ? robot_status_.chassis_power_limit : 0U));
  }

 private:
  LibXR::Topic::Domain referee_domain_{"referee"};
  LibXR::Topic::ASyncSubscriber<Statistics> statistics_subscriber_;
  LibXR::Topic::ASyncSubscriber<RefereeProtocol::RobotStatus>
      robot_status_subscriber_;
  Statistics statistics_{};
  RefereeProtocol::RobotStatus robot_status_{};
  std::uint32_t robot_status_ms_ = 0U;
  std::uint32_t last_log_ms_ = 0U;
  bool has_statistics_ = false;
  bool has_robot_status_ = false;
};
}  // namespace

void Initialize(LibXR::PWM& pwm)
{
  buzzer_pwm = &pwm;
}

void Run(std::uint32_t now_ms)
{
  REQUIRE(buzzer_pwm != nullptr);
  static MotorOfflineLogger logger(*buzzer_pwm);
  static RefereeLogger referee_logger;
  logger.Run(now_ms);
  referee_logger.Run(now_ms);
}

}  // namespace UserApp::Log
