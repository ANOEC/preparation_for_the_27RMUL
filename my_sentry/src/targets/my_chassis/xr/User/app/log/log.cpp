#include "app/log/log.hpp"

#include <array>
#include <cstddef>
#include <cstdint>

#include "DJIMotor.hpp"
#include "timebase.hpp"
#include "topic.hpp"

namespace UserApp::Log
{
namespace
{
constexpr std::uint32_t kFeedbackTimeoutMs = 100U; // 电机反馈超时时间
constexpr std::uint32_t kStartupGraceMs = 1000U; // 上电启动保护时间
constexpr std::uint32_t kToneMs = 500U; // 每一声蜂鸣持续时间
constexpr std::uint32_t kSilenceMs = 200U; // 同一组报警中，两声蜂鸣之间的静音间隔
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
        motor_1_sub_(MakeSubscriber("/fb/motor/chassis_motor1")),
        motor_2_sub_(MakeSubscriber("/fb/motor/chassis_motor2")),
        motor_3_sub_(MakeSubscriber("/fb/motor/chassis_motor3")),
        motor_4_sub_(MakeSubscriber("/fb/motor/chassis_motor4")),
        startup_ms_(static_cast<std::uint32_t>(LibXR::Timebase::GetMilliseconds()))
  {
    motor_1_sub_.StartWaiting();
    motor_2_sub_.StartWaiting();
    motor_3_sub_.StartWaiting();
    motor_4_sub_.StartWaiting();
    Silence();
  }

  void Run(std::uint32_t now_ms)
  {
    Poll(motor_1_sub_, states_[0], now_ms);
    Poll(motor_2_sub_, states_[1], now_ms);
    Poll(motor_3_sub_, states_[2], now_ms);
    Poll(motor_4_sub_, states_[3], now_ms);
    if (static_cast<std::uint32_t>(now_ms - startup_ms_) < kStartupGraceMs)
      return;
    UpdateBuzzer(now_ms);
  }

 private:
  using Subscriber = LibXR::Topic::ASyncSubscriber<DJIMotor::SolvedFeedback>;

  static Subscriber MakeSubscriber(const char* name)
  {
    return Subscriber(LibXR::Topic::CreateTopic<DJIMotor::SolvedFeedback>(name));
  }

  static void Poll(Subscriber& subscriber, FeedbackState& state,
                   std::uint32_t now_ms)
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
      deadline_ms_ = now_ms +
          ((--pulses_remaining_ == 0U) ? kPatternGapMs : kSilenceMs);
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
  Subscriber motor_1_sub_;
  Subscriber motor_2_sub_;
  Subscriber motor_3_sub_;
  Subscriber motor_4_sub_;
  std::array<FeedbackState, 4> states_{};
  std::uint32_t startup_ms_;
  std::uint32_t deadline_ms_ = 0U;
  std::size_t last_motor_ = 3U;
  std::uint8_t pulses_remaining_ = 0U;
  bool tone_on_ = false;
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
  logger.Run(now_ms);
}

}  // namespace UserApp::Log
