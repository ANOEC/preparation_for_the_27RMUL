#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>

#include "pwm.hpp"
#include "thread.hpp"

namespace UserApp::Buzzer
{

/**
 * 异步播放 GALA《你》的蜂鸣器应用。
 *
 * 构造后工作线程保持静默；调用 Play() 才会开始播放。播放过程中不会阻塞
 * 调用者或底盘控制线程，Stop() 最迟在 10 ms 内关闭 PWM。
 */
class YouBuzzer
{
 public:
  explicit YouBuzzer(
      LibXR::PWM& pwm, std::size_t thread_stack = 1024,
      LibXR::Thread::Priority priority = LibXR::Thread::Priority::LOW);

  /** 从头播放；播放中再次调用时也会从头开始。 */
  void Play();

  /** 请求停止播放并关闭 PWM 输出。 */
  void Stop();

  [[nodiscard]] bool IsPlaying() const;
  [[nodiscard]] LibXR::ErrorCode LastError() const;

 private:
  struct NoteEvent
  {
    uint16_t frequency_hz;
    uint16_t duration_ms;
  };

  static constexpr uint32_t CONTROL_GRANULARITY_MS = 10;
  static const NoteEvent SCORE[];
  static const std::size_t SCORE_SIZE;

  static void ThreadEntry(YouBuzzer* self);
  void Run();
  bool Output(const NoteEvent& event);
  void Silence();
  bool StoreResult(LibXR::ErrorCode result);

  LibXR::PWM& pwm_;
  LibXR::Thread thread_;
  std::atomic<bool> playing_{false};
  std::atomic<bool> restart_requested_{false};
  std::atomic<LibXR::ErrorCode> last_error_{LibXR::ErrorCode::OK};
};

}  // namespace UserApp::Buzzer
