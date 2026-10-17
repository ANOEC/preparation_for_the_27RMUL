#include "debug/topic_debug_app.hpp"

#include <cstdlib>
#include <cstring>

#include "RTTPort.hpp"
#include "libxr_def.hpp"
#include "stdio.hpp"
#include "timebase.hpp"

namespace UserApp::Debug
{
namespace
{

constexpr uint64_t MICROSECONDS_PER_MILLISECOND = 1000ULL;
constexpr uint64_t MICROSECONDS_PER_SECOND = 1000000ULL;

float ToSeconds(uint64_t microseconds)
{
  return static_cast<float>(microseconds) /
         static_cast<float>(MICROSECONDS_PER_SECOND);
}

}  // namespace

TopicDebugApp::TopicDebugApp(uint32_t offline_timeout_ms,
                             uint32_t print_period_ms,
                             std::size_t thread_stack,
                             LibXR::Thread::Priority priority)
    : offline_timeout_ms_(offline_timeout_ms), print_period_ms_(print_period_ms)
{
  REQUIRE(offline_timeout_ms > 0);
  REQUIRE(print_period_ms > 0);

  // DebugApp 在 XRobot 模块入口之前创建，因此先确保全局 STDIO 已绑定到 RTT。
  // RTTLogger 随后再次初始化时会复用这里的静态端口，不会创建第二套缓冲区。
  RTT::InitializeStdioToSeggerRtt();
  thread_.Create(this, ThreadEntry, "topic_debug", thread_stack, priority);
}

template <typename Data>
void TopicDebugApp::Monitor<Data>::OnMessage(
    bool in_isr, Monitor* self, LibXR::MicrosecondTimestamp timestamp,
    const Data& data)
{
  // 当前监控的话题都由任务上下文发布。若将来新增 ISR 话题，不能在 ISR 中使用互斥锁，
  // 因而安全地跳过该帧，避免破坏实时性。
  if (in_isr || self == nullptr)
  {
    return;
  }

  LibXR::Mutex::LockGuard lock(self->mutex_);
  self->latest_ = data;
  self->update_interval_us_ =
      self->received_ ? (timestamp - self->last_update_).ToMicrosecond() : 0;
  self->last_update_ = timestamp;
  self->received_ = true;
}

template <typename Data>
void TopicDebugApp::Monitor<Data>::PrintStatus(LibXR::MicrosecondTimestamp now,
                                               uint64_t timeout_us,
                                               bool print_value)
{
  Data snapshot{};
  LibXR::MicrosecondTimestamp last_update{};
  uint64_t interval_us = 0;
  bool received = false;

  {
    LibXR::Mutex::LockGuard lock(mutex_);
    snapshot = latest_;
    last_update = last_update_;
    interval_us = update_interval_us_;
    received = received_;
  }

  if (!received)
  {
    LibXR::STDIO::Print<"[{}] OFFLINE, never updated\r\n">(Name());
    return;
  }

  // 使用 MicrosecondTimestamp 的减法，STM32 硬件计时器回绕时仍能得到正确时间差。
  const uint64_t age_us = (now - last_update).ToMicrosecond();
  if (age_us > timeout_us)
  {
    LibXR::STDIO::Print<"[{}] OFFLINE, age={:.2f}s, timeout={:.2f}s\r\n">(
        Name(), ToSeconds(age_us), ToSeconds(timeout_us));
    return;
  }

  LibXR::STDIO::Print<"[{}] ONLINE, age={:.2f}s, update_dt={}us\r\n">(
      Name(), ToSeconds(age_us), static_cast<uint32_t>(interval_us));
  if (print_value)
  {
    printer_(snapshot);
  }
}

void TopicDebugApp::ThreadEntry(TopicDebugApp* self)
{
  REQUIRE(self != nullptr);
  self->Run();
}

void TopicDebugApp::Run()
{
  char line[COMMAND_LINE_SIZE]{};
  std::size_t length = 0;
  bool previous_was_cr = false;
  uint32_t last_print_ms = 0;
  LibXR::Semaphore read_semaphore;

  LibXR::STDIO::Print<"\r\nTopic debug ready. Type 'topic help'.\r\n> ">();

  while (true)
  {
    const auto now = LibXR::Timebase::GetMicroseconds();
    const uint32_t now_ms =
        static_cast<uint32_t>(LibXR::Timebase::GetMilliseconds());
    const int selected = selected_monitor_.load(std::memory_order_acquire);
    const uint32_t period_ms =
        print_period_ms_.load(std::memory_order_relaxed);

    if (selected >= 0 && static_cast<std::size_t>(selected) < MONITOR_COUNT &&
        static_cast<uint32_t>(now_ms - last_print_ms) >= period_ms)
    {
      last_print_ms = now_ms;
      const uint64_t timeout_us =
          static_cast<uint64_t>(
              offline_timeout_ms_.load(std::memory_order_relaxed)) *
          MICROSECONDS_PER_MILLISECOND;
      monitors_[selected]->PrintStatus(now, timeout_us, true);
    }

    char character = '\0';
    LibXR::ReadOperation read_operation(read_semaphore, INPUT_WAIT_MS);
    const auto result = (*LibXR::STDIO::read_)(LibXR::RawData{&character, 1},
                                               read_operation);
    if (result != LibXR::ErrorCode::OK)
    {
      continue;
    }

    if (character == '\r' || character == '\n')
    {
      // 大多数终端发送 CRLF，只把这一对字符当作一次回车。
      if (character == '\n' && previous_was_cr)
      {
        previous_was_cr = false;
        continue;
      }
      previous_was_cr = character == '\r';
      line[length] = '\0';
      LibXR::STDIO::Print<"\r\n">();
      ExecuteCommand(line);
      length = 0;
      line[0] = '\0';
      LibXR::STDIO::Print<"> ">();
      continue;
    }

    previous_was_cr = false;
    if (character == '\b' || character == 0x7F)
    {
      if (length > 0)
      {
        --length;
        LibXR::STDIO::Print<"\b \b">();
      }
      continue;
    }

    if (character >= 0x20 && character <= 0x7E &&
        length + 1 < COMMAND_LINE_SIZE)
    {
      line[length++] = character;
      LibXR::STDIO::Print<"{}">(character);
    }
  }
}

void TopicDebugApp::ExecuteCommand(char* line)
{
  if (line == nullptr)
  {
    return;
  }

  char* argv[4]{};
  int argc = 0;
  char* cursor = line;
  while (*cursor != '\0' && argc < 4)
  {
    while (*cursor == ' ' || *cursor == '\t')
    {
      ++cursor;
    }
    if (*cursor == '\0')
    {
      break;
    }

    argv[argc++] = cursor;
    while (*cursor != '\0' && *cursor != ' ' && *cursor != '\t')
    {
      ++cursor;
    }
    if (*cursor != '\0')
    {
      *cursor++ = '\0';
    }
  }

  if (argc == 0)
  {
    return;
  }
  if (std::strcmp(argv[0], "topic") != 0)
  {
    LibXR::STDIO::Print<"Unknown command. Type 'topic help'.\r\n">();
    return;
  }
  if (argc == 1 || std::strcmp(argv[1], "help") == 0)
  {
    PrintHelp();
    return;
  }
  if (std::strcmp(argv[1], "list") == 0)
  {
    PrintTopicList();
    return;
  }
  if (std::strcmp(argv[1], "stop") == 0)
  {
    selected_monitor_.store(-1, std::memory_order_release);
    LibXR::STDIO::Print<"Topic echo stopped.\r\n">();
    return;
  }
  if ((std::strcmp(argv[1], "echo") == 0 ||
       std::strcmp(argv[1], "once") == 0) &&
      argc >= 3)
  {
    const int index = FindMonitor(argv[2]);
    if (index < 0)
    {
      LibXR::STDIO::Print<"Unknown topic '{}'. Use 'topic list'.\r\n">(argv[2]);
      return;
    }

    if (std::strcmp(argv[1], "echo") == 0)
    {
      selected_monitor_.store(index, std::memory_order_release);
      LibXR::STDIO::Print<"Echoing '{}'. Use 'topic stop' to stop.\r\n">(
          monitors_[index]->Name());
    }
    else
    {
      const auto now = LibXR::Timebase::GetMicroseconds();
      const uint64_t timeout_us =
          static_cast<uint64_t>(
              offline_timeout_ms_.load(std::memory_order_relaxed)) *
          MICROSECONDS_PER_MILLISECOND;
      monitors_[index]->PrintStatus(now, timeout_us, true);
    }
    return;
  }
  if ((std::strcmp(argv[1], "timeout") == 0 ||
       std::strcmp(argv[1], "period") == 0) &&
      argc >= 3)
  {
    char* end = nullptr;
    const unsigned long value = std::strtoul(argv[2], &end, 10);
    if (value == 0 || value > 3600000UL || end == argv[2] || *end != '\0')
    {
      LibXR::STDIO::Print<"Value must be 1..3600000 milliseconds.\r\n">();
      return;
    }

    if (std::strcmp(argv[1], "timeout") == 0)
    {
      offline_timeout_ms_.store(static_cast<uint32_t>(value),
                                std::memory_order_relaxed);
      LibXR::STDIO::Print<"Offline timeout={}ms.\r\n">(
          static_cast<uint32_t>(value));
    }
    else
    {
      print_period_ms_.store(static_cast<uint32_t>(value),
                             std::memory_order_relaxed);
      LibXR::STDIO::Print<"Echo period={}ms.\r\n">(
          static_cast<uint32_t>(value));
    }
    return;
  }

  LibXR::STDIO::Print<"Invalid arguments. Type 'topic help'.\r\n">();
}

void TopicDebugApp::PrintTopicList()
{
  const auto now = LibXR::Timebase::GetMicroseconds();
  const uint64_t timeout_us =
      static_cast<uint64_t>(offline_timeout_ms_.load(std::memory_order_relaxed)) *
      MICROSECONDS_PER_MILLISECOND;

  LibXR::STDIO::Print<"Registered topics (timeout={:.2f}s):\r\n">(
      ToSeconds(timeout_us));
  for (auto* monitor : monitors_)
  {
    monitor->PrintStatus(now, timeout_us, false);
  }
}

int TopicDebugApp::FindMonitor(const char* name) const
{
  if (name == nullptr)
  {
    return -1;
  }
  for (std::size_t i = 0; i < MONITOR_COUNT; ++i)
  {
    if (std::strcmp(monitors_[i]->Name(), name) == 0)
    {
      return static_cast<int>(i);
    }
  }
  return -1;
}

void TopicDebugApp::PrintHelp()
{
  LibXR::STDIO::Print<
      "topic list                 list online states\r\n"
      "topic once <name>          print one snapshot\r\n"
      "topic echo <name>          periodically print values\r\n"
      "topic stop                 stop periodic output\r\n"
      "topic timeout <ms>         set offline timeout\r\n"
      "topic period <ms>          set echo period\r\n">();
}

void TopicDebugApp::PrintRemote(const RemoteData& data)
{
  LibXR::STDIO::Print<
      "  rc.ch=[{}, {}, {}, {}, {}], rc.s=[{}, {}]\r\n"
      "  mouse=[x={}, y={}, z={}, l={}, r={}], key=0x{:04x}\r\n">(
      data.rc.ch[0], data.rc.ch[1], data.rc.ch[2], data.rc.ch[3],
      data.rc.ch[4], static_cast<int>(data.rc.s[0]),
      static_cast<int>(data.rc.s[1]), data.mouse.x, data.mouse.y, data.mouse.z,
      data.mouse.press_l, data.mouse.press_r, data.key.v);
}

void TopicDebugApp::PrintChassisCommand(const ChassisCommand& data)
{
  LibXR::STDIO::Print<
      "  vx={:.2f}m/s, vy={:.2f}m/s, wz={:.2f}rad/s, state={}, mode={}, enabled={}\r\n">(
      data.vx_mps, data.vy_mps, data.wz_rad_per_sec,
      static_cast<int>(data.state), static_cast<int>(data.mode),
      static_cast<unsigned>(data.enabled));
}

void TopicDebugApp::PrintMotorFeedback(const MotorFeedback& data)
{
  LibXR::STDIO::Print<
      "  position={:.2f}rad, velocity={:.2f}rad/s, torque={:.2f}Nm, temp={}C, temp_valid={}\r\n">(
      data.position_rad, data.velocity_rad_per_sec, data.torque_nm,
      static_cast<unsigned>(data.temperature_c),
      static_cast<unsigned>(data.temperature_valid));
}

void TopicDebugApp::PrintImuVector(const ImuVector& data)
{
  LibXR::STDIO::Print<"  x={:.2f}, y={:.2f}, z={:.2f}\r\n">(
      data[0], data[1], data[2]);
}

void TopicDebugApp::PrintVqfOutput(const VqfOutputData& data)
{
  const auto& orientation = data.orientation_6d;
  const auto& gyro =
      data.bias_corrected_angular_velocity_sensor_rad_per_second;
  const auto& acceleration =
      data.acceleration_imu_meters_per_second_squared;
  LibXR::STDIO::Print<
      "  q=[{:.3f}, {:.3f}, {:.3f}, {:.3f}]\r\n"
      "  gyro=[{:.2f}, {:.2f}, {:.2f}]rad/s, accel=[{:.2f}, {:.2f}, {:.2f}]m/s^2\r\n">(
      orientation.w, orientation.x, orientation.y, orientation.z, gyro.x,
      gyro.y, gyro.z, acceleration.x, acceleration.y, acceleration.z);
}

void TopicDebugApp::PrintTorque(const float& data)
{
  LibXR::STDIO::Print<"  torque={:.2f}Nm\r\n">(data);
}

// 模板实例列表由 topic_monitor.toml 中实际使用的数据类型生成。
#define USER_APP_TOPIC_MONITOR_INSTANTIATIONS
#include "debug/generated/topic_monitor.hpp"
#undef USER_APP_TOPIC_MONITOR_INSTANTIATIONS

}  // namespace UserApp::Debug

