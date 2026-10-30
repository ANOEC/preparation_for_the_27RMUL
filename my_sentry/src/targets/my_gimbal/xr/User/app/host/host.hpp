#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

#include "RefereeProtocol/RefereeProtocolGenerated.hpp"
#include "RefereeProtocol/common/interaction.hpp"
#include "app/host/protocol.hpp"
#include "details/endpoint.hpp"
#include "topic.hpp"
#include "uart.hpp"

namespace UserApp::Host
{

class Host
{
 public:
  Host(LibXR::UART& vision_cdc, LibXR::UART& referee_cdc);

  void Run(std::uint32_t now_ms);

 private:
  template <std::size_t Capacity>
  struct ByteBuffer
  {
    std::array<std::uint8_t, Capacity> data{};
    std::size_t size = 0;

    void Append(const std::uint8_t* source, std::size_t count);
    void Consume(std::size_t count);
  };

  template <typename Packet, std::size_t Capacity, typename Callback>
  static void ParsePackets(ByteBuffer<Capacity>& buffer, Callback&& callback);

  void DrainUart(LibXR::UART& uart, ByteBuffer<512>& buffer);
  void DrainUart(LibXR::UART& uart, ByteBuffer<128>& buffer);
  bool Write(LibXR::UART& uart, const std::uint8_t* data, std::size_t size);
  void ReceiveVision();
  void ReceiveNavigation(std::uint32_t now_ms);
  void ReceiveStateTopics();
  void ForwardRefereeFrame();
  void SendVisionState(std::uint32_t now_ms);
  void SendSentryDecision(std::uint32_t now_ms);

  static void AppendLegacyCrc(void* packet, std::size_t size);
  static bool VerifyLegacyCrc(const void* packet, std::size_t size);

  LibXR::UART& vision_cdc_;
  LibXR::UART& referee_cdc_;
  ByteBuffer<512> vision_rx_{};
  ByteBuffer<128> nav_rx_{};

  LibXR::Topic::Domain referee_domain_{"referee"};
  LibXR::Topic vision_command_topic_;
  LibXR::Topic buff_command_topic_;
  LibXR::Topic chassis_command_topic_;
  LibXR::Topic nav_mode_topic_;
  LibXR::Topic sentry_decision_topic_;
  LibXR::Topic::ASyncSubscriber<GimbalState> gimbal_state_subscriber_;
  LibXR::Topic::ASyncSubscriber<BigBuffState> buff_state_subscriber_;
  LibXR::Topic::ASyncSubscriber<RefereeSystemDetails::RawFrame>
      referee_raw_subscriber_;
  LibXR::Topic::ASyncSubscriber<RefereeProtocol::RobotStatus>
      robot_status_subscriber_;

  VisionInfo vision_info_{};
  RefereeProtocol::RobotStatus robot_status_{};
  std::uint64_t nav_mode_ = 0;
  std::uint32_t last_nav_rx_ms_ = 0;
  std::uint32_t last_vision_tx_ms_ = 0;
  std::uint32_t last_sentry_tx_ms_ = 0;
  bool robot_status_valid_ = false;
  static constexpr std::uint32_t kVisionTxPeriodMs = 2;
  static constexpr std::uint32_t kSentryTxPeriodMs = 50;
  static constexpr std::uint32_t kNavTimeoutMs = 2000;
};

}  // namespace UserApp::Host
