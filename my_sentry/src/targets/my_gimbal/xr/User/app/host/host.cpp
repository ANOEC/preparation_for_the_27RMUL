#include "app/host/host.hpp"

#include <algorithm>
#include <cstring>

#include "utils/crc.hpp"

namespace UserApp::Host
{

template <std::size_t Capacity>
void Host::ByteBuffer<Capacity>::Append(const std::uint8_t* source,
                                        std::size_t count)
{
  if (source == nullptr || count == 0)
  {
    return;
  }

  if (count >= Capacity)
  {
    source += count - Capacity;
    count = Capacity;
    size = 0;
  }
  else if (size + count > Capacity)
  {
    const std::size_t drop = size + count - Capacity;
    Consume(drop);
  }

  std::memcpy(data.data() + size, source, count);
  size += count;
}

template <std::size_t Capacity>
void Host::ByteBuffer<Capacity>::Consume(std::size_t count)
{
  count = std::min(count, size);
  if (count == 0)
  {
    return;
  }
  std::memmove(data.data(), data.data() + count, size - count);
  size -= count;
}

template <typename Packet, std::size_t Capacity, typename Callback>
void Host::ParsePackets(ByteBuffer<Capacity>& buffer, Callback&& callback)
{
  while (buffer.size >= sizeof(Packet))
  {
    if (buffer.data[0] != kRecvHeader)
    {
      buffer.Consume(1);
      continue;
    }

    Packet packet{};
    std::memcpy(&packet, buffer.data.data(), sizeof(Packet));
    if (!VerifyLegacyCrc(&packet, sizeof(Packet)))
    {
      buffer.Consume(1);
      continue;
    }

    callback(packet);
    buffer.Consume(sizeof(Packet));
  }
}

Host::Host(LibXR::UART& vision_cdc, LibXR::UART& referee_cdc)
    : vision_cdc_(vision_cdc),
      referee_cdc_(referee_cdc),
      vision_command_topic_(
          LibXR::Topic::CreateTopic<GimbalCommand>("/vision_cmd")),
      buff_command_topic_(
          LibXR::Topic::CreateTopic<BigBuffCommand>("/buff_cmd")),
      chassis_command_topic_(
          LibXR::Topic::CreateTopic<ChassisCommand>("/vision_chassis_cmd")),
      nav_mode_topic_(LibXR::Topic::CreateTopic<std::uint64_t>(
          "/vision_nav_mode")),
      sentry_decision_topic_(LibXR::Topic::CreateTopic<
          RefereeProtocol::Common::InteractionTxRequest>(
          "regular/cmd/0301/tx", &referee_domain_, true)),
      gimbal_state_subscriber_(
          LibXR::Topic::CreateTopic<GimbalState>("/vision_state")),
      buff_state_subscriber_(
          LibXR::Topic::CreateTopic<BigBuffState>("/big_buff_state")),
      referee_raw_subscriber_(LibXR::Topic::CreateTopic<
                              RefereeSystemDetails::RawFrame>(
          "regular/raw_frame/rx", &referee_domain_)),
      robot_status_subscriber_(LibXR::Topic::CreateTopic<
                               RefereeProtocol::RobotStatus>(
          "regular/cmd/0201/rx", &referee_domain_))
{
  gimbal_state_subscriber_.StartWaiting();
  buff_state_subscriber_.StartWaiting();
  referee_raw_subscriber_.StartWaiting();
  robot_status_subscriber_.StartWaiting();
}

void Host::Run(std::uint32_t now_ms)
{
  ReceiveVision();
  ReceiveNavigation(now_ms);
  ReceiveStateTopics();
  ForwardRefereeFrame();
  SendVisionState(now_ms);
  SendSentryDecision(now_ms);
}

void Host::DrainUart(LibXR::UART& uart, ByteBuffer<512>& buffer)
{
  if (uart.read_port_ == nullptr || !uart.read_port_->Readable())
  {
    return;
  }
  std::array<std::uint8_t, 128> temp{};
  const std::size_t count = std::min(uart.read_port_->Size(), temp.size());
  if (count == 0)
  {
    return;
  }
  LibXR::ReadOperation operation;
  if (uart.Read({temp.data(), count}, operation) == LibXR::ErrorCode::OK)
  {
    buffer.Append(temp.data(), count);
  }
}

void Host::DrainUart(LibXR::UART& uart, ByteBuffer<128>& buffer)
{
  if (uart.read_port_ == nullptr || !uart.read_port_->Readable())
  {
    return;
  }
  std::array<std::uint8_t, 64> temp{};
  const std::size_t count = std::min(uart.read_port_->Size(), temp.size());
  if (count == 0)
  {
    return;
  }
  LibXR::ReadOperation operation;
  if (uart.Read({temp.data(), count}, operation) == LibXR::ErrorCode::OK)
  {
    buffer.Append(temp.data(), count);
  }
}

bool Host::Write(LibXR::UART& uart, const std::uint8_t* data, std::size_t size)
{
  if (data == nullptr || size == 0 || uart.write_port_ == nullptr ||
      !uart.write_port_->Writable() || size > uart.write_port_->Capacity())
  {
    return false;
  }
  LibXR::WriteOperation operation;
  return uart.Write({data, size}, operation) == LibXR::ErrorCode::OK;
}

void Host::ReceiveVision()
{
  DrainUart(vision_cdc_, vision_rx_);
  ParsePackets<VisionCommandPacket>(
      vision_rx_, [this](const VisionCommandPacket& packet)
      {
        auto buff = packet.buff;
        auto gimbal = packet.gimbal;
        buff_command_topic_.Publish(buff);
        vision_command_topic_.Publish(gimbal);
      });
}

void Host::ReceiveNavigation(std::uint32_t now_ms)
{
  DrainUart(referee_cdc_, nav_rx_);
  ParsePackets<NavCommandPacket>(
      nav_rx_, [this, now_ms](const NavCommandPacket& packet)
      {
        auto chassis = packet.chassis;
        auto nav_mode = packet.nav_mode;
        chassis_command_topic_.Publish(chassis);
        nav_mode_topic_.Publish(nav_mode);
        nav_mode_ = nav_mode;
        last_nav_rx_ms_ = now_ms;
      });
}

void Host::ReceiveStateTopics()
{
  if (gimbal_state_subscriber_.Available())
  {
    vision_info_.gimbal_state = gimbal_state_subscriber_.GetData();
    gimbal_state_subscriber_.StartWaiting();
  }
  if (buff_state_subscriber_.Available())
  {
    vision_info_.buff_state = buff_state_subscriber_.GetData();
    buff_state_subscriber_.StartWaiting();
  }
  if (robot_status_subscriber_.Available())
  {
    robot_status_ = robot_status_subscriber_.GetData();
    robot_status_valid_ = true;
    robot_status_subscriber_.StartWaiting();
  }
}

void Host::ForwardRefereeFrame()
{
  if (!referee_raw_subscriber_.Available())
  {
    return;
  }
  const auto frame = referee_raw_subscriber_.GetData();
  referee_raw_subscriber_.StartWaiting();
  if (frame.size > 0 && frame.size <= frame.data.size())
  {
    (void)Write(referee_cdc_, frame.data.data(), frame.size);
  }
}

void Host::SendVisionState(std::uint32_t now_ms)
{
  if (static_cast<std::uint32_t>(now_ms - last_vision_tx_ms_) <
      kVisionTxPeriodMs)
  {
    return;
  }
  last_vision_tx_ms_ = now_ms;
  vision_info_.header = kSendHeader;
  AppendLegacyCrc(&vision_info_, sizeof(vision_info_));
  (void)Write(vision_cdc_, reinterpret_cast<const std::uint8_t*>(&vision_info_),
              sizeof(vision_info_));
}


void Host::SendSentryDecision(std::uint32_t now_ms)
{
  if (static_cast<std::uint32_t>(now_ms - last_sentry_tx_ms_) <
      kSentryTxPeriodMs)
  {
    return;
  }
  last_sentry_tx_ms_ = now_ms;

  // Keep the senior project's 0x0120 bit layout:
  // bit0 relife, bits21..22 posture, bit23 active energy mechanism.
  const bool nav_alive =
      static_cast<std::uint32_t>(now_ms - last_nav_rx_ms_) <= kNavTimeoutMs;
  std::uint32_t command = 0;
  if (robot_status_valid_ && robot_status_.current_hp == 0)
  {
    command |= 1U;
  }

  std::uint32_t posture = 1U;
  if (nav_alive)
  {
    if (((nav_mode_ >> 1U) & 0x1U) != 0U)
    {
      command |= (1U << 23U);
    }
    posture = static_cast<std::uint32_t>((nav_mode_ >> 2U) & 0x3U);
    if (posture == 0U)
    {
      posture = 1U;
    }
  }
  command |= (posture & 0x3U) << 21U;

  RefereeProtocol::InteractionSentryCommand payload{};
  payload.sentry_cmd = command;
  RefereeProtocol::Common::InteractionTxRequest request{};
  if (RefereeProtocol::Common::MakeInteractionTxRequest(
          payload, RefereeSystemDetails::RefereeServerId, request))
  {
    sentry_decision_topic_.Publish(request);
  }
}

void Host::AppendLegacyCrc(void* packet, std::size_t size)
{
  if (packet == nullptr || size < sizeof(std::uint16_t))
  {
    return;
  }
  auto* bytes = static_cast<std::uint8_t*>(packet);
  const std::uint16_t crc = LibXR::CRC16::Calculate(bytes, size - 2);
  bytes[size - 2] = static_cast<std::uint8_t>(crc & 0xFFU);
  bytes[size - 1] = static_cast<std::uint8_t>((crc >> 8U) & 0xFFU);
}

bool Host::VerifyLegacyCrc(const void* packet, std::size_t size)
{
  return packet != nullptr && size >= sizeof(std::uint16_t) &&
         LibXR::CRC16::Verify(packet, size);
}

}  // namespace UserApp::Host
