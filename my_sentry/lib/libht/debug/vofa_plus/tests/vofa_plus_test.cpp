#include "vofa_plus.hpp"

#include <cassert>
#include <cstdint>
#include <cstring>
#include <vector>

namespace
{

struct FakeTransport
{
  std::vector<std::uint8_t> tx;
  std::vector<std::uint8_t> rx;
  std::size_t rx_offset = 0;
  std::size_t write_limit = debug::vofa_plus::VofaPlus::kFrameCapacity;
};

bool Write(void* context, const std::uint8_t* data, std::size_t size)
{
  auto& fake = *static_cast<FakeTransport*>(context);
  if (size > fake.write_limit)
  {
    return false;
  }
  fake.tx.assign(data, data + size);
  return true;
}

std::size_t Available(void* context)
{
  const auto& fake = *static_cast<FakeTransport*>(context);
  return fake.rx.size() - fake.rx_offset;
}

bool Read(void* context, std::uint8_t* data, std::size_t size)
{
  auto& fake = *static_cast<FakeTransport*>(context);
  if (Available(context) < size)
  {
    return false;
  }
  std::memcpy(data, fake.rx.data() + fake.rx_offset, size);
  fake.rx_offset += size;
  return true;
}

debug::vofa_plus::VofaPlus MakeVofa(FakeTransport& fake)
{
  return debug::vofa_plus::VofaPlus({&fake, Write, Available, Read});
}

}  // namespace

int main()
{
  using Result = debug::vofa_plus::VofaPlus::Result;
  const float values[] = {1.25F, -2.5F, 42.0F};
  const std::vector<std::uint8_t> expected_frame = {
      'd', 'a', 't', 'a', ':', '1', '.', '2', '5', ',', '-', '2', '.', '5', ',',
      '4', '2', '\n'};

  FakeTransport send_transport;
  auto sender = MakeVofa(send_transport);
  assert(sender.Send(values, 3) == Result::OK);
  assert(send_transport.tx == expected_frame);

  assert(sender.Send(values, 3, "") == Result::OK);
  const std::vector<std::uint8_t> no_prefix = {
      '1', '.', '2', '5', ',', '-', '2', '.', '5', ',', '4', '2', '\n'};
  assert(send_transport.tx == no_prefix);

  send_transport.write_limit = expected_frame.size() - 1;
  assert(sender.Send(values, 3) == Result::IO_ERROR);
  assert(sender.Send(nullptr, 1) == Result::INVALID_ARGUMENT);
  assert(sender.Send(values, 3, "image") == Result::INVALID_ARGUMENT);
  assert(sender.Send(values, sender.kMaxFloatCount + 1) == Result::FRAME_TOO_LARGE);

  FakeTransport partial_transport;
  auto partial_receiver = MakeVofa(partial_transport);
  partial_transport.rx = {'s', 'e', 't', ':', '1'};
  float output[3]{};
  assert(partial_receiver.Receive(output, 3) == Result::NO_FRAME);
  const std::vector<std::uint8_t> remaining = {
      '.', '2', '5', ',', ' ', '-', '2', '.', '5', ',', '4', '2', '\r', '\n'};
  partial_transport.rx.insert(partial_transport.rx.end(), remaining.begin(),
                              remaining.end());
  assert(partial_receiver.Receive(output, 3) == Result::OK);
  assert(std::memcmp(output, values, sizeof(values)) == 0);

  FakeTransport malformed_transport;
  auto malformed_receiver = MakeVofa(malformed_transport);
  malformed_transport.rx = {'d', 'a', 't', 'a', ':', '1', ',', 'x', ',', '3', '\n'};
  assert(malformed_receiver.Receive(output, 3) == Result::PARSE_ERROR);

  return 0;
}
