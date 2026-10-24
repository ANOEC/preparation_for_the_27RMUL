#include "app/debug/vofa_plus_libxr.hpp"

#include <optional>

namespace UserApp::Debug
{
namespace
{

std::optional<debug::vofa_plus::VofaPlus> vofa_plus;

bool Write(void* context, const std::uint8_t* data, std::size_t size)
{
  auto& uart = *static_cast<LibXR::UART*>(context);
  if (uart.write_port_ == nullptr || !uart.write_port_->Writable() ||
      size > uart.write_port_->Capacity())
  {
    return false;
  }

  LibXR::WriteOperation operation;
  return uart.Write({data, size}, operation) == LibXR::ErrorCode::OK;
}

std::size_t Available(void* context)
{
  auto& uart = *static_cast<LibXR::UART*>(context);
  return uart.read_port_ == nullptr ? 0 : uart.read_port_->Size();
}

bool Read(void* context, std::uint8_t* data, std::size_t size)
{
  auto& uart = *static_cast<LibXR::UART*>(context);
  if (uart.read_port_ == nullptr || !uart.read_port_->Readable() ||
      uart.read_port_->Size() < size)
  {
    return false;
  }

  LibXR::ReadOperation operation;
  return uart.Read({data, size}, operation) == LibXR::ErrorCode::OK;
}

}  // namespace

debug::vofa_plus::VofaPlus::Transport MakeVofaPlusTransport(LibXR::UART& uart) noexcept
{
  return {
      .context = &uart,
      .write = Write,
      .available = Available,
      .read = Read,
  };
}

debug::vofa_plus::VofaPlus& InitializeVofaPlus(LibXR::UART& uart) noexcept
{
  if (!vofa_plus.has_value())
  {
    vofa_plus.emplace(MakeVofaPlusTransport(uart));
  }
  return *vofa_plus;
}

debug::vofa_plus::VofaPlus* GetVofaPlus() noexcept
{
  return vofa_plus ? &*vofa_plus : nullptr;
}

}  // namespace UserApp::Debug
