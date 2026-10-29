#include "app/communication/b2b/task.hpp"

#include "app/communication/b2b/chassis_command_link.hpp"

namespace
{
UserApp::Communication::ChassisCommandLink* board_link = nullptr;
}

void UserApp::BoardLink::Initialize(LibXR::FDCAN& can)
{
  static Communication::ChassisCommandLink link(
      can, Communication::ChassisCommandLink::Role::CHASSIS);
  board_link = &link;
}

void UserApp::BoardLink::Run(std::uint32_t now_ms)
{
  REQUIRE(board_link != nullptr);
  board_link->Run(now_ms);
}
