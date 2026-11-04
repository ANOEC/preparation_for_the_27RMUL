#include "app/communication/b2b/board_protocol.hpp"

#include <cassert>
#include <cmath>
#include <cstddef>
#include <span>

int main()
{
  using namespace UserApp;
  Command::ChassisCommand source{};
  source.referee_alive = true;
  source.big_yaw_alive = true;
  source.yaw_alive = true;
  source.vx_mps = 1.25F;
  source.vy_mps = -2.5F;
  source.wz_rad_per_sec = 3.75F;
  source.mode = Command::ChassisMode::SPIN;
  source.state = Command::ControlState::GAME;
  source.game_progress = 4U;
  source.stage_remain_time_s = 409U;
  source.current_hp = 321U;
  source.chassis_power_limit_w = 100U;
  source.buffer_energy_j = 60U;
  source.big_yaw_angle_rad = 0.5F;
  source.yaw_angle_rad = -0.25F;

  const auto frame = BoardProtocol::EncodeChassisCommand(source);
  assert(frame[0] == 1U && frame[1] == 1U && frame[2] == 1U);
  assert(frame[24] == 4U && frame[25] == 0U);
  assert(frame[26] == 0x99U && frame[27] == 0x01U);
  assert(frame[28] == 0x41U && frame[29] == 0x01U);
  assert(frame[30] == 100U && frame[31] == 0U);
  assert(frame[32] == 60U && frame[33] == 0U);

  Command::ChassisCommand decoded{};
  const auto bytes = std::as_bytes(std::span(frame));
  assert(BoardProtocol::DecodeChassisCommand(bytes, decoded));
  assert(decoded.referee_alive && decoded.big_yaw_alive && decoded.yaw_alive);
  assert(decoded.game_progress == source.game_progress);
  assert(decoded.stage_remain_time_s == source.stage_remain_time_s);
  assert(decoded.current_hp == source.current_hp);
  assert(decoded.chassis_power_limit_w == source.chassis_power_limit_w);
  assert(decoded.buffer_energy_j == source.buffer_energy_j);
  assert(std::abs(decoded.big_yaw_angle_rad - source.big_yaw_angle_rad) < 1e-6F);
  assert(std::abs(decoded.yaw_angle_rad - source.yaw_angle_rad) < 1e-6F);
}
