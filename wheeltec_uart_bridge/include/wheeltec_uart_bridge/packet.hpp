#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

namespace wheeltec_uart_bridge
{

constexpr std::size_t kPacketSize = 11;
using Packet = std::array<std::uint8_t, kPacketSize>;

struct Limits
{
  double linear_x;
  double linear_y;
  double angular_z;
};

Packet encode_packet(double linear_x, double linear_y, double angular_z, const Limits & limits);

}  // namespace wheeltec_uart_bridge
