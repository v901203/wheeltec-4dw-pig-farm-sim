#include "wheeltec_uart_bridge/packet.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace wheeltec_uart_bridge
{
namespace
{

std::int16_t scaled_i16(const double value, const double limit)
{
  if (!std::isfinite(value) || !std::isfinite(limit) || limit < 0.0) {
    throw std::invalid_argument("velocity and limit must be finite; limit must be nonnegative");
  }
  const double bounded = std::clamp(value, -limit, limit);
  const long scaled = std::lround(bounded * 1000.0);
  return static_cast<std::int16_t>(std::clamp(
    scaled, static_cast<long>(std::numeric_limits<std::int16_t>::min()),
    static_cast<long>(std::numeric_limits<std::int16_t>::max())));
}

void put_big_endian(Packet & packet, const std::size_t offset, const std::int16_t value)
{
  const auto raw = static_cast<std::uint16_t>(value);
  packet[offset] = static_cast<std::uint8_t>((raw >> 8U) & 0xffU);
  packet[offset + 1] = static_cast<std::uint8_t>(raw & 0xffU);
}

}  // namespace

Packet encode_packet(
  const double linear_x, const double linear_y, const double angular_z,
  const Limits & limits)
{
  Packet packet{};
  packet[0] = 0x7b;
  packet[1] = 0x00;
  packet[2] = 0x00;
  put_big_endian(packet, 3, scaled_i16(linear_x, limits.linear_x));
  put_big_endian(packet, 5, scaled_i16(linear_y, limits.linear_y));
  put_big_endian(packet, 7, scaled_i16(angular_z, limits.angular_z));
  for (std::size_t index = 0; index < 9; ++index) {
    packet[9] ^= packet[index];
  }
  packet[10] = 0x7d;
  return packet;
}

}  // namespace wheeltec_uart_bridge
