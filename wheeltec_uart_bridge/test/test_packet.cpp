#include "wheeltec_uart_bridge/packet.hpp"

#include <gtest/gtest.h>

#include <array>
#include <limits>
#include <stdexcept>

using wheeltec_uart_bridge::Limits;
using wheeltec_uart_bridge::Packet;
using wheeltec_uart_bridge::encode_packet;

TEST(Packet, EncodesPositiveVelocity)
{
  const Packet expected{0x7b, 0x00, 0x00, 0x00, 0xc8, 0x00, 0x00, 0x01, 0x2c, 0x9e, 0x7d};
  EXPECT_EQ(encode_packet(0.2, 0.0, 0.3, Limits{0.3, 0.0, 0.6}), expected);
}

TEST(Packet, EncodesNegativeVelocity)
{
  const Packet expected{0x7b, 0x00, 0x00, 0xff, 0x6a, 0x00, 0x00, 0xfe, 0x3e, 0x2e, 0x7d};
  EXPECT_EQ(encode_packet(-0.15, 0.0, -0.45, Limits{0.3, 0.0, 0.6}), expected);
}

TEST(Packet, EncodesZeroAndChecksum)
{
  const Packet expected{0x7b, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7b, 0x7d};
  EXPECT_EQ(encode_packet(0.0, 0.0, 0.0, Limits{0.3, 0.0, 0.6}), expected);
}

TEST(Packet, AppliesPhysicalLimitsBeforeScaling)
{
  const Packet packet = encode_packet(2.0, -2.0, 3.0, Limits{0.3, 0.1, 0.6});
  EXPECT_EQ(packet[3], 0x01);
  EXPECT_EQ(packet[4], 0x2c);  // +300
  EXPECT_EQ(packet[5], 0xff);
  EXPECT_EQ(packet[6], 0x9c);  // -100
  EXPECT_EQ(packet[7], 0x02);
  EXPECT_EQ(packet[8], 0x58);  // +600
}

TEST(Packet, RejectsNonFiniteValues)
{
  EXPECT_THROW(
    encode_packet(std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0, Limits{0.3, 0.0, 0.6}),
    std::invalid_argument);
}
