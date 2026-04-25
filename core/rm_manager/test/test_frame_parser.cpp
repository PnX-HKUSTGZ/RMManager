#include "gtest/gtest.h"

#include "rm_manager/frame_parser.hpp"
#include "rm_manager/util.hpp"

#include <cstdint>
#include <vector>

namespace
{

void append_u16(std::vector<uint8_t> & data, uint16_t value)
{
    data.push_back(static_cast<uint8_t>(value & 0xFF));
    data.push_back(static_cast<uint8_t>((value >> 8) & 0xFF));
}

void set_bits_le(
    std::vector<uint8_t> & data,
    std::size_t bit_offset,
    std::size_t bit_length,
    uint32_t value)
{
    for (std::size_t i = 0; i < bit_length; ++i) {
        const std::size_t current_bit = bit_offset + i;
        const auto mask = static_cast<uint8_t>(1U << (current_bit % 8));
        auto & current_byte = data[current_bit / 8];
        if (((value >> i) & 0x01U) != 0U) {
            current_byte |= mask;
        } else {
            current_byte &= static_cast<uint8_t>(~mask);
        }
    }
}

std::vector<uint8_t> make_standard_frame(
    uint16_t cmd_id,
    const std::vector<uint8_t> & payload)
{
    RMManager::FrameHeader header{};
    header.sof = 0xA5;
    header.data_length = payload.size();
    header.seq = 0x12;
    header.crc8 = Get_CRC8_Check_Sum(reinterpret_cast<uint8_t *>(&header), sizeof(header) - 1);

    std::vector<uint8_t> frame;
    frame.insert(frame.end(), reinterpret_cast<uint8_t *>(&header),
      reinterpret_cast<uint8_t *>(&header) + sizeof(header));
    append_u16(frame, cmd_id);
    frame.insert(frame.end(), payload.begin(), payload.end());

    const auto crc16 = Get_CRC16_Check_Sum(frame.data(), frame.size());
    append_u16(frame, crc16);
    return frame;
}

std::vector<uint8_t> make_legacy_remote_control_frame()
{
    std::vector<uint8_t> frame(21, 0);
    frame[0] = 0xA9;
    frame[1] = 0x53;

    set_bits_le(frame, 16, 11, 1024 + 321);
    set_bits_le(frame, 27, 11, 1024 - 210);
    set_bits_le(frame, 38, 11, 1024 + 87);
    set_bits_le(frame, 49, 11, 1024 - 42);
    set_bits_le(frame, 60, 2, 2);
    set_bits_le(frame, 62, 1, 1);
    set_bits_le(frame, 63, 1, 0);
    set_bits_le(frame, 64, 1, 1);
    set_bits_le(frame, 65, 11, 511);
    set_bits_le(frame, 76, 1, 1);
    set_bits_le(frame, 77, 16, static_cast<uint16_t>(-321));
    set_bits_le(frame, 93, 16, static_cast<uint16_t>(456));
    set_bits_le(frame, 109, 16, static_cast<uint16_t>(-789));
    set_bits_le(frame, 125, 2, 1);
    set_bits_le(frame, 127, 2, 2);
    set_bits_le(frame, 129, 2, 3);

    uint16_t keyboard_value = 0;
    keyboard_value |= (1U << 0);
    keyboard_value |= (1U << 2);
    keyboard_value |= (1U << 4);
    keyboard_value |= (1U << 6);
    keyboard_value |= (1U << 8);
    keyboard_value |= (1U << 10);
    keyboard_value |= (1U << 12);
    keyboard_value |= (1U << 14);
    set_bits_le(frame, 136, 16, keyboard_value);

    const auto crc16 = Get_CRC16_Check_Sum(frame.data(), frame.size() - 2);
    frame[19] = static_cast<uint8_t>(crc16 & 0xFF);
    frame[20] = static_cast<uint8_t>((crc16 >> 8) & 0xFF);
    return frame;
}

TEST(FrameParserTest, RejectsLegacyA953FrameAsStandardFrame)
{
    std::vector<uint8_t> payload(21, 0);
    payload[0] = 0xA9;
    payload[1] = 0x53;

    RMManager::ParsedFrame frame;
    EXPECT_EQ(
        RMManager::parse_standard_frame(payload, 0, frame),
        RMManager::FrameParseResult::kInvalidHeader);
}

TEST(StreamParserTest, ParsesLegacyRemoteControlFrame)
{
    auto buffer = make_legacy_remote_control_frame();

    RMManager::ParsedPacket packet;
    const auto outcome = RMManager::extract_next_packet(
        buffer,
        RMManager::StreamParserMode::kImageLink,
        packet);

    ASSERT_EQ(outcome.result, RMManager::StreamParseResult::kOk);
    ASSERT_EQ(packet.type, RMManager::ParsedPacketType::kLegacyRemoteControl);
    EXPECT_TRUE(buffer.empty());
    EXPECT_EQ(packet.remote_control.chanal0, 1345);
    EXPECT_EQ(packet.remote_control.chanal1, 814);
    EXPECT_EQ(packet.remote_control.chanal2, 1111);
    EXPECT_EQ(packet.remote_control.chanal3, 982);
    EXPECT_EQ(packet.remote_control.cut, 2);
    EXPECT_EQ(packet.remote_control.stop, 1);
    EXPECT_EQ(packet.remote_control.keyl, 0);
    EXPECT_EQ(packet.remote_control.keyr, 1);
    EXPECT_EQ(packet.remote_control.wheel, 511);
    EXPECT_EQ(packet.remote_control.keyb, 1);
    EXPECT_EQ(packet.remote_control.mousex, -321);
    EXPECT_EQ(packet.remote_control.mousey, 456);
    EXPECT_EQ(packet.remote_control.mousez, -789);
    EXPECT_EQ(packet.remote_control.pressl, 1);
    EXPECT_EQ(packet.remote_control.pressr, 2);
    EXPECT_EQ(packet.remote_control.pressmid, 3);
    EXPECT_EQ(packet.remote_control.w, 1);
    EXPECT_EQ(packet.remote_control.s, 0);
    EXPECT_EQ(packet.remote_control.a, 1);
    EXPECT_EQ(packet.remote_control.d, 0);
    EXPECT_EQ(packet.remote_control.shift, 1);
    EXPECT_EQ(packet.remote_control.ctrl, 0);
    EXPECT_EQ(packet.remote_control.q, 1);
    EXPECT_EQ(packet.remote_control.e, 0);
    EXPECT_EQ(packet.remote_control.r, 1);
    EXPECT_EQ(packet.remote_control.f, 0);
    EXPECT_EQ(packet.remote_control.g, 1);
    EXPECT_EQ(packet.remote_control.z, 0);
    EXPECT_EQ(packet.remote_control.x, 1);
    EXPECT_EQ(packet.remote_control.c, 0);
    EXPECT_EQ(packet.remote_control.v, 1);
    EXPECT_EQ(packet.remote_control.b, 0);
}

TEST(StreamParserTest, BuffersPartialLegacyFrameUntilComplete)
{
    const auto frame = make_legacy_remote_control_frame();
    std::vector<uint8_t> buffer(frame.begin(), frame.begin() + 10);

    RMManager::ParsedPacket packet;
    auto outcome = RMManager::extract_next_packet(
        buffer,
        RMManager::StreamParserMode::kImageLink,
        packet);
    ASSERT_EQ(outcome.result, RMManager::StreamParseResult::kNeedMoreData);
    ASSERT_EQ(buffer.size(), 10U);

    buffer.insert(buffer.end(), frame.begin() + 10, frame.end());
    outcome = RMManager::extract_next_packet(
        buffer,
        RMManager::StreamParserMode::kImageLink,
        packet);
    ASSERT_EQ(outcome.result, RMManager::StreamParseResult::kOk);
    ASSERT_EQ(packet.type, RMManager::ParsedPacketType::kLegacyRemoteControl);
    EXPECT_TRUE(buffer.empty());
}

TEST(StreamParserTest, BuffersPartialStandardFrameUntilComplete)
{
    const auto frame = make_standard_frame(0x0311, std::vector<uint8_t>(30, 0x22));
    std::vector<uint8_t> buffer(frame.begin(), frame.begin() + 9);

    RMManager::ParsedPacket packet;
    auto outcome = RMManager::extract_next_packet(
        buffer,
        RMManager::StreamParserMode::kImageLink,
        packet);
    ASSERT_EQ(outcome.result, RMManager::StreamParseResult::kNeedMoreData);
    ASSERT_EQ(buffer.size(), 9U);

    buffer.insert(buffer.end(), frame.begin() + 9, frame.end());
    outcome = RMManager::extract_next_packet(
        buffer,
        RMManager::StreamParserMode::kImageLink,
        packet);
    ASSERT_EQ(outcome.result, RMManager::StreamParseResult::kOk);
    ASSERT_EQ(packet.type, RMManager::ParsedPacketType::kStandardFrame);
    EXPECT_EQ(packet.frame.command_id, 0x0311);
    EXPECT_EQ(packet.frame.payload, std::vector<uint8_t>(30, 0x22));
    EXPECT_TRUE(buffer.empty());
}

TEST(StreamParserTest, ParsesMixedLegacyAndStandardFramesFromSingleBuffer)
{
    auto buffer = make_legacy_remote_control_frame();
    const auto standard_frame = make_standard_frame(0x0302, std::vector<uint8_t>(30, 0x11));
    buffer.insert(buffer.end(), standard_frame.begin(), standard_frame.end());

    RMManager::ParsedPacket packet;
    auto outcome = RMManager::extract_next_packet(
        buffer,
        RMManager::StreamParserMode::kImageLink,
        packet);
    ASSERT_EQ(outcome.result, RMManager::StreamParseResult::kOk);
    ASSERT_EQ(packet.type, RMManager::ParsedPacketType::kLegacyRemoteControl);
    ASSERT_FALSE(buffer.empty());

    outcome = RMManager::extract_next_packet(
        buffer,
        RMManager::StreamParserMode::kImageLink,
        packet);
    ASSERT_EQ(outcome.result, RMManager::StreamParseResult::kOk);
    ASSERT_EQ(packet.type, RMManager::ParsedPacketType::kStandardFrame);
    EXPECT_EQ(packet.frame.command_id, 0x0302);
    EXPECT_EQ(packet.frame.payload, std::vector<uint8_t>(30, 0x11));
    EXPECT_TRUE(buffer.empty());
}

TEST(StreamParserTest, ResynchronizesAfterNoiseAndCorruptedLegacyFrame)
{
    auto corrupted_legacy = make_legacy_remote_control_frame();
    corrupted_legacy[7] ^= 0x01;

    std::vector<uint8_t> buffer = {0x00, 0x55, 0x66};
    buffer.insert(buffer.end(), corrupted_legacy.begin(), corrupted_legacy.end());

    const auto standard_frame = make_standard_frame(0x0F01, {4});
    buffer.insert(buffer.end(), standard_frame.begin(), standard_frame.end());

    RMManager::ParsedPacket packet;
    bool found_standard_frame = false;
    int skipped_count = 0;

    for (std::size_t i = 0; i < standard_frame.size() + corrupted_legacy.size(); ++i) {
        const auto outcome = RMManager::extract_next_packet(
            buffer,
            RMManager::StreamParserMode::kImageLink,
            packet);
        if (outcome.result == RMManager::StreamParseResult::kOk) {
            found_standard_frame = packet.type == RMManager::ParsedPacketType::kStandardFrame &&
              packet.frame.command_id == 0x0F01;
            break;
        }
        if (outcome.result == RMManager::StreamParseResult::kSkippedBytes) {
            ++skipped_count;
            continue;
        }
        FAIL() << "Parser requested more data before reaching the valid trailing frame.";
    }

    EXPECT_TRUE(found_standard_frame);
    EXPECT_GT(skipped_count, 0);
}

}  // namespace
