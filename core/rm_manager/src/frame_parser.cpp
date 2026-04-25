#include "rm_manager/frame_parser.hpp"
#include "rm_manager/util.hpp"

#include <cstring>
#include <utility>

namespace RMManager
{

namespace
{

constexpr uint8_t kGeneralMessageHeader = 0xA5;
constexpr uint8_t kLegacyRemoteControlHeader0 = 0xA9;
constexpr uint8_t kLegacyRemoteControlHeader1 = 0x53;
constexpr std::size_t kFrameHeaderLength = sizeof(FrameHeader);
constexpr std::size_t kCommandIdLength = 2;
constexpr std::size_t kFrameTailLength = 2;
constexpr std::size_t kMinimumFrameLength =
  kFrameHeaderLength + kCommandIdLength + kFrameTailLength;
constexpr std::size_t kLegacyRemoteControlLength = 21;
constexpr uint16_t kMaxStandardPayloadLength = 512;

enum class LegacyRemoteControlParseResult
{
    kOk,
    kNeedMoreData,
    kInvalidHeader,
    kInvalidCrc16,
};

void erase_prefix(std::vector<uint8_t> & data, std::size_t count)
{
    if (count == 0) {
        return;
    }
    data.erase(data.begin(), data.begin() + count);
}

uint32_t read_bits_le(
    const std::vector<uint8_t> & data,
    std::size_t bit_offset,
    std::size_t bit_length)
{
    uint32_t value = 0;
    for (std::size_t i = 0; i < bit_length; ++i) {
        const std::size_t current_bit = bit_offset + i;
        const auto current_byte = static_cast<uint32_t>(data[current_bit / 8]);
        const auto bit_value = (current_byte >> (current_bit % 8)) & 0x01U;
        value |= (bit_value << i);
    }
    return value;
}

int16_t read_signed_16_bits_le(const std::vector<uint8_t> & data, std::size_t bit_offset)
{
    return static_cast<int16_t>(read_bits_le(data, bit_offset, 16));
}

rm_message::msg::RemoteControl decode_legacy_remote_control(
    const std::vector<uint8_t> & data)
{
    rm_message::msg::RemoteControl msg;

    msg.chanal0 = static_cast<uint16_t>(read_bits_le(data, 16, 11));
    msg.chanal1 = static_cast<uint16_t>(read_bits_le(data, 27, 11));
    msg.chanal2 = static_cast<uint16_t>(read_bits_le(data, 38, 11));
    msg.chanal3 = static_cast<uint16_t>(read_bits_le(data, 49, 11));
    msg.cut = static_cast<uint8_t>(read_bits_le(data, 60, 2));
    msg.stop = static_cast<uint8_t>(read_bits_le(data, 62, 1));
    msg.keyl = static_cast<uint8_t>(read_bits_le(data, 63, 1));
    msg.keyr = static_cast<uint8_t>(read_bits_le(data, 64, 1));
    msg.wheel = static_cast<uint16_t>(read_bits_le(data, 65, 11));
    msg.keyb = static_cast<uint8_t>(read_bits_le(data, 76, 1));
    msg.mousex = read_signed_16_bits_le(data, 77);
    msg.mousey = read_signed_16_bits_le(data, 93);
    msg.mousez = read_signed_16_bits_le(data, 109);
    msg.pressl = static_cast<uint8_t>(read_bits_le(data, 125, 2));
    msg.pressr = static_cast<uint8_t>(read_bits_le(data, 127, 2));
    msg.pressmid = static_cast<uint8_t>(read_bits_le(data, 129, 2));

    const auto keyboard_value = static_cast<uint16_t>(read_bits_le(data, 136, 16));
    msg.w = (keyboard_value >> 0) & 0x01;
    msg.s = (keyboard_value >> 1) & 0x01;
    msg.a = (keyboard_value >> 2) & 0x01;
    msg.d = (keyboard_value >> 3) & 0x01;
    msg.shift = (keyboard_value >> 4) & 0x01;
    msg.ctrl = (keyboard_value >> 5) & 0x01;
    msg.q = (keyboard_value >> 6) & 0x01;
    msg.e = (keyboard_value >> 7) & 0x01;
    msg.r = (keyboard_value >> 8) & 0x01;
    msg.f = (keyboard_value >> 9) & 0x01;
    msg.g = (keyboard_value >> 10) & 0x01;
    msg.z = (keyboard_value >> 11) & 0x01;
    msg.x = (keyboard_value >> 12) & 0x01;
    msg.c = (keyboard_value >> 13) & 0x01;
    msg.v = (keyboard_value >> 14) & 0x01;
    msg.b = (keyboard_value >> 15) & 0x01;

    return msg;
}

LegacyRemoteControlParseResult parse_legacy_remote_control(
    const std::vector<uint8_t> & data,
    rm_message::msg::RemoteControl & remote_control)
{
    remote_control = rm_message::msg::RemoteControl{};

    if (data.empty()) {
        return LegacyRemoteControlParseResult::kNeedMoreData;
    }
    if (data[0] != kLegacyRemoteControlHeader0) {
        return LegacyRemoteControlParseResult::kInvalidHeader;
    }
    if (data.size() < 2) {
        return LegacyRemoteControlParseResult::kNeedMoreData;
    }
    if (data[1] != kLegacyRemoteControlHeader1) {
        return LegacyRemoteControlParseResult::kInvalidHeader;
    }
    if (data.size() < kLegacyRemoteControlLength) {
        return LegacyRemoteControlParseResult::kNeedMoreData;
    }

    uint16_t received_crc = 0;
    std::memcpy(
        &received_crc,
        data.data() + (kLegacyRemoteControlLength - kFrameTailLength),
        sizeof(received_crc));
    if (Get_CRC16_Check_Sum(data.data(), kLegacyRemoteControlLength - kFrameTailLength) !=
      received_crc)
    {
        return LegacyRemoteControlParseResult::kInvalidCrc16;
    }

    remote_control = decode_legacy_remote_control(data);
    return LegacyRemoteControlParseResult::kOk;
}

std::size_t find_next_candidate_start(
    const std::vector<uint8_t> & data,
    StreamParserMode mode)
{
    for (std::size_t i = 0; i < data.size(); ++i) {
        if (data[i] == kGeneralMessageHeader) {
            return i;
        }
        if (mode == StreamParserMode::kImageLink && data[i] == kLegacyRemoteControlHeader0) {
            return i;
        }
    }
    return data.size();
}

std::size_t trailing_header_prefix_to_preserve(
    const std::vector<uint8_t> & data,
    StreamParserMode mode)
{
    if (data.empty()) {
        return 0;
    }
    if (data.back() == kGeneralMessageHeader) {
        return 1;
    }
    if (mode == StreamParserMode::kImageLink && data.back() == kLegacyRemoteControlHeader0) {
        return 1;
    }
    return 0;
}

}  // namespace

FrameParseResult parse_standard_frame(
    const std::vector<uint8_t> & data,
    std::size_t start_ptr,
    ParsedFrame & frame)
{
    frame = ParsedFrame{};

    if (start_ptr >= data.size()) {
        return FrameParseResult::kNeedMoreData;
    }
    if (data[start_ptr] != kGeneralMessageHeader) {
        return FrameParseResult::kInvalidHeader;
    }
    if (data.size() - start_ptr < kFrameHeaderLength) {
        return FrameParseResult::kNeedMoreData;
    }

    FrameHeader header{};
    std::memcpy(&header, data.data() + start_ptr, sizeof(FrameHeader));

    if (Get_CRC8_Check_Sum(reinterpret_cast<const uint8_t *>(&header),
      sizeof(FrameHeader) - 1) != header.crc8)
    {
        return FrameParseResult::kInvalidCrc8;
    }
    if (header.data_length > kMaxStandardPayloadLength) {
        return FrameParseResult::kInvalidLength;
    }

    const std::size_t total_length =
      kFrameHeaderLength + kCommandIdLength + header.data_length + kFrameTailLength;
    if (total_length < kMinimumFrameLength) {
        return FrameParseResult::kInvalidLength;
    }
    if (data.size() - start_ptr < total_length) {
        return FrameParseResult::kNeedMoreData;
    }

    std::memcpy(
        &frame.command_id,
        data.data() + start_ptr + kFrameHeaderLength,
        sizeof(frame.command_id));
    frame.payload.assign(
        data.begin() + start_ptr + kFrameHeaderLength + kCommandIdLength,
        data.begin() + start_ptr + kFrameHeaderLength + kCommandIdLength + header.data_length);
    frame.total_length = total_length;

    uint16_t received_crc = 0;
    std::memcpy(
        &received_crc,
        data.data() + start_ptr + total_length - kFrameTailLength,
        sizeof(received_crc));
    if (Get_CRC16_Check_Sum(data.data() + start_ptr,
      total_length - kFrameTailLength) != received_crc)
    {
        return FrameParseResult::kInvalidCrc16;
    }

    return FrameParseResult::kOk;
}

StreamParseOutcome extract_next_packet(
    std::vector<uint8_t> & buffer,
    StreamParserMode mode,
    ParsedPacket & packet)
{
    packet = ParsedPacket();

    if (buffer.empty()) {
        return {};
    }

    const auto candidate_start = find_next_candidate_start(buffer, mode);
    if (candidate_start == buffer.size()) {
        const auto preserved = trailing_header_prefix_to_preserve(buffer, mode);
        if (buffer.size() <= preserved) {
            return {};
        }
        erase_prefix(buffer, buffer.size() - preserved);
        return {StreamParseResult::kSkippedBytes, StreamParserEvent::kSkippedNoise};
    }
    if (candidate_start > 0) {
        erase_prefix(buffer, candidate_start);
        return {StreamParseResult::kSkippedBytes, StreamParserEvent::kSkippedNoise};
    }

    if (mode == StreamParserMode::kImageLink && buffer.front() == kLegacyRemoteControlHeader0) {
        rm_message::msg::RemoteControl remote_control;
        switch (parse_legacy_remote_control(buffer, remote_control)) {
          case LegacyRemoteControlParseResult::kOk:
              packet.type = ParsedPacketType::kLegacyRemoteControl;
              packet.remote_control = std::move(remote_control);
              erase_prefix(buffer, kLegacyRemoteControlLength);
              return {StreamParseResult::kOk, StreamParserEvent::kNone};
          case LegacyRemoteControlParseResult::kNeedMoreData:
              return {};
          case LegacyRemoteControlParseResult::kInvalidHeader:
              erase_prefix(buffer, 1);
              return {StreamParseResult::kSkippedBytes, StreamParserEvent::kInvalidLegacyHeader};
          case LegacyRemoteControlParseResult::kInvalidCrc16:
              erase_prefix(buffer, 1);
              return {StreamParseResult::kSkippedBytes, StreamParserEvent::kInvalidLegacyCrc16};
        }
    }

    ParsedFrame frame;
    switch (parse_standard_frame(buffer, 0, frame)) {
      case FrameParseResult::kOk:
          packet.type = ParsedPacketType::kStandardFrame;
          packet.frame = std::move(frame);
          erase_prefix(buffer, packet.frame.total_length);
          return {StreamParseResult::kOk, StreamParserEvent::kNone};
      case FrameParseResult::kNeedMoreData:
          return {};
      case FrameParseResult::kInvalidHeader:
          erase_prefix(buffer, 1);
          return {StreamParseResult::kSkippedBytes, StreamParserEvent::kSkippedNoise};
      case FrameParseResult::kInvalidCrc8:
          erase_prefix(buffer, 1);
          return {StreamParseResult::kSkippedBytes, StreamParserEvent::kInvalidStandardCrc8};
      case FrameParseResult::kInvalidLength:
          erase_prefix(buffer, 1);
          return {StreamParseResult::kSkippedBytes, StreamParserEvent::kInvalidStandardLength};
      case FrameParseResult::kInvalidCrc16:
          erase_prefix(buffer, 1);
          return {StreamParseResult::kSkippedBytes, StreamParserEvent::kInvalidStandardCrc16};
    }

    return {};
}

}  // namespace RMManager
