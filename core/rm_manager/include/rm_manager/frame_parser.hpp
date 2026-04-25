#ifndef RM_MANAGER__FRAME_PARSER_HPP_
#define RM_MANAGER__FRAME_PARSER_HPP_

#include "rm_message/msg/remote_control.hpp"

#include <cstddef>
#include <cstdint>
#include <vector>

namespace RMManager
{

struct FrameHeader
{
    uint8_t sof;
    uint16_t data_length;
    uint8_t seq;
    uint8_t crc8;
} __attribute__((packed));

struct ParsedFrame
{
    uint16_t command_id{};
    std::vector<uint8_t> payload;
    std::size_t total_length{};
};

enum class FrameParseResult
{
    kOk,
    kNeedMoreData,
    kInvalidHeader,
    kInvalidCrc8,
    kInvalidLength,
    kInvalidCrc16,
};

enum class StreamParserMode
{
    kStandardOnly,
    kImageLink,
};

enum class ParsedPacketType
{
    kStandardFrame,
    kLegacyRemoteControl,
};

enum class StreamParseResult
{
    kOk,
    kNeedMoreData,
    kSkippedBytes,
};

enum class StreamParserEvent
{
    kNone,
    kSkippedNoise,
    kInvalidStandardCrc8,
    kInvalidStandardLength,
    kInvalidStandardCrc16,
    kInvalidLegacyHeader,
    kInvalidLegacyCrc16,
};

struct ParsedPacket
{
    ParsedPacketType type{ParsedPacketType::kStandardFrame};
    ParsedFrame frame;
    rm_message::msg::RemoteControl remote_control;
};

struct StreamParseOutcome
{
    StreamParseResult result{StreamParseResult::kNeedMoreData};
    StreamParserEvent event{StreamParserEvent::kNone};
};

FrameParseResult parse_standard_frame(
    const std::vector<uint8_t> & data,
    std::size_t start_ptr,
    ParsedFrame & frame);

StreamParseOutcome extract_next_packet(
    std::vector<uint8_t> & buffer,
    StreamParserMode mode,
    ParsedPacket & packet);

}  // namespace RMManager

#endif  // RM_MANAGER__FRAME_PARSER_HPP_
