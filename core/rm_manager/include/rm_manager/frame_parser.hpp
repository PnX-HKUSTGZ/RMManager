#ifndef RM_MANAGER__FRAME_PARSER_HPP_
#define RM_MANAGER__FRAME_PARSER_HPP_

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

FrameParseResult parse_standard_frame(
    const std::vector<uint8_t> & data,
    std::size_t start_ptr,
    ParsedFrame & frame);

}  // namespace RMManager

#endif  // RM_MANAGER__FRAME_PARSER_HPP_
