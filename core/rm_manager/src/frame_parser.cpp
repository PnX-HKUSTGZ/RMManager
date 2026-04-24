#include "rm_manager/frame_parser.hpp"
#include "rm_manager/util.hpp"

#include <cstring>

namespace RMManager
{

namespace
{

constexpr uint8_t kGeneralMessageHeader = 0xA5;
constexpr std::size_t kFrameHeaderLength = sizeof(FrameHeader);
constexpr std::size_t kCommandIdLength = 2;
constexpr std::size_t kFrameTailLength = 2;
constexpr std::size_t kMinimumFrameLength =
  kFrameHeaderLength + kCommandIdLength + kFrameTailLength;

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

}  // namespace RMManager
