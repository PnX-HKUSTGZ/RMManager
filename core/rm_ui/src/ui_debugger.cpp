#include "rm_ui/ui_debugger.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <stdexcept>
#include <utility>

#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/imgproc.hpp>

#include "rclcpp_components/register_node_macro.hpp"
#include "std_msgs/msg/header.hpp"

namespace rm_ui
{
namespace
{

std::string figureNameToString(const std::array<uint8_t, 3> & name)
{
    std::string result;
    result.reserve(3);
    for (const uint8_t ch : name) {
        if (ch == 0) {
            result.push_back('.');
        } else {
            result.push_back(static_cast<char>(ch));
        }
    }
    return result;
}

std::string charsToString(const std::array<uint8_t, 30> & chars, size_t length)
{
    std::string result;
    const size_t safe_length = std::min(length, chars.size());
    result.reserve(safe_length);
    for (size_t i = 0; i < safe_length; ++i) {
        if (chars[i] == 0) {
            break;
        }
        result.push_back(static_cast<char>(chars[i]));
    }
    return result;
}

} // namespace

RmUiDebugger::RmUiDebugger(const rclcpp::NodeOptions & options)
  : Node("rm_ui_debugger", options)
{
    input_topic_ = this->declare_parameter<std::string>("input_topic", "send_message");
    image_topic_ = this->declare_parameter<std::string>("image_topic", "ui_debug/image");
    frame_id_ = this->declare_parameter<std::string>("frame_id", "rm_ui_debug");
    image_width_ = this->declare_parameter<int>("image_width", 1920);
    image_height_ = this->declare_parameter<int>("image_height", 1080);
    protocol_width_ = this->declare_parameter<int>("protocol_width", 1920);
    protocol_height_ = this->declare_parameter<int>("protocol_height", 1080);
    draw_names_ = this->declare_parameter<bool>("draw_names", true);
    font_scale_factor_ = this->declare_parameter<double>("font_scale_factor", 0.04);
    const double publish_hz = this->declare_parameter<double>("publish_hz", 30.0);

    if (image_width_ <= 0 || image_height_ <= 0) {
        throw std::runtime_error("image_width and image_height must be greater than 0");
    }
    if (protocol_width_ <= 0 || protocol_height_ <= 0) {
        throw std::runtime_error("protocol_width and protocol_height must be greater than 0");
    }
    if (publish_hz <= 0.0) {
        throw std::runtime_error("publish_hz must be greater than 0");
    }

    send_sub_ = this->create_subscription<rm_message::msg::SendMessage>(
        input_topic_,
        10,
        std::bind(&RmUiDebugger::handleSendMessage, this, std::placeholders::_1));
    image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(image_topic_, 10);

    const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / publish_hz));
    publish_timer_ = this->create_wall_timer(period, std::bind(&RmUiDebugger::publishImage, this));

    RCLCPP_INFO(
        this->get_logger(),
        "rm_ui_debugger initialized: input_topic=%s image_topic=%s image=%dx%d hz=%.3f",
        input_topic_.c_str(), image_topic_.c_str(), image_width_, image_height_, publish_hz);
}

void RmUiDebugger::handleSendMessage(const rm_message::msg::SendMessage::SharedPtr message)
{
    if (message->cmd_id != kInteractionCmdId) {
        return;
    }

    if (message->data_length != message->data_payload.size()) {
        RCLCPP_WARN_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            1000,
            "SendMessage data_length=%u differs from payload size=%zu; using payload size",
            message->data_length,
            message->data_payload.size());
    }

    std::lock_guard<std::mutex> lock(mutex_);
    if (!parseInteractionPayload(message->data_payload)) {
        RCLCPP_WARN_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            1000,
            "Failed to parse rm_ui debug payload");
    }
}

void RmUiDebugger::publishImage()
{
    cv::Mat image;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        image = renderImage();
    }

    std_msgs::msg::Header header;
    header.stamp = this->now();
    header.frame_id = frame_id_;
    cv_bridge::CvImage cv_image(header, "bgr8", image);
    image_pub_->publish(*cv_image.toImageMsg());
}

cv::Mat RmUiDebugger::renderImage() const
{
    cv::Mat image(image_height_, image_width_, CV_8UC3, cv::Scalar(20, 20, 20));

    std::vector<Figure> ordered_figures;
    ordered_figures.reserve(figures_.size());
    for (const auto & item : figures_) {
        ordered_figures.push_back(item.second);
    }
    std::sort(
        ordered_figures.begin(),
        ordered_figures.end(),
        [](const Figure & lhs, const Figure & rhs) {
            if (lhs.layer != rhs.layer) {
                return lhs.layer < rhs.layer;
            }
            return lhs.name < rhs.name;
        });

    for (const auto & figure : ordered_figures) {
        drawFigure(image, figure);
        if (draw_names_) {
            drawLabel(image, figure);
        }
    }

    return image;
}

bool RmUiDebugger::parseInteractionPayload(const std::vector<uint8_t> & payload)
{
    if (payload.size() < 6) {
        return false;
    }

    const uint16_t content_id = readUint16(payload, 0);
    switch (content_id) {
      case kDeleteContentId:
          return parseDeleteCommand(payload);
      case kSingleFigureContentId:
          return parseFigureCommand(payload, 1);
      case kDoubleFigureContentId:
          return parseFigureCommand(payload, 2);
      case kFiveFigureContentId:
          return parseFigureCommand(payload, 5);
      case kSevenFigureContentId:
          return parseFigureCommand(payload, 7);
      case kStringContentId:
          return parseStringCommand(payload);
      default:
          return false;
    }
}

bool RmUiDebugger::parseDeleteCommand(const std::vector<uint8_t> & payload)
{
    if (payload.size() != 8) {
        return false;
    }

    const uint8_t delete_type = payload[6];
    const uint8_t layer = payload[7];
    if (delete_type == 0) {
        return true;
    }
    if (delete_type == 1) {
        for (auto it = figures_.begin(); it != figures_.end(); ) {
            if (it->second.layer == layer) {
                it = figures_.erase(it);
            } else {
                ++it;
            }
        }
        return true;
    }
    if (delete_type == 2) {
        figures_.clear();
        return true;
    }
    return false;
}

bool RmUiDebugger::parseFigureCommand(
    const std::vector<uint8_t> & payload,
    size_t figure_count)
{
    const size_t expected_size = 6 + figure_count * kFigureRecordLength;
    if (payload.size() != expected_size) {
        return false;
    }

    for (size_t i = 0; i < figure_count; ++i) {
        Operation operation = Operation::Noop;
        const Figure figure = parseFigureRecord(payload, 6 + i * kFigureRecordLength, operation);
        applyFigure(figure, operation);
    }
    return true;
}

bool RmUiDebugger::parseStringCommand(const std::vector<uint8_t> & payload)
{
    if (payload.size() != 6 + kFigureRecordLength + kStringLength) {
        return false;
    }

    Operation operation = Operation::Noop;
    Figure figure = parseFigureRecord(payload, 6, operation);
    std::copy_n(payload.begin() + 6 + kFigureRecordLength, kStringLength, figure.chars.begin());
    applyFigure(figure, operation);
    return true;
}

void RmUiDebugger::applyFigure(const Figure & figure, Operation operation)
{
    switch (operation) {
      case Operation::Noop:
          return;
      case Operation::Add:
      case Operation::Modify:
          figures_[figure.name] = figure;
          return;
      case Operation::Delete:
          figures_.erase(figure.name);
          return;
    }
}

void RmUiDebugger::drawFigure(cv::Mat & image, const Figure & figure) const
{
    const cv::Scalar color = colorFor(figure.color);
    const int thickness = lineThickness(figure.width);
    const cv::Point start = toImagePoint(figure.start_x, figure.start_y);

    switch (figure.figure_type) {
      case 0:
          cv::line(image, start, toImagePoint(figure.details_d, figure.details_e), color,
              thickness);
          break;
      case 1:
          cv::rectangle(
                image,
                start,
                toImagePoint(figure.details_d, figure.details_e),
                color,
                thickness);
          break;
      case 2:
          cv::circle(image, start, scaleLength(figure.details_c), color, thickness);
          break;
      case 3:
          cv::ellipse(
                image,
                start,
                cv::Size(scaleLength(figure.details_d), scaleLength(figure.details_e, false)),
                0.0,
                0.0,
                360.0,
                color,
                thickness);
          break;
      case 4:
          cv::ellipse(
                image,
                start,
                cv::Size(scaleLength(figure.details_d), scaleLength(figure.details_e, false)),
                0.0,
                static_cast<double>(figure.details_a),
                static_cast<double>(figure.details_b),
                color,
                thickness);
          break;
      case 5: {
            const double value = static_cast<double>(composeDetailDword3(figure)) / 1000.0;
            cv::putText(
                image,
                std::to_string(value),
                start,
                cv::FONT_HERSHEY_SIMPLEX,
                std::max(0.3, figure.details_a * font_scale_factor_),
                color,
                thickness);
            break;
        }
      case 6: {
            const auto value = static_cast<int32_t>(composeDetailDword3(figure));
            cv::putText(
                image,
                std::to_string(value),
                start,
                cv::FONT_HERSHEY_SIMPLEX,
                std::max(0.3, figure.details_a * font_scale_factor_),
                color,
                thickness);
            break;
        }
      case 7:
          cv::putText(
                image,
                charsToString(figure.chars, figure.details_b),
                start,
                cv::FONT_HERSHEY_SIMPLEX,
                std::max(0.3, figure.details_a * font_scale_factor_),
                color,
                thickness);
          break;
      default:
          break;
    }
}

void RmUiDebugger::drawLabel(cv::Mat & image, const Figure & figure) const
{
    const cv::Point point = toImagePoint(figure.start_x, figure.start_y);
    const cv::Point label_point{
        std::clamp(point.x + 4, 0, image_width_ - 1),
        std::clamp(point.y - 4, 0, image_height_ - 1)};
    cv::putText(
        image,
        figureNameToString(figure.name),
        label_point,
        cv::FONT_HERSHEY_SIMPLEX,
        0.35,
        cv::Scalar(180, 180, 180),
        1);
}

cv::Point RmUiDebugger::toImagePoint(uint32_t x, uint32_t y) const
{
    const double scaled_x = static_cast<double>(x) * image_width_ / protocol_width_;
    const double scaled_y = static_cast<double>(y) * image_height_ / protocol_height_;
    const int image_x = std::clamp(static_cast<int>(std::lround(scaled_x)), 0, image_width_ - 1);
    const int image_y = std::clamp(
        image_height_ - 1 - static_cast<int>(std::lround(scaled_y)),
        0,
        image_height_ - 1);
    return {image_x, image_y};
}

int RmUiDebugger::scaleLength(uint32_t value, bool use_x_axis) const
{
    const int protocol_size = use_x_axis ? protocol_width_ : protocol_height_;
    const int image_size = use_x_axis ? image_width_ : image_height_;
    return std::max(1, static_cast<int>(std::lround(
        static_cast<double>(value) * image_size / protocol_size)));
}

int RmUiDebugger::lineThickness(uint32_t width) const
{
    return std::max(1, scaleLength(width));
}

uint16_t RmUiDebugger::readUint16(const std::vector<uint8_t> & data, size_t offset)
{
    return static_cast<uint16_t>(data.at(offset)) |
           static_cast<uint16_t>(data.at(offset + 1)) << 8;
}

uint32_t RmUiDebugger::readUint32(const std::vector<uint8_t> & data, size_t offset)
{
    return static_cast<uint32_t>(data.at(offset)) |
           static_cast<uint32_t>(data.at(offset + 1)) << 8 |
           static_cast<uint32_t>(data.at(offset + 2)) << 16 |
           static_cast<uint32_t>(data.at(offset + 3)) << 24;
}

RmUiDebugger::Figure RmUiDebugger::parseFigureRecord(
    const std::vector<uint8_t> & data,
    size_t offset,
    Operation & operation)
{
    Figure figure;
    figure.name = {data.at(offset), data.at(offset + 1), data.at(offset + 2)};

    const uint32_t dword1 = readUint32(data, offset + 3);
    const uint32_t dword2 = readUint32(data, offset + 7);
    const uint32_t dword3 = readUint32(data, offset + 11);

    operation = static_cast<Operation>(dword1 & 0x7u);
    figure.figure_type = (dword1 >> 3) & 0x7u;
    figure.layer = (dword1 >> 6) & 0xfu;
    figure.color = (dword1 >> 10) & 0xfu;
    figure.details_a = (dword1 >> 14) & 0x1ffu;
    figure.details_b = (dword1 >> 23) & 0x1ffu;
    figure.width = dword2 & 0x3ffu;
    figure.start_x = (dword2 >> 10) & 0x7ffu;
    figure.start_y = (dword2 >> 21) & 0x7ffu;
    figure.details_c = dword3 & 0x3ffu;
    figure.details_d = (dword3 >> 10) & 0x7ffu;
    figure.details_e = (dword3 >> 21) & 0x7ffu;
    return figure;
}

uint32_t RmUiDebugger::composeDetailDword3(const Figure & figure)
{
    return (figure.details_c & 0x3ffu) |
           ((figure.details_d & 0x7ffu) << 10) |
           ((figure.details_e & 0x7ffu) << 21);
}

cv::Scalar RmUiDebugger::colorFor(uint32_t color)
{
    switch (color) {
      case 0:
          return {40, 40, 255};
      case 1:
          return {0, 255, 255};
      case 2:
          return {0, 220, 0};
      case 3:
          return {0, 140, 255};
      case 4:
          return {255, 0, 255};
      case 5:
          return {180, 105, 255};
      case 6:
          return {255, 255, 0};
      case 7:
          return {0, 0, 0};
      case 8:
          return {255, 255, 255};
      default:
          return {180, 180, 180};
    }
}

} // namespace rm_ui

RCLCPP_COMPONENTS_REGISTER_NODE(rm_ui::RmUiDebugger)
