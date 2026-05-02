#include "rm_ui/ui.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "rclcpp_components/register_node_macro.hpp"

namespace rm_ui
{
namespace
{

void appendUint16(std::vector<uint8_t> & buffer, uint16_t value)
{
    buffer.push_back(static_cast<uint8_t>(value & 0xffu));
    buffer.push_back(static_cast<uint8_t>((value >> 8) & 0xffu));
}

void appendUint32(std::vector<uint8_t> & buffer, uint32_t value)
{
    buffer.push_back(static_cast<uint8_t>(value & 0xffu));
    buffer.push_back(static_cast<uint8_t>((value >> 8) & 0xffu));
    buffer.push_back(static_cast<uint8_t>((value >> 16) & 0xffu));
    buffer.push_back(static_cast<uint8_t>((value >> 24) & 0xffu));
}

void appendFigureRecord(
    std::vector<uint8_t> & buffer,
    const std::array<uint8_t, 15> & record)
{
    buffer.insert(buffer.end(), record.begin(), record.end());
}

std::string FigureNameToString(const std::array<uint8_t, 3> & name)
{
    return std::string(name.data(), name.data() + name.size());
}

std::pair<size_t, uint16_t> batchShapeForCount(size_t count)
{
    if (count == 1) {
        return {1, 0x0101};
    }
    if (count == 2) {
        return {2, 0x0102};
    }
    if (count <= 5) {
        return {5, 0x0103};
    }
    return {7, 0x0104};
}

bool isPrintableAscii(const std::string & value)
{
    return std::all_of(
        value.begin(),
        value.end(),
        [](char ch) {
            const auto byte = static_cast<unsigned char>(ch);
            return byte >= 0x20u && byte <= 0x7eu;
        });
}

} // namespace

RmUi::RmUi(const rclcpp::NodeOptions & options)
  : Node("rm_ui", options)
{
    sender_topic_ = this->declare_parameter<std::string>("sender_topic", "send_message");
    const double sender_hz = this->declare_parameter<double>("sender_hz", 30.0);
    const int sender_target = this->declare_parameter<int>("sender_target", 2);
    const int sender_id = this->declare_parameter<int>("sender_id", 0);
    const int receiver_id = this->declare_parameter<int>("receiver_id", 0);

    if (sender_hz <= 0.0) {
        throw std::runtime_error("sender_hz must be greater than 0");
    }
    if (sender_target != 1 && sender_target != 2) {
        throw std::runtime_error("sender_target must be 1 (image link) or 2 (referee link)");
    }
    if (sender_id < 1 || sender_id > 0xffff) {
        throw std::runtime_error("sender_id must be configured in range 1..65535");
    }
    if (receiver_id < 1 || receiver_id > 0xffff) {
        throw std::runtime_error("receiver_id must be configured in range 1..65535");
    }

    sender_target_ = static_cast<uint8_t>(sender_target);
    sender_id_ = static_cast<uint16_t>(sender_id);
    receiver_id_ = static_cast<uint16_t>(receiver_id);

    sender_pub_ = this->create_publisher<rm_message::msg::SendMessage>(sender_topic_, 10);
    draw_service_ = this->create_service<rm_ui::srv::DrawFigure>(
        "~/draw_figure",
        std::bind(&RmUi::handleDrawFigure, this, std::placeholders::_1, std::placeholders::_2));
    draw_shape_service_ = this->create_service<rm_ui::srv::DrawShape>(
        "~/draw_shape",
        std::bind(&RmUi::handleDrawShape, this, std::placeholders::_1, std::placeholders::_2));
    delete_service_ = this->create_service<rm_ui::srv::DeleteLayer>(
        "~/delete_layer",
        std::bind(&RmUi::handleDeleteLayer, this, std::placeholders::_1, std::placeholders::_2));
    delete_shape_service_ = this->create_service<rm_ui::srv::DeleteShape>(
        "~/delete_shape",
        std::bind(&RmUi::handleDeleteShape, this, std::placeholders::_1, std::placeholders::_2));
    redraw_trigger_service_ = this->create_service<std_srvs::srv::Trigger>(
        "~/redraw",
        std::bind(&RmUi::handleRedrawTrigger, this, std::placeholders::_1, std::placeholders::_2));


    const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / sender_hz));
    update_timer_ = this->create_wall_timer(period, std::bind(&RmUi::update, this));

    RCLCPP_INFO(
        this->get_logger(),
        "rm_ui initialized: sender_topic=%s target=%d sender_id=%u receiver_id=%u hz=%.3f",
        sender_topic_.c_str(), sender_target_, sender_id_, receiver_id_, sender_hz);
}

bool RmUi::fitsUnsignedBits(uint32_t value, uint8_t bits)
{
    return bits < 32 && value < (1u << bits);
}

bool RmUi::isValidFigureType(uint32_t value)
{
    return value <= kMaxFigureType;
}

RmUi::FigureType RmUi::toFigureType(uint32_t value)
{
    return static_cast<FigureType>(value);
}

bool RmUi::validateDrawRequest(
    const rm_ui::srv::DrawFigure::Request & request,
    std::string & error_message) const
{
    if (!isValidFigureType(request.figure_type)) {
        error_message = "figure_type must be in range 0..7";
        return false;
    }
    if (request.layer > 9) {
        error_message = "layer must be in range 0..9";
        return false;
    }
    if (request.color > 8) {
        error_message = "color must be in range 0..8";
        return false;
    }
    if (!fitsUnsignedBits(request.details_a, 9)) {
        error_message = "details_a must fit in 9 bits";
        return false;
    }
    if (!fitsUnsignedBits(request.details_b, 9)) {
        error_message = "details_b must fit in 9 bits";
        return false;
    }
    if (!fitsUnsignedBits(request.width, 10)) {
        error_message = "width must fit in 10 bits";
        return false;
    }
    if (!fitsUnsignedBits(request.start_x, 11)) {
        error_message = "start_x must fit in 11 bits";
        return false;
    }
    if (!fitsUnsignedBits(request.start_y, 11)) {
        error_message = "start_y must fit in 11 bits";
        return false;
    }
    if (!fitsUnsignedBits(request.details_c, 10)) {
        error_message = "details_c must fit in 10 bits";
        return false;
    }
    if (!fitsUnsignedBits(request.details_d, 11)) {
        error_message = "details_d must fit in 11 bits";
        return false;
    }
    if (!fitsUnsignedBits(request.details_e, 11)) {
        error_message = "details_e must fit in 11 bits";
        return false;
    }

    if (toFigureType(request.figure_type) == FigureType::String) {
        if (request.chars.size() > kStringLength) {
            error_message = "chars length must be at most 30 for string figures";
            return false;
        }
        if (request.details_b != request.chars.size()) {
            error_message = "details_b must equal chars length for string figures";
            return false;
        }
    } else if (!request.chars.empty()) {
        error_message = "chars must be empty for non-string figures";
        return false;
    }

    return true;
}

bool RmUi::buildShapeFigure(
    const rm_ui::srv::DrawShape::Request & request,
    Figure & figure,
    std::string & error_message) const
{
    if (!isValidFigureType(request.figure_type)) {
        error_message = "figure_type must be in range 0..7";
        return false;
    }
    if (request.layer > 9) {
        error_message = "layer must be in range 0..9";
        return false;
    }
    if (request.color > 8) {
        error_message = "color must be in range 0..8";
        return false;
    }
    if (!fitsUnsignedBits(request.width, 10)) {
        error_message = "width must fit in 10 bits";
        return false;
    }
    if (!fitsUnsignedBits(request.start_x, 11)) {
        error_message = "start_x must fit in 11 bits";
        return false;
    }
    if (!fitsUnsignedBits(request.start_y, 11)) {
        error_message = "start_y must fit in 11 bits";
        return false;
    }

    const auto rejectNonZero = [&error_message](bool is_nonzero, const char * field) {
          if (is_nonzero) {
              error_message = std::string(field) + " must be zero for this figure_type";
              return false;
          }
          return true;
      };
    const auto rejectNonEmpty = [&error_message](bool is_nonempty, const char * field) {
          if (is_nonempty) {
              error_message = std::string(field) + " must be empty for this figure_type";
              return false;
          }
          return true;
      };
    const auto rejectCommonUnused = [&]() {
          return rejectNonZero(request.end_x != 0, "end_x") &&
                 rejectNonZero(request.end_y != 0, "end_y") &&
                 rejectNonZero(request.radius != 0, "radius") &&
                 rejectNonZero(request.x_semiaxis != 0, "x_semiaxis") &&
                 rejectNonZero(request.y_semiaxis != 0, "y_semiaxis") &&
                 rejectNonZero(request.start_angle != 0, "start_angle") &&
                 rejectNonZero(request.end_angle != 0, "end_angle") &&
                 rejectNonZero(request.font_size != 0, "font_size") &&
                 rejectNonZero(request.float_value != 0.0, "float_value") &&
                 rejectNonZero(request.int_value != 0, "int_value") &&
                 rejectNonEmpty(!request.text.empty(), "text");
      };
    const auto rejectTextValueUnused = [&]() {
          return rejectNonZero(request.float_value != 0.0, "float_value") &&
                 rejectNonZero(request.int_value != 0, "int_value") &&
                 rejectNonEmpty(!request.text.empty(), "text");
      };
    const auto rejectGeometryUnused = [&]() {
          return rejectNonZero(request.end_x != 0, "end_x") &&
                 rejectNonZero(request.end_y != 0, "end_y") &&
                 rejectNonZero(request.radius != 0, "radius") &&
                 rejectNonZero(request.x_semiaxis != 0, "x_semiaxis") &&
                 rejectNonZero(request.y_semiaxis != 0, "y_semiaxis") &&
                 rejectNonZero(request.start_angle != 0, "start_angle") &&
                 rejectNonZero(request.end_angle != 0, "end_angle");
      };
    const auto setPackedDetails = [&figure](uint32_t value) {
          figure.details_c = value & 0x3ffu;
          figure.details_d = (value >> 10) & 0x7ffu;
          figure.details_e = (value >> 21) & 0x7ffu;
      };

    figure = Figure{};
    figure.name = request.figure_name;
    figure.figure_type = request.figure_type;
    figure.layer = request.layer;
    figure.color = request.color;
    figure.width = request.width;
    figure.start_x = request.start_x;
    figure.start_y = request.start_y;

    switch (toFigureType(request.figure_type)) {
      case FigureType::Line:
      case FigureType::Rect:
          if (!fitsUnsignedBits(request.end_x, 11)) {
              error_message = "end_x must fit in 11 bits";
              return false;
          }
          if (!fitsUnsignedBits(request.end_y, 11)) {
              error_message = "end_y must fit in 11 bits";
              return false;
          }
          if (!rejectNonZero(request.radius != 0, "radius") ||
            !rejectNonZero(request.x_semiaxis != 0, "x_semiaxis") ||
            !rejectNonZero(request.y_semiaxis != 0, "y_semiaxis") ||
            !rejectNonZero(request.start_angle != 0, "start_angle") ||
            !rejectNonZero(request.end_angle != 0, "end_angle") ||
            !rejectTextValueUnused())
          {
              return false;
          }
          figure.details_d = request.end_x;
          figure.details_e = request.end_y;
          return true;
      case FigureType::Circle:
          if (!fitsUnsignedBits(request.radius, 10)) {
              error_message = "radius must fit in 10 bits";
              return false;
          }
          if (!rejectNonZero(request.end_x != 0, "end_x") ||
            !rejectNonZero(request.end_y != 0, "end_y") ||
            !rejectNonZero(request.x_semiaxis != 0, "x_semiaxis") ||
            !rejectNonZero(request.y_semiaxis != 0, "y_semiaxis") ||
            !rejectNonZero(request.start_angle != 0, "start_angle") ||
            !rejectNonZero(request.end_angle != 0, "end_angle") ||
            !rejectTextValueUnused())
          {
              return false;
          }
          figure.details_c = request.radius;
          return true;
      case FigureType::Ellipse:
          if (!fitsUnsignedBits(request.x_semiaxis, 11)) {
              error_message = "x_semiaxis must fit in 11 bits";
              return false;
          }
          if (!fitsUnsignedBits(request.y_semiaxis, 11)) {
              error_message = "y_semiaxis must fit in 11 bits";
              return false;
          }
          if (!rejectNonZero(request.end_x != 0, "end_x") ||
            !rejectNonZero(request.end_y != 0, "end_y") ||
            !rejectNonZero(request.radius != 0, "radius") ||
            !rejectNonZero(request.start_angle != 0, "start_angle") ||
            !rejectNonZero(request.end_angle != 0, "end_angle") ||
            !rejectTextValueUnused())
          {
              return false;
          }
          figure.details_d = request.x_semiaxis;
          figure.details_e = request.y_semiaxis;
          return true;
      case FigureType::Arc:
          if (!fitsUnsignedBits(request.x_semiaxis, 11)) {
              error_message = "x_semiaxis must fit in 11 bits";
              return false;
          }
          if (!fitsUnsignedBits(request.y_semiaxis, 11)) {
              error_message = "y_semiaxis must fit in 11 bits";
              return false;
          }
        //   if (request.start_angle > 360) {
        //       error_message = "start_angle must be in range 0..360";
        //       return false;
        //   }
        //   if (request.end_angle > 360) {
        //       error_message = "end_angle must be in range 0..360";
        //       return false;
        //   }
          if (!rejectNonZero(request.end_x != 0, "end_x") ||
            !rejectNonZero(request.end_y != 0, "end_y") ||
            !rejectNonZero(request.radius != 0, "radius") ||
            !rejectTextValueUnused())
          {
              return false;
          }
          figure.details_a = request.start_angle;
          figure.details_b = request.end_angle;
          figure.details_d = request.x_semiaxis;
          figure.details_e = request.y_semiaxis;
          return true;
      case FigureType::Float: {
            if (!fitsUnsignedBits(request.font_size, 9)) {
                error_message = "font_size must fit in 9 bits";
                return false;
            }
            if (!rejectGeometryUnused() ||
              !rejectNonZero(request.int_value != 0, "int_value") ||
              !rejectNonEmpty(!request.text.empty(), "text"))
            {
                return false;
            }
            if (!std::isfinite(request.float_value)) {
                error_message = "float_value must be finite";
                return false;
            }
            if (request.float_value < 0.0) {
                error_message = "float_value must be non-negative";
                return false;
            }
            const double scaled = request.float_value * 1000.0;
            if (scaled > static_cast<double>(std::numeric_limits<uint32_t>::max())) {
                error_message = "float_value is too large after scaling";
                return false;
            }
            figure.details_a = request.font_size;
            setPackedDetails(static_cast<uint32_t>(scaled));
            return true;
        }
      case FigureType::Int: {
            if (!fitsUnsignedBits(request.font_size, 9)) {
                error_message = "font_size must fit in 9 bits";
                return false;
            }
            if (!rejectGeometryUnused() ||
              !rejectNonZero(request.float_value != 0.0, "float_value") ||
              !rejectNonEmpty(!request.text.empty(), "text"))
            {
                return false;
            }
            figure.details_a = request.font_size;
            setPackedDetails(static_cast<uint32_t>(request.int_value));
            return true;
        }
      case FigureType::String:
          if (!fitsUnsignedBits(request.font_size, 9)) {
              error_message = "font_size must fit in 9 bits";
              return false;
          }
          if (!rejectGeometryUnused() ||
            !rejectNonZero(request.float_value != 0.0, "float_value") ||
            !rejectNonZero(request.int_value != 0, "int_value"))
          {
              return false;
          }
          if (request.text.size() > kStringLength) {
              error_message = "text length must be at most 30 bytes";
              return false;
          }
          if (!isPrintableAscii(request.text)) {
              error_message = "text must contain printable ASCII only";
              return false;
          }
          figure.details_a = request.font_size;
          figure.details_b = request.text.size();
          std::copy(request.text.begin(), request.text.end(), figure.chars.begin());
          return true;
      default:
          break;
    }

    return rejectCommonUnused();
}

void RmUi::handleDrawFigure(
    const std::shared_ptr<rm_ui::srv::DrawFigure::Request> request,
    std::shared_ptr<rm_ui::srv::DrawFigure::Response> response)
{
    std::string error_message;
    if (!validateDrawRequest(*request, error_message)) {
        response->success = false;
        response->message = error_message;
        RCLCPP_WARN(this->get_logger(), "Invalid draw figure request: %s", error_message.c_str());
        return;
    }

    Figure figure;
    figure.name = request->figure_name;
    figure.figure_type = request->figure_type;
    figure.layer = request->layer;
    figure.color = request->color;
    figure.details_a = request->details_a;
    figure.details_b = request->details_b;
    figure.width = request->width;
    figure.start_x = request->start_x;
    figure.start_y = request->start_y;
    figure.details_c = request->details_c;
    figure.details_d = request->details_d;
    figure.details_e = request->details_e;
    std::copy(request->chars.begin(), request->chars.end(), figure.chars.begin());

    {
        std::lock_guard<std::mutex> lock(mutex_);
        const auto existing = cached_figures_.find(figure.name);
        if (existing != cached_figures_.end() &&
          existing->second.figure_type != figure.figure_type)
        {
            response->success = false;
            response->message = "figure_type cannot change for an existing name";
            RCLCPP_WARN(this->get_logger(), "Invalid draw figure request: figure_type cannot change for an existing name");
            return;
        }
        enqueueFigureLocked(figure);
    }

    response->success = true;
    response->message = "figure accepted";
    RCLCPP_INFO(this->get_logger(), "Draw figure request accepted: name=%s type=%d layer=%d color=%d",
        FigureNameToString(request->figure_name).c_str(), request->figure_type, request->layer, request->color);
}

void RmUi::handleDrawShape(
    const std::shared_ptr<rm_ui::srv::DrawShape::Request> request,
    std::shared_ptr<rm_ui::srv::DrawShape::Response> response)
{
    Figure figure;
    std::string error_message;
    if (!buildShapeFigure(*request, figure, error_message)) {
        response->success = false;
        response->message = error_message;
        RCLCPP_WARN(this->get_logger(), "Invalid draw shape request: %s", error_message.c_str());
        return;
    }

    {
        std::lock_guard<std::mutex> lock(mutex_);
        const auto existing = cached_figures_.find(figure.name);
        if (existing != cached_figures_.end() &&
          existing->second.figure_type != figure.figure_type)
        {
            response->success = false;
            response->message = "figure_type cannot change for an existing name";
            RCLCPP_WARN(this->get_logger(), "Invalid draw shape request: figure_type cannot change for an existing name");
            return;
        }
        enqueueFigureLocked(figure);
    }

    response->success = true;
    response->message = "shape accepted";
    RCLCPP_INFO(this->get_logger(), "Draw shape request accepted: name=%s type=%d layer=%d color=%d",
        FigureNameToString(request->figure_name).c_str(), request->figure_type, request->layer, request->color);
}

void RmUi::handleDeleteLayer(
    const std::shared_ptr<rm_ui::srv::DeleteLayer::Request> request,
    std::shared_ptr<rm_ui::srv::DeleteLayer::Response> response)
{
    const int layer = static_cast<int>(request->layer);
    std::lock_guard<std::mutex> lock(mutex_);

    if (layer == -1) {
        pending_palettes_.push_back(PendingDelete{PendingDelete::DELETE_ALL_LAYER, 0});
        cached_figures_.clear();
        // pending_figures_.clear();
        response->success = true;
        response->message = "delete all accepted";
        RCLCPP_INFO(this->get_logger(), "Delete all request accepted");
        return;
    }

    if (layer < 0 || layer > 9) {
        response->success = false;
        response->message = "layer must be -1 or in range 0..9";
        RCLCPP_WARN(this->get_logger(), "Invalid delete layer request: layer %d is out of range",
            layer);
        return;
    }

    const auto layer_u8 = static_cast<uint8_t>(layer);
    pending_palettes_.push_back(PendingDelete{1, layer_u8});
    // for (auto & pending : pending_figures_) {
    //     if (pending.operation == Operation::Modify &&
    //       pending.had_previous &&
    //       pending.previous_layer == layer_u8 &&
    //       pending.figure.layer != layer_u8)
    //     {
    //         pending.operation = Operation::Add;
    //         pending.had_previous = false;
    //     }
    // }
    eraseLayerFromCacheLocked(layer_u8);
    // eraseLayerFromPendingFigures(layer_u8);
    response->success = true;
    response->message = "delete layer accepted";
    RCLCPP_INFO(this->get_logger(), "Delete layer request accepted: %d", layer);
}

void RmUi::handleDeleteShape(
    const std::shared_ptr<rm_ui::srv::DeleteShape::Request> request,
    std::shared_ptr<rm_ui::srv::DeleteShape::Response> response)
{
    std::lock_guard<std::mutex> lock(mutex_);

    // RCLCPP_INFO_STREAM(this->get_logger(), "Received delete shape request: name="
    //     << int(request->figure_name[0]) <<  int(request->figure_name[1]) << int(request->figure_name[2]));

    if (request->figure_name.empty() || request->figure_name.size() > 3) {
        response->success = false;
        response->message = "figure_name must be 1 to 3 bytes";
        RCLCPP_WARN(this->get_logger(), "Invalid delete shape request: figure_name length %zu is out of range",
            request->figure_name.size());
        return;
    }

    // build a 3-byte name array, padding with zeros if necessary
    FigureName figure_name{};
    std::copy(request->figure_name.begin(), request->figure_name.end(), figure_name.begin());

    // check if the figure exists in cache
    const auto existing = cached_figures_.find(figure_name);
    if (existing == cached_figures_.end()) {
        response->success = false;
        response->message = "figure with the given name does not exist";
        RCLCPP_WARN(this->get_logger(), "Invalid delete shape request: figure with name %s does not exist",
            FigureNameToString(request->figure_name).c_str());
        return;
    }

    pending_palettes_.push_back(PendingFigure{existing->second, Operation::Delete, existing->second.layer, true});
    cached_figures_.erase(existing);

    response->success = true;
    response->message = "delete shape accepted";
    RCLCPP_INFO(this->get_logger(), "Delete shape request accepted: %s", FigureNameToString(request->figure_name).c_str());

    return;
}

void RmUi::handleRedrawTrigger(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    (void)request;
    {
        std::lock_guard<std::mutex> lock(mutex_);

        // remove all layers
        pending_palettes_.push_back(PendingDelete{PendingDelete::DELETE_ALL_LAYER, 0});

        for (const auto & entry : cached_figures_) {
            const auto & figure = entry.second;
            pending_palettes_.push_back(PendingFigure{figure, Operation::Add, figure.layer, true});
        }
    }
    response->success = true;
    response->message = "redraw triggered";
    RCLCPP_INFO(this->get_logger(), "Redraw trigger request accepted");
}

void RmUi::update()
{
    std::optional<std::vector<uint8_t>> payload;

    {
        std::lock_guard<std::mutex> lock(mutex_);
        if(!pending_palettes_.empty()){
            // if the type is delete layer
            if(std::holds_alternative<PendingDelete>(pending_palettes_.front())){
                const PendingDelete delete_op = std::get<PendingDelete>(pending_palettes_.front());
                payload = buildInteractionPayload(kDeleteContentId, buildDeleteUserData(delete_op));
                pending_palettes_.pop_front();
            }
            else if(std::get<PendingFigure>(pending_palettes_.front()).figure.figure_type != 
                kStringFigureType){
                // count the number of consecutive PendingFigure
                std::vector<PendingFigure> batch;
                batch.reserve(kMaxBatchFigureCount);
                // until the type is PendingDelete
                while(!pending_palettes_.empty() && 
                    !std::holds_alternative<PendingDelete>(pending_palettes_.front())){
                    
                    const auto pending = pending_palettes_.front();
                    if(std::holds_alternative<PendingFigure>(pending)){
                        if(std::get<PendingFigure>(pending).figure.figure_type == kStringFigureType){
                            break;
                        }
                        batch.push_back(std::get<PendingFigure>(pending));
                    }
                    pending_palettes_.pop_front();

                    // check if the batch is full
                    if(batch.size() >= kMaxBatchFigureCount){
                        break;
                    }

                }

                // build the payload for the batch of PendingFigure
                const auto batch_shape = batchShapeForCount(batch.size());
                payload = buildInteractionPayload(
                    batch_shape.second,
                    buildFigureBatchUserData(batch));
            }
            else if(std::get<PendingFigure>(pending_palettes_.front()).figure.figure_type == 
                    kStringFigureType){
                const PendingFigure pending = std::get<PendingFigure>(pending_palettes_.front());
                payload = buildInteractionPayload(kStringContentId, buildStringUserData(pending));
                pending_palettes_.pop_front();
            }
            else{
                // this should not happen, but just in case
                pending_palettes_.pop_front();
                RCLCPP_ERROR(this->get_logger(), "Invalid pending palette type progress while building payload");
            }
        }
    }

    if (payload.has_value()) {
        publishInteractionPayload(payload.value());
    }

    // log all the cached figures for debugging
    // {
    //     std::lock_guard<std::mutex> lock(mutex_);
    //     RCLCPP_INFO(this->get_logger(), "Cached figures:");
    //     for (const auto & entry : cached_figures_) {
    //         const auto & figure = entry.second;
    //         RCLCPP_INFO(this->get_logger(), "  name=%s type=%d layer=%d color=%d details_a=%d details_b=%d width=%d start_x=%d start_y=%d details_c=%d details_d=%d details_e=%d",
    //             FigureNameToString(figure.name).c_str(), figure.figure_type, figure.layer, figure.color,
    //             figure.details_a, figure.details_b, figure.width, figure.start_x, figure.start_y, figure.details_c, figure.details_d, figure.details_e);
    //     }
    // }


}

void RmUi::publishInteractionPayload(const std::vector<uint8_t> & payload)
{
    rm_message::msg::SendMessage message;
    message.target = sender_target_;
    message.cmd_id = kInteractionCmdId;
    message.data_length = static_cast<uint16_t>(payload.size());
    message.data_payload = payload;
    sender_pub_->publish(message);
}

std::vector<uint8_t> RmUi::buildInteractionPayload(
    uint16_t content_id,
    const std::vector<uint8_t> & user_data) const
{
    std::vector<uint8_t> payload;
    payload.reserve(6 + user_data.size());
    appendUint16(payload, content_id);
    appendUint16(payload, sender_id_);
    appendUint16(payload, receiver_id_);
    payload.insert(payload.end(), user_data.begin(), user_data.end());
    return payload;
}

std::vector<uint8_t> RmUi::buildDeleteUserData(const PendingDelete & delete_op) const
{
    return {delete_op.delete_type, delete_op.layer};
}

std::vector<uint8_t> RmUi::buildStringUserData(const PendingFigure & pending) const
{
    std::vector<uint8_t> user_data;
    user_data.reserve(kFigureRecordLength + kStringLength);
    appendFigureRecord(user_data, packFigure(pending.figure, pending.operation));
    user_data.insert(user_data.end(), pending.figure.chars.begin(), pending.figure.chars.end());
    return user_data;
}

std::vector<uint8_t> RmUi::buildFigureBatchUserData(std::vector<PendingFigure> & batch) const
{
    const auto batch_shape = batchShapeForCount(batch.size());
    const size_t slot_count = batch_shape.first;

    std::vector<uint8_t> user_data;
    user_data.reserve(slot_count * kFigureRecordLength);
    for (const auto & pending : batch) {
        appendFigureRecord(user_data, packFigure(pending.figure, pending.operation));
    }

    const Figure noop_figure;
    for (size_t i = batch.size(); i < slot_count; ++i) {
        appendFigureRecord(user_data, packFigure(noop_figure, Operation::Noop));
    }
    return user_data;
}

void RmUi::eraseLayerFromCacheLocked(uint8_t layer)
{
    for (auto it = cached_figures_.begin(); it != cached_figures_.end(); ) {
        if (it->second.layer == layer) {
            it = cached_figures_.erase(it);
        } else {
            ++it;
        }
    }
}

void RmUi::enqueueFigureLocked(const Figure & figure)
{
    const auto existing = cached_figures_.find(figure.name);
    const bool had_previous = existing != cached_figures_.end();
    const Operation operation = had_previous ? Operation::Modify : Operation::Add;
    // const uint32_t previous_layer = had_previous ? existing->second.layer : 0;
    cached_figures_[figure.name] = figure;
    pending_palettes_.push_back(PendingFigure{figure, operation, figure.layer, had_previous});
}

std::array<uint8_t, RmUi::kFigureRecordLength> RmUi::packFigure(
    const Figure & figure,
    Operation operation)
{
    std::array<uint8_t, kFigureRecordLength> record{};
    record[0] = figure.name[0];
    record[1] = figure.name[1];
    record[2] = figure.name[2];

    const uint32_t dword1 =
      (static_cast<uint32_t>(operation) & 0x7u) |
      ((figure.figure_type & 0x7u) << 3) |
      ((figure.layer & 0xfu) << 6) |
      ((figure.color & 0xfu) << 10) |
      ((figure.details_a & 0x1ffu) << 14) |
      ((figure.details_b & 0x1ffu) << 23);
    const uint32_t dword2 =
      (figure.width & 0x3ffu) |
      ((figure.start_x & 0x7ffu) << 10) |
      ((figure.start_y & 0x7ffu) << 21);
    const uint32_t dword3 =
      (figure.details_c & 0x3ffu) |
      ((figure.details_d & 0x7ffu) << 10) |
      ((figure.details_e & 0x7ffu) << 21);

    std::vector<uint8_t> dwords;
    dwords.reserve(12);
    appendUint32(dwords, dword1);
    appendUint32(dwords, dword2);
    appendUint32(dwords, dword3);
    std::copy(dwords.begin(), dwords.end(), record.begin() + 3);
    return record;
}

} // namespace rm_ui

RCLCPP_COMPONENTS_REGISTER_NODE(rm_ui::RmUi)
