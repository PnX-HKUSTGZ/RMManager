#ifndef RM_UI__UI_DEBUGGER_HPP_
#define RM_UI__UI_DEBUGGER_HPP_

#include <array>
#include <cstddef>
#include <cstdint>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

#include "rm_message/msg/send_message.hpp"

namespace rm_ui
{

class RmUiDebugger : public rclcpp::Node
{
public:
    explicit RmUiDebugger(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
    ~RmUiDebugger() override = default;

private:
    enum class Operation : uint8_t
    {
        Noop = 0,
        Add = 1,
        Modify = 2,
        Delete = 3,
    };

    struct Figure
    {
        std::array<uint8_t, 3> name{};
        uint32_t figure_type{0};
        uint32_t layer{0};
        uint32_t color{0};
        uint32_t details_a{0};
        uint32_t details_b{0};
        uint32_t width{0};
        uint32_t start_x{0};
        uint32_t start_y{0};
        uint32_t details_c{0};
        uint32_t details_d{0};
        uint32_t details_e{0};
        std::array<uint8_t, 30> chars{};
    };

    using FigureName = std::array<uint8_t, 3>;

    static constexpr uint16_t kInteractionCmdId = 0x0301;
    static constexpr uint16_t kDeleteContentId = 0x0100;
    static constexpr uint16_t kSingleFigureContentId = 0x0101;
    static constexpr uint16_t kDoubleFigureContentId = 0x0102;
    static constexpr uint16_t kFiveFigureContentId = 0x0103;
    static constexpr uint16_t kSevenFigureContentId = 0x0104;
    static constexpr uint16_t kStringContentId = 0x0110;
    static constexpr size_t kFigureRecordLength = 15;
    static constexpr size_t kStringLength = 30;

    void handleSendMessage(const rm_message::msg::SendMessage::SharedPtr message);
    void publishImage();
    cv::Mat renderImage() const;

    bool parseInteractionPayload(const std::vector<uint8_t> & payload);
    bool parseDeleteCommand(const std::vector<uint8_t> & payload);
    bool parseFigureCommand(const std::vector<uint8_t> & payload, size_t figure_count);
    bool parseStringCommand(const std::vector<uint8_t> & payload);
    void applyFigure(const Figure & figure, Operation operation);

    void drawFigure(cv::Mat & image, const Figure & figure) const;
    void drawLabel(cv::Mat & image, const Figure & figure) const;
    cv::Point toImagePoint(uint32_t x, uint32_t y) const;
    int scaleLength(uint32_t value, bool use_x_axis = true) const;
    int lineThickness(uint32_t width) const;

    static uint16_t readUint16(const std::vector<uint8_t> & data, size_t offset);
    static uint32_t readUint32(const std::vector<uint8_t> & data, size_t offset);
    static Figure parseFigureRecord(
        const std::vector<uint8_t> & data,
        size_t offset,
        Operation & operation);
    static uint32_t composeDetailDword3(const Figure & figure);
    static cv::Scalar colorFor(uint32_t color);

    rclcpp::Subscription<rm_message::msg::SendMessage>::SharedPtr send_sub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
    rclcpp::TimerBase::SharedPtr publish_timer_;

    std::string input_topic_;
    std::string image_topic_;
    std::string frame_id_;
    int image_width_{1920};
    int image_height_{1080};
    int protocol_width_{1920};
    int protocol_height_{1080};
    bool draw_names_{true};
    double font_scale_factor_{0.04};

    std::map<FigureName, Figure> figures_;
    mutable std::mutex mutex_;
};

} // namespace rm_ui

#endif // RM_UI__UI_DEBUGGER_HPP_
