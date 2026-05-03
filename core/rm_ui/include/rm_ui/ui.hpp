#ifndef RM_UI__UI_HPP_
#define RM_UI__UI_HPP_

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>
#include <variant>

#include "rclcpp/rclcpp.hpp"

#include "rm_message/msg/send_message.hpp"
#include "rm_ui/srv/delete_layer.hpp"
#include "rm_ui/srv/draw_figure.hpp"
#include "rm_ui/srv/draw_shape.hpp"
#include "rm_ui/srv/delete_shape.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace rm_ui
{

class RmUi : public rclcpp::Node
{
public:
    explicit RmUi(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
    ~RmUi() override = default;

private:
    enum class Operation : uint8_t
    {
        Noop = 0,
        Add = 1,
        Modify = 2,
        Delete = 3,
    };

    enum class FigureType : uint32_t
    {
        Line = 0,
        Rect = 1,
        Circle = 2,
        Ellipse = 3,
        Arc = 4,
        Float = 5,
        Int = 6,
        String = 7,
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

    struct PendingFigure
    {
        Figure figure;
        Operation operation{Operation::Noop};
        uint32_t previous_layer{0};
        bool had_previous{false};
    };

    struct PendingDelete
    {
        uint8_t delete_type{0};
        uint8_t layer{0};

        static constexpr uint8_t DELETE_ALL_LAYER = 2;

    };

    using FigureName = std::array<uint8_t, 3>;
    using PendingPalette = std::variant<PendingFigure, PendingDelete>;


    static constexpr uint16_t kInteractionCmdId = 0x0301;
    static constexpr uint16_t kDeleteContentId = 0x0100;
    static constexpr uint16_t kSingleFigureContentId = 0x0101;
    static constexpr uint16_t kDoubleFigureContentId = 0x0102;
    static constexpr uint16_t kFiveFigureContentId = 0x0103;
    static constexpr uint16_t kSevenFigureContentId = 0x0104;
    static constexpr uint16_t kStringContentId = 0x0110;
    static constexpr uint32_t kMaxFigureType = static_cast<uint32_t>(FigureType::String);
    static constexpr uint32_t kStringFigureType = static_cast<uint32_t>(FigureType::String);
    static constexpr size_t kFigureRecordLength = 15;
    static constexpr size_t kStringLength = 30;
    static constexpr size_t kMaxBatchFigureCount = 7;

    bool validateDrawRequest(
        const rm_ui::srv::DrawFigure::Request & request,
        std::string & error_message) const;

    bool buildShapeFigure(
        const rm_ui::srv::DrawShape::Request & request,
        Figure & figure,
        std::string & error_message) const;

    void handleDrawFigure(
        const std::shared_ptr<rm_ui::srv::DrawFigure::Request> request,
        std::shared_ptr<rm_ui::srv::DrawFigure::Response> response);

    void handleDrawShape(
        const std::shared_ptr<rm_ui::srv::DrawShape::Request> request,
        std::shared_ptr<rm_ui::srv::DrawShape::Response> response);

    void handleDeleteLayer(
        const std::shared_ptr<rm_ui::srv::DeleteLayer::Request> request,
        std::shared_ptr<rm_ui::srv::DeleteLayer::Response> response);

    void handleDeleteShape(
        const std::shared_ptr<rm_ui::srv::DeleteShape::Request> request,
        std::shared_ptr<rm_ui::srv::DeleteShape::Response> response);

    void handleRedrawTrigger(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response);

    void update();
    void publishInteractionPayload(const std::vector<uint8_t> & payload);
    std::vector<uint8_t> buildInteractionPayload(
        uint16_t content_id,
        const std::vector<uint8_t> & user_data) const;
    std::vector<uint8_t> buildDeleteUserData(const PendingDelete & delete_op) const;
    std::vector<uint8_t> buildStringUserData(const PendingFigure & pending) const;
    std::vector<uint8_t> buildFigureBatchUserData(std::vector<PendingFigure> & batch) const;

    void eraseLayerFromCacheLocked(uint8_t layer);
    // void eraseLayerFromPendingFigures(uint8_t layer);
    void enqueueFigureLocked(const Figure & figure);

    static bool fitsUnsignedBits(uint32_t value, uint8_t bits);
    static bool isValidFigureType(uint32_t value);
    static FigureType toFigureType(uint32_t value);
    static std::array<uint8_t, kFigureRecordLength> packFigure(
        const Figure & figure,
        Operation operation);

    rclcpp::Publisher<rm_message::msg::SendMessage>::SharedPtr sender_pub_;
    rclcpp::Service<rm_ui::srv::DrawFigure>::SharedPtr draw_service_;
    rclcpp::Service<rm_ui::srv::DrawShape>::SharedPtr draw_shape_service_;
    rclcpp::Service<rm_ui::srv::DeleteLayer>::SharedPtr delete_service_;
    rclcpp::Service<rm_ui::srv::DeleteShape>::SharedPtr delete_shape_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr redraw_trigger_service_;
    rclcpp::TimerBase::SharedPtr update_timer_;

    std::string sender_topic_;
    uint8_t sender_target_{2};
    uint16_t sender_id_{0};
    uint16_t receiver_id_{0};

    std::map<FigureName, Figure> cached_figures_;
    std::deque<PendingPalette> pending_palettes_;
    // protect both cached_figures_ and pending_palettes_
    mutable std::mutex mutex_;

    double publish_hz_{0.0};
    std::chrono::steady_clock::time_point last_publish_time_{
        std::chrono::steady_clock::time_point::min()};
};

} // namespace rm_ui

#endif // RM_UI__UI_HPP_
