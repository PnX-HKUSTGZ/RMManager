#include <gtest/gtest.h>

#include <atomic>
#include <array>
#include <chrono>
#include <cstdint>
#include <limits>
#include <memory>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "rm_message/msg/send_message.hpp"
#include "rm_ui/srv/delete_layer.hpp"
#include "rm_ui/srv/draw_figure.hpp"
#include "rm_ui/srv/draw_shape.hpp"
#include "rm_ui/ui.hpp"

namespace
{

using namespace std::chrono_literals;

constexpr uint16_t kSenderId = 0x0102;
constexpr uint16_t kReceiverId = 0x0103;

uint16_t readLe16(const std::vector<uint8_t> & data, size_t offset)
{
    return static_cast<uint16_t>(data.at(offset)) |
           static_cast<uint16_t>(data.at(offset + 1)) << 8;
}

uint32_t readLe32(const std::vector<uint8_t> & data, size_t offset)
{
    return static_cast<uint32_t>(data.at(offset)) |
           static_cast<uint32_t>(data.at(offset + 1)) << 8 |
           static_cast<uint32_t>(data.at(offset + 2)) << 16 |
           static_cast<uint32_t>(data.at(offset + 3)) << 24;
}

rm_ui::srv::DrawFigure::Request makeFigureRequest(
    uint8_t name_suffix,
    uint32_t figure_type = 0,
    uint32_t layer = 1)
{
    rm_ui::srv::DrawFigure::Request request;
    request.figure_name = std::array<uint8_t, 3>{
        static_cast<uint8_t>('A'),
        static_cast<uint8_t>('A' + name_suffix),
        static_cast<uint8_t>('0' + name_suffix)};
    request.figure_type = figure_type;
    request.layer = layer;
    request.color = 1;
    request.details_a = 0;
    request.details_b = 0;
    request.width = 1;
    request.start_x = 100;
    request.start_y = 200;
    request.details_c = 0;
    request.details_d = 300;
    request.details_e = 400;
    return request;
}

rm_ui::srv::DrawShape::Request makeShapeRequest(
    const std::string & name,
    uint8_t figure_type,
    uint8_t layer = 1)
{
    rm_ui::srv::DrawShape::Request request;
    (void)name; // avoid unused parameter warning
    // request.figure_name = name;
    request.figure_type = figure_type;
    request.layer = layer;
    request.color = rm_ui::srv::DrawShape::Request::COLOR_YELLOW;
    request.width = 1;
    request.start_x = 100;
    request.start_y = 200;
    return request;
}

struct PackedFigure
{
    std::array<uint8_t, 3> name{};
    uint32_t operation{0};
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
    uint32_t details_dword3{0};
};

PackedFigure parsePackedFigure(
    const rm_message::msg::SendMessage & message,
    size_t record_offset = 6)
{
    PackedFigure figure;
    figure.name = {
        message.data_payload.at(record_offset),
        message.data_payload.at(record_offset + 1),
        message.data_payload.at(record_offset + 2)};

    const uint32_t dword1 = readLe32(message.data_payload, record_offset + 3);
    figure.operation = dword1 & 0x7u;
    figure.figure_type = (dword1 >> 3) & 0x7u;
    figure.layer = (dword1 >> 6) & 0xfu;
    figure.color = (dword1 >> 10) & 0xfu;
    figure.details_a = (dword1 >> 14) & 0x1ffu;
    figure.details_b = (dword1 >> 23) & 0x1ffu;

    const uint32_t dword2 = readLe32(message.data_payload, record_offset + 7);
    figure.width = dword2 & 0x3ffu;
    figure.start_x = (dword2 >> 10) & 0x7ffu;
    figure.start_y = (dword2 >> 21) & 0x7ffu;

    figure.details_dword3 = readLe32(message.data_payload, record_offset + 11);
    figure.details_c = figure.details_dword3 & 0x3ffu;
    figure.details_d = (figure.details_dword3 >> 10) & 0x7ffu;
    figure.details_e = (figure.details_dword3 >> 21) & 0x7ffu;
    return figure;
}

class RmUiHarness
{
public:
    explicit RmUiHarness(double sender_hz = 1000.0)
    {
        const int id = counter_.fetch_add(1);
        const std::string suffix = std::to_string(id);
        sender_topic_ = "/rm_ui_test_" + suffix + "/send_message";
        draw_service_ = "/rm_ui_test_" + suffix + "/draw_figure";
        draw_shape_service_ = "/rm_ui_test_" + suffix + "/draw_shape";
        delete_service_ = "/rm_ui_test_" + suffix + "/delete_layer";

        rclcpp::NodeOptions options;
        options.parameter_overrides({
                rclcpp::Parameter("sender_topic", sender_topic_),
                rclcpp::Parameter("sender_hz", sender_hz),
                rclcpp::Parameter("sender_target", 2),
                rclcpp::Parameter("sender_id", static_cast<int>(kSenderId)),
                rclcpp::Parameter("receiver_id", static_cast<int>(kReceiverId)),
        });
        options.arguments({
                "--ros-args",
                "-r", "__node:=rm_ui_test_" + suffix,
                "-r", "draw_figure:=" + draw_service_,
                "-r", "draw_shape:=" + draw_shape_service_,
                "-r", "delete_layer:=" + delete_service_,
        });

        ui_node_ = std::make_shared<rm_ui::RmUi>(options);
        helper_node_ = std::make_shared<rclcpp::Node>("rm_ui_test_helper_" + suffix);

        sender_sub_ = helper_node_->create_subscription<rm_message::msg::SendMessage>(
            sender_topic_,
            10,
            [this](const rm_message::msg::SendMessage::SharedPtr message) {
                received_messages_.push_back(*message);
            });
        draw_client_ = helper_node_->create_client<rm_ui::srv::DrawFigure>(draw_service_);
        draw_shape_client_ =
          helper_node_->create_client<rm_ui::srv::DrawShape>(draw_shape_service_);
        delete_client_ = helper_node_->create_client<rm_ui::srv::DeleteLayer>(delete_service_);

        executor_.add_node(ui_node_);
        executor_.add_node(helper_node_);
        spinFor(100ms);
    }

    ~RmUiHarness()
    {
        executor_.remove_node(helper_node_);
        executor_.remove_node(ui_node_);
    }

    rm_ui::srv::DrawFigure::Response::SharedPtr draw(
        const rm_ui::srv::DrawFigure::Request & request)
    {
        if (!draw_client_->wait_for_service(1s)) {
            return nullptr;
        }
        auto request_ptr = std::make_shared<rm_ui::srv::DrawFigure::Request>(request);
        auto future = draw_client_->async_send_request(request_ptr);
        if (executor_.spin_until_future_complete(future, 1s) !=
          rclcpp::FutureReturnCode::SUCCESS)
        {
            return nullptr;
        }
        return future.get();
    }

    rm_ui::srv::DrawShape::Response::SharedPtr drawShape(
        const rm_ui::srv::DrawShape::Request & request)
    {
        if (!draw_shape_client_->wait_for_service(1s)) {
            return nullptr;
        }
        auto request_ptr = std::make_shared<rm_ui::srv::DrawShape::Request>(request);
        auto future = draw_shape_client_->async_send_request(request_ptr);
        if (executor_.spin_until_future_complete(future, 1s) !=
          rclcpp::FutureReturnCode::SUCCESS)
        {
            return nullptr;
        }
        return future.get();
    }

    rm_ui::srv::DeleteLayer::Response::SharedPtr deleteLayer(int8_t layer)
    {
        if (!delete_client_->wait_for_service(1s)) {
            return nullptr;
        }
        auto request = std::make_shared<rm_ui::srv::DeleteLayer::Request>();
        request->layer = layer;
        auto future = delete_client_->async_send_request(request);
        if (executor_.spin_until_future_complete(future, 1s) !=
          rclcpp::FutureReturnCode::SUCCESS)
        {
            return nullptr;
        }
        return future.get();
    }

    bool waitForMessages(size_t count, std::chrono::milliseconds timeout)
    {
        const auto deadline = std::chrono::steady_clock::now() + timeout;
        while (std::chrono::steady_clock::now() < deadline) {
            executor_.spin_some();
            if (received_messages_.size() >= count) {
                return true;
            }
            std::this_thread::sleep_for(1ms);
        }
        return received_messages_.size() >= count;
    }

    void spinFor(std::chrono::milliseconds duration)
    {
        const auto deadline = std::chrono::steady_clock::now() + duration;
        while (std::chrono::steady_clock::now() < deadline) {
            executor_.spin_some();
            std::this_thread::sleep_for(1ms);
        }
    }

    const std::vector<rm_message::msg::SendMessage> & messages() const
    {
        return received_messages_;
    }

private:
    static std::atomic<int> counter_;

    std::string sender_topic_;
    std::string draw_service_;
    std::string draw_shape_service_;
    std::string delete_service_;
    rclcpp::executors::SingleThreadedExecutor executor_;
    std::shared_ptr<rm_ui::RmUi> ui_node_;
    rclcpp::Node::SharedPtr helper_node_;
    rclcpp::Subscription<rm_message::msg::SendMessage>::SharedPtr sender_sub_;
    rclcpp::Client<rm_ui::srv::DrawFigure>::SharedPtr draw_client_;
    rclcpp::Client<rm_ui::srv::DrawShape>::SharedPtr draw_shape_client_;
    rclcpp::Client<rm_ui::srv::DeleteLayer>::SharedPtr delete_client_;
    std::vector<rm_message::msg::SendMessage> received_messages_;
};

std::atomic<int> RmUiHarness::counter_{0};

void expectCommonSendMessage(const rm_message::msg::SendMessage & message, size_t payload_size)
{
    EXPECT_EQ(message.target, 2);
    EXPECT_EQ(message.cmd_id, 0x0301);
    EXPECT_EQ(message.data_length, payload_size);
    ASSERT_EQ(message.data_payload.size(), payload_size);
    EXPECT_EQ(readLe16(message.data_payload, 2), kSenderId);
    EXPECT_EQ(readLe16(message.data_payload, 4), kReceiverId);
}

void runBatchCase(size_t figure_count, uint16_t expected_content_id, size_t expected_slots)
{
    RmUiHarness harness(0.5);
    for (size_t i = 0; i < figure_count; ++i) {
        auto request = makeFigureRequest(static_cast<uint8_t>(i));
        const auto response = harness.draw(request);
        ASSERT_NE(response, nullptr);
        ASSERT_TRUE(response->success) << response->message;
    }

    ASSERT_TRUE(harness.waitForMessages(1, 2500ms));
    const auto & message = harness.messages().front();
    expectCommonSendMessage(message, 6 + expected_slots * 15);
    EXPECT_EQ(readLe16(message.data_payload, 0), expected_content_id);

    for (size_t i = figure_count; i < expected_slots; ++i) {
        const size_t record_offset = 6 + i * 15;
        const uint32_t dword1 = readLe32(message.data_payload, record_offset + 3);
        EXPECT_EQ(dword1 & 0x7u, 0u);
    }
}

} // namespace

TEST(RmUi, PacksSingleFigureFields)
{
    RmUiHarness harness;
    auto request = makeFigureRequest(1, 0, 2);
    request.color = 3;
    request.details_a = 4;
    request.details_b = 5;
    request.width = 6;
    request.start_x = 7;
    request.start_y = 8;
    request.details_c = 9;
    request.details_d = 10;
    request.details_e = 11;

    const auto response = harness.draw(request);
    ASSERT_NE(response, nullptr);
    ASSERT_TRUE(response->success) << response->message;

    ASSERT_TRUE(harness.waitForMessages(1, 200ms));
    const auto & message = harness.messages().front();
    expectCommonSendMessage(message, 21);
    EXPECT_EQ(readLe16(message.data_payload, 0), 0x0101);

    const size_t record_offset = 6;
    EXPECT_EQ(message.data_payload.at(record_offset + 0), 'A');
    EXPECT_EQ(message.data_payload.at(record_offset + 1), 'B');
    EXPECT_EQ(message.data_payload.at(record_offset + 2), '1');

    const uint32_t dword1 = readLe32(message.data_payload, record_offset + 3);
    EXPECT_EQ(dword1 & 0x7u, 1u);
    EXPECT_EQ((dword1 >> 3) & 0x7u, 0u);
    EXPECT_EQ((dword1 >> 6) & 0xfu, 2u);
    EXPECT_EQ((dword1 >> 10) & 0xfu, 3u);
    EXPECT_EQ((dword1 >> 14) & 0x1ffu, 4u);
    EXPECT_EQ((dword1 >> 23) & 0x1ffu, 5u);

    const uint32_t dword2 = readLe32(message.data_payload, record_offset + 7);
    EXPECT_EQ(dword2 & 0x3ffu, 6u);
    EXPECT_EQ((dword2 >> 10) & 0x7ffu, 7u);
    EXPECT_EQ((dword2 >> 21) & 0x7ffu, 8u);

    const uint32_t dword3 = readLe32(message.data_payload, record_offset + 11);
    EXPECT_EQ(dword3 & 0x3ffu, 9u);
    EXPECT_EQ((dword3 >> 10) & 0x7ffu, 10u);
    EXPECT_EQ((dword3 >> 21) & 0x7ffu, 11u);
}

TEST(RmUi, PublishesDeletePayloads)
{
    RmUiHarness harness;

    const auto delete_all = harness.deleteLayer(-1);
    ASSERT_NE(delete_all, nullptr);
    ASSERT_TRUE(delete_all->success) << delete_all->message;
    ASSERT_TRUE(harness.waitForMessages(1, 200ms));
    const auto & all_message = harness.messages().at(0);
    expectCommonSendMessage(all_message, 8);
    EXPECT_EQ(readLe16(all_message.data_payload, 0), 0x0100);
    EXPECT_EQ(all_message.data_payload.at(6), 2u);
    EXPECT_EQ(all_message.data_payload.at(7), 0u);

    const auto delete_layer = harness.deleteLayer(3);
    ASSERT_NE(delete_layer, nullptr);
    ASSERT_TRUE(delete_layer->success) << delete_layer->message;
    ASSERT_TRUE(harness.waitForMessages(2, 200ms));
    const auto & layer_message = harness.messages().at(1);
    expectCommonSendMessage(layer_message, 8);
    EXPECT_EQ(readLe16(layer_message.data_payload, 0), 0x0100);
    EXPECT_EQ(layer_message.data_payload.at(6), 1u);
    EXPECT_EQ(layer_message.data_payload.at(7), 3u);
}

TEST(RmUi, BatchesNonStringFiguresWithProtocolSlotCounts)
{
    runBatchCase(1, 0x0101, 1);
    runBatchCase(2, 0x0102, 2);
    runBatchCase(3, 0x0103, 5);
    runBatchCase(5, 0x0103, 5);
    runBatchCase(7, 0x0104, 7);
}

TEST(RmUi, PublishesStringFigurePayload)
{
    RmUiHarness harness;
    auto request = makeFigureRequest(2, 7, 4);
    request.color = 8;
    request.details_a = 16;
    request.details_b = 2;
    request.details_c = 0;
    request.details_d = 0;
    request.details_e = 0;
    request.chars = {'h', 'i'};

    const auto response = harness.draw(request);
    ASSERT_NE(response, nullptr);
    ASSERT_TRUE(response->success) << response->message;

    ASSERT_TRUE(harness.waitForMessages(1, 200ms));
    const auto & message = harness.messages().front();
    expectCommonSendMessage(message, 51);
    EXPECT_EQ(readLe16(message.data_payload, 0), 0x0110);

    const size_t record_offset = 6;
    const uint32_t dword1 = readLe32(message.data_payload, record_offset + 3);
    EXPECT_EQ(dword1 & 0x7u, 1u);
    EXPECT_EQ((dword1 >> 3) & 0x7u, 7u);
    EXPECT_EQ((dword1 >> 6) & 0xfu, 4u);
    EXPECT_EQ((dword1 >> 10) & 0xfu, 8u);
    EXPECT_EQ((dword1 >> 14) & 0x1ffu, 16u);
    EXPECT_EQ((dword1 >> 23) & 0x1ffu, 2u);

    const size_t chars_offset = 21;
    EXPECT_EQ(message.data_payload.at(chars_offset), 'h');
    EXPECT_EQ(message.data_payload.at(chars_offset + 1), 'i');
    for (size_t i = chars_offset + 2; i < message.data_payload.size(); ++i) {
        EXPECT_EQ(message.data_payload.at(i), 0u);
    }
}

TEST(RmUi, DrawShapeMapsLineAndModifyByName)
{
    RmUiHarness harness;
    auto request = makeShapeRequest("LN1", rm_ui::srv::DrawShape::Request::TYPE_LINE, 2);
    request.color = rm_ui::srv::DrawShape::Request::COLOR_ORANGE;
    request.width = 6;
    request.start_x = 7;
    request.start_y = 8;
    request.end_x = 10;
    request.end_y = 11;

    const auto add_response = harness.drawShape(request);
    ASSERT_NE(add_response, nullptr);
    ASSERT_TRUE(add_response->success) << add_response->message;
    ASSERT_TRUE(harness.waitForMessages(1, 200ms));

    const auto add_message = harness.messages().at(0);
    expectCommonSendMessage(add_message, 21);
    EXPECT_EQ(readLe16(add_message.data_payload, 0), 0x0101);
    auto add_figure = parsePackedFigure(add_message);
    EXPECT_EQ(add_figure.name, (std::array<uint8_t, 3>{'L', 'N', '1'}));
    EXPECT_EQ(add_figure.operation, 1u);
    EXPECT_EQ(add_figure.figure_type, 0u);
    EXPECT_EQ(add_figure.layer, 2u);
    EXPECT_EQ(add_figure.color, 3u);
    EXPECT_EQ(add_figure.width, 6u);
    EXPECT_EQ(add_figure.start_x, 7u);
    EXPECT_EQ(add_figure.start_y, 8u);
    EXPECT_EQ(add_figure.details_c, 0u);
    EXPECT_EQ(add_figure.details_d, 10u);
    EXPECT_EQ(add_figure.details_e, 11u);

    request.end_x = 12;
    request.end_y = 13;
    const auto modify_response = harness.drawShape(request);
    ASSERT_NE(modify_response, nullptr);
    ASSERT_TRUE(modify_response->success) << modify_response->message;
    ASSERT_TRUE(harness.waitForMessages(2, 200ms));

    const auto modify_message = harness.messages().at(1);
    expectCommonSendMessage(modify_message, 21);
    auto modify_figure = parsePackedFigure(modify_message);
    EXPECT_EQ(modify_figure.operation, 2u);
    EXPECT_EQ(modify_figure.figure_type, 0u);
    EXPECT_EQ(modify_figure.details_d, 12u);
    EXPECT_EQ(modify_figure.details_e, 13u);
}

TEST(RmUi, DrawShapeMapsGeometricTypes)
{
    RmUiHarness harness(0.5);

    auto rect = makeShapeRequest("RC1", rm_ui::srv::DrawShape::Request::TYPE_RECT);
    rect.width = 2;
    rect.start_x = 11;
    rect.start_y = 12;
    rect.end_x = 111;
    rect.end_y = 112;
    ASSERT_TRUE(harness.drawShape(rect)->success);

    auto circle = makeShapeRequest("CC1", rm_ui::srv::DrawShape::Request::TYPE_CIRCLE);
    circle.width = 3;
    circle.start_x = 21;
    circle.start_y = 22;
    circle.radius = 23;
    ASSERT_TRUE(harness.drawShape(circle)->success);

    auto ellipse = makeShapeRequest("EL1", rm_ui::srv::DrawShape::Request::TYPE_ELLIPSE);
    ellipse.width = 4;
    ellipse.start_x = 31;
    ellipse.start_y = 32;
    ellipse.x_semiaxis = 33;
    ellipse.y_semiaxis = 34;
    ASSERT_TRUE(harness.drawShape(ellipse)->success);

    auto arc = makeShapeRequest("AR1", rm_ui::srv::DrawShape::Request::TYPE_ARC);
    arc.width = 5;
    arc.start_x = 41;
    arc.start_y = 42;
    arc.x_semiaxis = 43;
    arc.y_semiaxis = 44;
    arc.start_angle = 45;
    arc.end_angle = 90;
    ASSERT_TRUE(harness.drawShape(arc)->success);

    ASSERT_TRUE(harness.waitForMessages(1, 2500ms));
    const auto & message = harness.messages().front();
    expectCommonSendMessage(message, 81);
    EXPECT_EQ(readLe16(message.data_payload, 0), 0x0103);

    auto packed_rect = parsePackedFigure(message, 6);
    EXPECT_EQ(packed_rect.name, (std::array<uint8_t, 3>{'R', 'C', '1'}));
    EXPECT_EQ(packed_rect.operation, 1u);
    EXPECT_EQ(packed_rect.figure_type, 1u);
    EXPECT_EQ(packed_rect.width, 2u);
    EXPECT_EQ(packed_rect.start_x, 11u);
    EXPECT_EQ(packed_rect.start_y, 12u);
    EXPECT_EQ(packed_rect.details_d, 111u);
    EXPECT_EQ(packed_rect.details_e, 112u);

    auto packed_circle = parsePackedFigure(message, 21);
    EXPECT_EQ(packed_circle.name, (std::array<uint8_t, 3>{'C', 'C', '1'}));
    EXPECT_EQ(packed_circle.operation, 1u);
    EXPECT_EQ(packed_circle.figure_type, 2u);
    EXPECT_EQ(packed_circle.width, 3u);
    EXPECT_EQ(packed_circle.start_x, 21u);
    EXPECT_EQ(packed_circle.start_y, 22u);
    EXPECT_EQ(packed_circle.details_c, 23u);
    EXPECT_EQ(packed_circle.details_d, 0u);
    EXPECT_EQ(packed_circle.details_e, 0u);

    auto packed_ellipse = parsePackedFigure(message, 36);
    EXPECT_EQ(packed_ellipse.name, (std::array<uint8_t, 3>{'E', 'L', '1'}));
    EXPECT_EQ(packed_ellipse.operation, 1u);
    EXPECT_EQ(packed_ellipse.figure_type, 3u);
    EXPECT_EQ(packed_ellipse.width, 4u);
    EXPECT_EQ(packed_ellipse.start_x, 31u);
    EXPECT_EQ(packed_ellipse.start_y, 32u);
    EXPECT_EQ(packed_ellipse.details_c, 0u);
    EXPECT_EQ(packed_ellipse.details_d, 33u);
    EXPECT_EQ(packed_ellipse.details_e, 34u);

    auto packed_arc = parsePackedFigure(message, 51);
    EXPECT_EQ(packed_arc.name, (std::array<uint8_t, 3>{'A', 'R', '1'}));
    EXPECT_EQ(packed_arc.operation, 1u);
    EXPECT_EQ(packed_arc.figure_type, 4u);
    EXPECT_EQ(packed_arc.details_a, 45u);
    EXPECT_EQ(packed_arc.details_b, 90u);
    EXPECT_EQ(packed_arc.width, 5u);
    EXPECT_EQ(packed_arc.start_x, 41u);
    EXPECT_EQ(packed_arc.start_y, 42u);
    EXPECT_EQ(packed_arc.details_c, 0u);
    EXPECT_EQ(packed_arc.details_d, 43u);
    EXPECT_EQ(packed_arc.details_e, 44u);

    auto noop = parsePackedFigure(message, 66);
    EXPECT_EQ(noop.operation, 0u);
}

TEST(RmUi, DrawShapeMapsNumberTypes)
{
    RmUiHarness harness(0.5);

    auto float_request = makeShapeRequest("FL1", rm_ui::srv::DrawShape::Request::TYPE_FLOAT);
    float_request.width = 2;
    float_request.start_x = 1000;
    float_request.start_y = 1001;
    float_request.font_size = 20;
    float_request.float_value = 12.5;
    ASSERT_TRUE(harness.drawShape(float_request)->success);

    auto int_request = makeShapeRequest("IN1", rm_ui::srv::DrawShape::Request::TYPE_INT);
    int_request.width = 3;
    int_request.start_x = 1002;
    int_request.start_y = 1003;
    int_request.font_size = 21;
    int_request.int_value = -123456;
    ASSERT_TRUE(harness.drawShape(int_request)->success);

    ASSERT_TRUE(harness.waitForMessages(1, 2500ms));
    const auto & message = harness.messages().front();
    expectCommonSendMessage(message, 36);
    EXPECT_EQ(readLe16(message.data_payload, 0), 0x0102);

    auto packed_float = parsePackedFigure(message, 6);
    EXPECT_EQ(packed_float.name, (std::array<uint8_t, 3>{'F', 'L', '1'}));
    EXPECT_EQ(packed_float.operation, 1u);
    EXPECT_EQ(packed_float.figure_type, 5u);
    EXPECT_EQ(packed_float.details_a, 20u);
    EXPECT_EQ(packed_float.details_b, 0u);
    EXPECT_EQ(packed_float.width, 2u);
    EXPECT_EQ(packed_float.start_x, 1000u);
    EXPECT_EQ(packed_float.start_y, 1001u);
    EXPECT_EQ(packed_float.details_dword3, 12500u);

    auto packed_int = parsePackedFigure(message, 21);
    EXPECT_EQ(packed_int.name, (std::array<uint8_t, 3>{'I', 'N', '1'}));
    EXPECT_EQ(packed_int.operation, 1u);
    EXPECT_EQ(packed_int.figure_type, 6u);
    EXPECT_EQ(packed_int.details_a, 21u);
    EXPECT_EQ(packed_int.details_b, 0u);
    EXPECT_EQ(packed_int.width, 3u);
    EXPECT_EQ(packed_int.start_x, 1002u);
    EXPECT_EQ(packed_int.start_y, 1003u);
    EXPECT_EQ(packed_int.details_dword3, static_cast<uint32_t>(-123456));
}

TEST(RmUi, DrawShapeMapsStringType)
{
    RmUiHarness harness;
    auto request = makeShapeRequest("S1", rm_ui::srv::DrawShape::Request::TYPE_STRING, 4);
    request.color = rm_ui::srv::DrawShape::Request::COLOR_WHITE;
    request.width = 4;
    request.start_x = 300;
    request.start_y = 301;
    request.font_size = 18;
    request.text = "hi";

    const auto response = harness.drawShape(request);
    ASSERT_NE(response, nullptr);
    ASSERT_TRUE(response->success) << response->message;
    ASSERT_TRUE(harness.waitForMessages(1, 200ms));

    const auto & message = harness.messages().front();
    expectCommonSendMessage(message, 51);
    EXPECT_EQ(readLe16(message.data_payload, 0), 0x0110);

    auto packed = parsePackedFigure(message);
    EXPECT_EQ(packed.name, (std::array<uint8_t, 3>{'S', '1', 0}));
    EXPECT_EQ(packed.operation, 1u);
    EXPECT_EQ(packed.figure_type, 7u);
    EXPECT_EQ(packed.layer, 4u);
    EXPECT_EQ(packed.color, 8u);
    EXPECT_EQ(packed.details_a, 18u);
    EXPECT_EQ(packed.details_b, 2u);
    EXPECT_EQ(packed.width, 4u);
    EXPECT_EQ(packed.start_x, 300u);
    EXPECT_EQ(packed.start_y, 301u);
    EXPECT_EQ(packed.details_dword3, 0u);

    const size_t chars_offset = 21;
    EXPECT_EQ(message.data_payload.at(chars_offset), 'h');
    EXPECT_EQ(message.data_payload.at(chars_offset + 1), 'i');
    for (size_t i = chars_offset + 2; i < message.data_payload.size(); ++i) {
        EXPECT_EQ(message.data_payload.at(i), 0u);
    }
}

TEST(RmUi, DrawShapeRejectsTypeChangeUntilDeleted)
{
    RmUiHarness harness;
    auto line = makeShapeRequest("A1", rm_ui::srv::DrawShape::Request::TYPE_LINE);
    line.end_x = 300;
    line.end_y = 400;
    ASSERT_TRUE(harness.drawShape(line)->success);
    ASSERT_TRUE(harness.waitForMessages(1, 200ms));

    auto circle = makeShapeRequest("A1", rm_ui::srv::DrawShape::Request::TYPE_CIRCLE);
    circle.radius = 50;
    const auto rejected = harness.drawShape(circle);
    ASSERT_NE(rejected, nullptr);
    EXPECT_FALSE(rejected->success);
    harness.spinFor(100ms);
    EXPECT_EQ(harness.messages().size(), 1u);

    const auto delete_all = harness.deleteLayer(-1);
    ASSERT_NE(delete_all, nullptr);
    ASSERT_TRUE(delete_all->success) << delete_all->message;
    ASSERT_TRUE(harness.waitForMessages(2, 200ms));

    const auto accepted_after_delete = harness.drawShape(circle);
    ASSERT_NE(accepted_after_delete, nullptr);
    ASSERT_TRUE(accepted_after_delete->success) << accepted_after_delete->message;
    ASSERT_TRUE(harness.waitForMessages(3, 200ms));

    auto packed_circle = parsePackedFigure(harness.messages().at(2));
    EXPECT_EQ(packed_circle.operation, 1u);
    EXPECT_EQ(packed_circle.figure_type, 2u);
    EXPECT_EQ(packed_circle.details_c, 50u);
}

TEST(RmUi, DrawShapeLayerMoveBeforeDeletePublishesAdd)
{
    RmUiHarness harness(2.0);
    auto request = makeShapeRequest("MV1", rm_ui::srv::DrawShape::Request::TYPE_LINE, 1);
    request.end_x = 300;
    request.end_y = 400;

    const auto add_response = harness.drawShape(request);
    ASSERT_NE(add_response, nullptr);
    ASSERT_TRUE(add_response->success) << add_response->message;
    ASSERT_TRUE(harness.waitForMessages(1, 1000ms));
    EXPECT_EQ(parsePackedFigure(harness.messages().at(0)).operation, 1u);

    request.layer = 2;
    request.end_x = 500;
    request.end_y = 600;
    const auto move_response = harness.drawShape(request);
    ASSERT_NE(move_response, nullptr);
    ASSERT_TRUE(move_response->success) << move_response->message;

    const auto delete_old_layer = harness.deleteLayer(1);
    ASSERT_NE(delete_old_layer, nullptr);
    ASSERT_TRUE(delete_old_layer->success) << delete_old_layer->message;

    ASSERT_TRUE(harness.waitForMessages(3, 2500ms));
    const auto & delete_message = harness.messages().at(1);
    expectCommonSendMessage(delete_message, 8);
    EXPECT_EQ(readLe16(delete_message.data_payload, 0), 0x0100);
    EXPECT_EQ(delete_message.data_payload.at(6), 1u);
    EXPECT_EQ(delete_message.data_payload.at(7), 1u);

    const auto & moved_message = harness.messages().at(2);
    expectCommonSendMessage(moved_message, 21);
    auto moved_figure = parsePackedFigure(moved_message);
    EXPECT_EQ(moved_figure.operation, 1u);
    EXPECT_EQ(moved_figure.figure_type, 0u);
    EXPECT_EQ(moved_figure.layer, 2u);
    EXPECT_EQ(moved_figure.details_d, 500u);
    EXPECT_EQ(moved_figure.details_e, 600u);
}

TEST(RmUi, DrawFigureRejectsTypeChangeForDrawShapeNames)
{
    RmUiHarness harness;
    auto shape = makeShapeRequest("RAW", rm_ui::srv::DrawShape::Request::TYPE_LINE);
    shape.end_x = 300;
    shape.end_y = 400;

    const auto shape_response = harness.drawShape(shape);
    ASSERT_NE(shape_response, nullptr);
    ASSERT_TRUE(shape_response->success) << shape_response->message;
    ASSERT_TRUE(harness.waitForMessages(1, 200ms));

    auto raw = makeFigureRequest(0, rm_ui::srv::DrawShape::Request::TYPE_CIRCLE);
    raw.figure_name = {'R', 'A', 'W'};
    raw.details_c = 50;
    raw.details_d = 0;
    raw.details_e = 0;
    const auto raw_response = harness.draw(raw);
    ASSERT_NE(raw_response, nullptr);
    EXPECT_FALSE(raw_response->success);

    harness.spinFor(100ms);
    EXPECT_EQ(harness.messages().size(), 1u);
}

TEST(RmUi, RejectsInvalidRequestsWithoutPublishing)
{
    RmUiHarness harness;
    std::vector<rm_ui::srv::DrawFigure::Request> invalid_requests;

    auto invalid_type = makeFigureRequest(0);
    invalid_type.figure_type = 8;
    invalid_requests.push_back(invalid_type);

    auto invalid_layer = makeFigureRequest(1);
    invalid_layer.layer = 10;
    invalid_requests.push_back(invalid_layer);

    auto invalid_color = makeFigureRequest(2);
    invalid_color.color = 9;
    invalid_requests.push_back(invalid_color);

    auto invalid_width = makeFigureRequest(3);
    invalid_width.width = 1024;
    invalid_requests.push_back(invalid_width);

    auto invalid_x = makeFigureRequest(4);
    invalid_x.start_x = 2048;
    invalid_requests.push_back(invalid_x);

    auto invalid_string = makeFigureRequest(5, 7);
    invalid_string.details_b = 31;
    invalid_string.chars.assign(31, static_cast<uint8_t>('x'));
    invalid_requests.push_back(invalid_string);

    for (const auto & request : invalid_requests) {
        const auto response = harness.draw(request);
        ASSERT_NE(response, nullptr);
        EXPECT_FALSE(response->success);
    }

    const auto bad_delete = harness.deleteLayer(10);
    ASSERT_NE(bad_delete, nullptr);
    EXPECT_FALSE(bad_delete->success);

    harness.spinFor(100ms);
    EXPECT_TRUE(harness.messages().empty());
}

TEST(RmUi, DrawShapeRejectsInvalidRequestsWithoutPublishing)
{
    RmUiHarness harness;
    std::vector<rm_ui::srv::DrawShape::Request> invalid_requests;

    auto empty_name = makeShapeRequest("", rm_ui::srv::DrawShape::Request::TYPE_LINE);
    empty_name.end_x = 300;
    empty_name.end_y = 400;
    invalid_requests.push_back(empty_name);

    auto long_name = makeShapeRequest("ABCD", rm_ui::srv::DrawShape::Request::TYPE_LINE);
    long_name.end_x = 300;
    long_name.end_y = 400;
    invalid_requests.push_back(long_name);

    auto bad_name = makeShapeRequest("A\n", rm_ui::srv::DrawShape::Request::TYPE_LINE);
    bad_name.end_x = 300;
    bad_name.end_y = 400;
    invalid_requests.push_back(bad_name);

    auto invalid_type = makeShapeRequest("T1", rm_ui::srv::DrawShape::Request::TYPE_LINE);
    invalid_type.figure_type = 8;
    invalid_requests.push_back(invalid_type);

    auto invalid_layer = makeShapeRequest("L1", rm_ui::srv::DrawShape::Request::TYPE_LINE);
    invalid_layer.layer = 10;
    invalid_layer.end_x = 300;
    invalid_layer.end_y = 400;
    invalid_requests.push_back(invalid_layer);

    auto invalid_color = makeShapeRequest("C1", rm_ui::srv::DrawShape::Request::TYPE_LINE);
    invalid_color.color = 9;
    invalid_color.end_x = 300;
    invalid_color.end_y = 400;
    invalid_requests.push_back(invalid_color);

    auto invalid_width = makeShapeRequest("W1", rm_ui::srv::DrawShape::Request::TYPE_LINE);
    invalid_width.width = 1024;
    invalid_width.end_x = 300;
    invalid_width.end_y = 400;
    invalid_requests.push_back(invalid_width);

    auto invalid_x = makeShapeRequest("X1", rm_ui::srv::DrawShape::Request::TYPE_LINE);
    invalid_x.start_x = 2048;
    invalid_x.end_x = 300;
    invalid_x.end_y = 400;
    invalid_requests.push_back(invalid_x);

    auto invalid_line_unused = makeShapeRequest("U1", rm_ui::srv::DrawShape::Request::TYPE_LINE);
    invalid_line_unused.end_x = 300;
    invalid_line_unused.end_y = 400;
    invalid_line_unused.radius = 1;
    invalid_requests.push_back(invalid_line_unused);

    auto invalid_radius = makeShapeRequest("R1", rm_ui::srv::DrawShape::Request::TYPE_CIRCLE);
    invalid_radius.radius = 1024;
    invalid_requests.push_back(invalid_radius);

    auto invalid_angle = makeShapeRequest("A1", rm_ui::srv::DrawShape::Request::TYPE_ARC);
    invalid_angle.x_semiaxis = 20;
    invalid_angle.y_semiaxis = 30;
    invalid_angle.start_angle = 361;
    invalid_requests.push_back(invalid_angle);

    auto invalid_font = makeShapeRequest("F1", rm_ui::srv::DrawShape::Request::TYPE_INT);
    invalid_font.font_size = 512;
    invalid_requests.push_back(invalid_font);

    auto negative_float = makeShapeRequest("F2", rm_ui::srv::DrawShape::Request::TYPE_FLOAT);
    negative_float.font_size = 10;
    negative_float.float_value = -1.0;
    invalid_requests.push_back(negative_float);

    auto infinite_float = makeShapeRequest("F3", rm_ui::srv::DrawShape::Request::TYPE_FLOAT);
    infinite_float.font_size = 10;
    infinite_float.float_value = std::numeric_limits<double>::infinity();
    invalid_requests.push_back(infinite_float);

    auto oversized_float = makeShapeRequest("F4", rm_ui::srv::DrawShape::Request::TYPE_FLOAT);
    oversized_float.font_size = 10;
    oversized_float.float_value = 4294968.0;
    invalid_requests.push_back(oversized_float);

    auto long_text = makeShapeRequest("S1", rm_ui::srv::DrawShape::Request::TYPE_STRING);
    long_text.font_size = 10;
    long_text.text.assign(31, 'x');
    invalid_requests.push_back(long_text);

    auto bad_text = makeShapeRequest("S2", rm_ui::srv::DrawShape::Request::TYPE_STRING);
    bad_text.font_size = 10;
    bad_text.text = std::string(1, static_cast<char>(0x80));
    invalid_requests.push_back(bad_text);

    for (const auto & request : invalid_requests) {
        const auto response = harness.drawShape(request);
        ASSERT_NE(response, nullptr);
        EXPECT_FALSE(response->success);
    }

    harness.spinFor(100ms);
    EXPECT_TRUE(harness.messages().empty());
}

int main(int argc, char ** argv)
{
    testing::InitGoogleTest(&argc, argv);
    rclcpp::init(argc, argv);
    const int result = RUN_ALL_TESTS();
    rclcpp::shutdown();
    return result;
}
