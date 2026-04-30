#include <gtest/gtest.h>

#include <atomic>
#include <array>
#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

#include "rm_message/msg/send_message.hpp"
#include "rm_ui/ui_debugger.hpp"

namespace
{

using namespace std::chrono_literals;

struct TestFigure
{
    std::array<uint8_t, 3> name{
        static_cast<uint8_t>('F'),
        static_cast<uint8_t>('G'),
        static_cast<uint8_t>('0')};
    uint32_t figure_type{0};
    uint32_t layer{1};
    uint32_t color{1};
    uint32_t details_a{0};
    uint32_t details_b{0};
    uint32_t width{8};
    uint32_t start_x{100};
    uint32_t start_y{100};
    uint32_t details_c{0};
    uint32_t details_d{700};
    uint32_t details_e{700};
};

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

void appendCommonHeader(std::vector<uint8_t> & payload, uint16_t content_id)
{
    appendUint16(payload, content_id);
    appendUint16(payload, 0x0102);
    appendUint16(payload, 0x0103);
}

void appendFigureRecord(
    std::vector<uint8_t> & payload, const TestFigure & figure,
    uint8_t operation)
{
    payload.push_back(figure.name[0]);
    payload.push_back(figure.name[1]);
    payload.push_back(figure.name[2]);

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

    appendUint32(payload, dword1);
    appendUint32(payload, dword2);
    appendUint32(payload, dword3);
}

rm_message::msg::SendMessage makeMessage(const std::vector<uint8_t> & payload)
{
    rm_message::msg::SendMessage message;
    message.target = 2;
    message.cmd_id = 0x0301;
    message.data_length = static_cast<uint16_t>(payload.size());
    message.data_payload = payload;
    return message;
}

rm_message::msg::SendMessage makeFigureBatchMessage(
    uint16_t content_id,
    size_t slot_count,
    size_t real_count)
{
    std::vector<uint8_t> payload;
    appendCommonHeader(payload, content_id);
    for (size_t i = 0; i < slot_count; ++i) {
        if (i < real_count) {
            TestFigure figure;
            figure.name = {
                static_cast<uint8_t>('B'),
                static_cast<uint8_t>('A' + i),
                static_cast<uint8_t>('0' + i)};
            figure.layer = static_cast<uint32_t>(i % 10);
            figure.color = static_cast<uint32_t>((i % 8) + 1);
            figure.start_x = static_cast<uint32_t>(100 + i * 120);
            figure.start_y = static_cast<uint32_t>(120 + i * 50);
            figure.details_d = static_cast<uint32_t>(400 + i * 120);
            figure.details_e = static_cast<uint32_t>(420 + i * 40);
            appendFigureRecord(payload, figure, 1);
        } else {
            appendFigureRecord(payload, TestFigure{}, 0);
        }
    }
    return makeMessage(payload);
}

rm_message::msg::SendMessage makeLineMessage(uint8_t layer = 1)
{
    TestFigure figure;
    figure.name = {
        static_cast<uint8_t>('L'),
        static_cast<uint8_t>('N'),
        static_cast<uint8_t>('0' + layer)};
    figure.layer = layer;
    std::vector<uint8_t> payload;
    appendCommonHeader(payload, 0x0101);
    appendFigureRecord(payload, figure, 1);
    return makeMessage(payload);
}

rm_message::msg::SendMessage makeStringMessage()
{
    TestFigure figure;
    figure.name = {
        static_cast<uint8_t>('T'),
        static_cast<uint8_t>('X'),
        static_cast<uint8_t>('T')};
    figure.figure_type = 7;
    figure.layer = 2;
    figure.color = 8;
    figure.details_a = 16;
    figure.details_b = 5;
    figure.width = 4;
    figure.start_x = 120;
    figure.start_y = 120;
    figure.details_c = 0;
    figure.details_d = 0;
    figure.details_e = 0;

    std::vector<uint8_t> payload;
    appendCommonHeader(payload, 0x0110);
    appendFigureRecord(payload, figure, 1);
    const std::string text = "debug";
    for (size_t i = 0; i < 30; ++i) {
        payload.push_back(i < text.size() ? static_cast<uint8_t>(text[i]) : 0u);
    }
    return makeMessage(payload);
}

rm_message::msg::SendMessage makeDeleteMessage(uint8_t delete_type, uint8_t layer)
{
    std::vector<uint8_t> payload;
    appendCommonHeader(payload, 0x0100);
    payload.push_back(delete_type);
    payload.push_back(layer);
    return makeMessage(payload);
}

size_t countChangedPixels(const sensor_msgs::msg::Image & image)
{
    if (image.encoding != "bgr8" || image.height == 0 || image.width == 0) {
        return 0;
    }

    size_t changed_pixels = 0;
    for (size_t i = 0; i + 2 < image.data.size(); i += 3) {
        if (image.data[i] != 20 || image.data[i + 1] != 20 || image.data[i + 2] != 20) {
            ++changed_pixels;
        }
    }
    return changed_pixels;
}

class RmUiDebuggerHarness
{
public:
    RmUiDebuggerHarness()
    {
        const int id = counter_.fetch_add(1);
        const std::string suffix = std::to_string(id);
        input_topic_ = "/rm_ui_debugger_test_" + suffix + "/send_message";
        image_topic_ = "/rm_ui_debugger_test_" + suffix + "/image";

        rclcpp::NodeOptions options;
        options.parameter_overrides({
                rclcpp::Parameter("input_topic", input_topic_),
                rclcpp::Parameter("image_topic", image_topic_),
                rclcpp::Parameter("publish_hz", 100.0),
                rclcpp::Parameter("image_width", 320),
                rclcpp::Parameter("image_height", 180),
                rclcpp::Parameter("draw_names", false),
        });
        options.arguments({
                "--ros-args",
                "-r", "__node:=rm_ui_debugger_test_" + suffix,
        });

        debugger_node_ = std::make_shared<rm_ui::RmUiDebugger>(options);
        helper_node_ = std::make_shared<rclcpp::Node>("rm_ui_debugger_test_helper_" + suffix);

        send_pub_ = helper_node_->create_publisher<rm_message::msg::SendMessage>(input_topic_, 10);
        image_sub_ = helper_node_->create_subscription<sensor_msgs::msg::Image>(
            image_topic_,
            10,
            [this](const sensor_msgs::msg::Image::SharedPtr message) {
                images_.push_back(*message);
            });

        executor_.add_node(debugger_node_);
        executor_.add_node(helper_node_);
        spinFor(100ms);
    }

    ~RmUiDebuggerHarness()
    {
        executor_.remove_node(helper_node_);
        executor_.remove_node(debugger_node_);
    }

    void publishMessage(const rm_message::msg::SendMessage & message)
    {
        send_pub_->publish(message);
    }

    void publishLine(uint8_t layer = 1)
    {
        publishMessage(makeLineMessage(layer));
    }

    bool waitForChangedPixels(
        size_t threshold,
        std::chrono::milliseconds timeout)
    {
        const auto deadline = std::chrono::steady_clock::now() + timeout;
        while (std::chrono::steady_clock::now() < deadline) {
            executor_.spin_some();
            for (const auto & image : images_) {
                if (countChangedPixels(image) > threshold) {
                    return true;
                }
            }
            std::this_thread::sleep_for(1ms);
        }
        return false;
    }

    void clearImages()
    {
        images_.clear();
    }

    std::optional<size_t> latestChangedPixels() const
    {
        if (images_.empty()) {
            return std::nullopt;
        }
        return countChangedPixels(images_.back());
    }

    void spinFor(std::chrono::milliseconds duration)
    {
        const auto deadline = std::chrono::steady_clock::now() + duration;
        while (std::chrono::steady_clock::now() < deadline) {
            executor_.spin_some();
            std::this_thread::sleep_for(1ms);
        }
    }

    const std::vector<sensor_msgs::msg::Image> & images() const
    {
        return images_;
    }

private:
    static std::atomic<int> counter_;

    std::string input_topic_;
    std::string image_topic_;
    rclcpp::executors::SingleThreadedExecutor executor_;
    std::shared_ptr<rm_ui::RmUiDebugger> debugger_node_;
    rclcpp::Node::SharedPtr helper_node_;
    rclcpp::Publisher<rm_message::msg::SendMessage>::SharedPtr send_pub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    std::vector<sensor_msgs::msg::Image> images_;
};

std::atomic<int> RmUiDebuggerHarness::counter_{0};

} // namespace

TEST(RmUiDebugger, PublishesRenderedImageFromSendMessage)
{
    RmUiDebuggerHarness harness;
    harness.publishLine();

    ASSERT_TRUE(harness.waitForChangedPixels(10, 500ms));
    ASSERT_FALSE(harness.images().empty());
    const auto & image = harness.images().back();
    EXPECT_EQ(image.encoding, "bgr8");
    EXPECT_EQ(image.width, 320u);
    EXPECT_EQ(image.height, 180u);
    EXPECT_EQ(image.step, 960u);
}

TEST(RmUiDebugger, ParsesFigureBatchesAndNoopPadding)
{
    struct BatchCase
    {
        uint16_t content_id;
        size_t slot_count;
        size_t real_count;
    };

    const std::vector<BatchCase> cases{
        {0x0101, 1, 1},
        {0x0102, 2, 1},
        {0x0103, 5, 3},
        {0x0104, 7, 5},
    };

    for (const auto & batch_case : cases) {
        RmUiDebuggerHarness harness;
        harness.publishMessage(makeFigureBatchMessage(
            batch_case.content_id,
            batch_case.slot_count,
            batch_case.real_count));
        EXPECT_TRUE(harness.waitForChangedPixels(10, 500ms))
            << "content_id=0x" << std::hex << batch_case.content_id;
    }
}

TEST(RmUiDebugger, ParsesStringFigure)
{
    RmUiDebuggerHarness harness;
    harness.publishMessage(makeStringMessage());

    ASSERT_TRUE(harness.waitForChangedPixels(10, 500ms));
    ASSERT_FALSE(harness.images().empty());
    EXPECT_EQ(harness.images().back().encoding, "bgr8");
}

TEST(RmUiDebugger, AppliesDeleteLayerAndDeleteAll)
{
    RmUiDebuggerHarness harness;

    harness.publishLine(3);
    ASSERT_TRUE(harness.waitForChangedPixels(10, 500ms));

    harness.clearImages();
    harness.publishMessage(makeDeleteMessage(1, 3));
    harness.spinFor(150ms);
    ASSERT_TRUE(harness.latestChangedPixels().has_value());
    EXPECT_LT(harness.latestChangedPixels().value(), 10u);

    harness.clearImages();
    harness.publishLine(4);
    ASSERT_TRUE(harness.waitForChangedPixels(10, 500ms));

    harness.clearImages();
    harness.publishMessage(makeDeleteMessage(2, 0));
    harness.spinFor(150ms);
    ASSERT_TRUE(harness.latestChangedPixels().has_value());
    EXPECT_LT(harness.latestChangedPixels().value(), 10u);
}

TEST(RmUiDebugger, RejectsMalformedPayloadWithoutChangingCanvas)
{
    RmUiDebuggerHarness harness;
    auto malformed = makeLineMessage();
    malformed.data_payload.push_back(0xff);
    malformed.data_length = static_cast<uint16_t>(malformed.data_payload.size());

    harness.clearImages();
    harness.publishMessage(malformed);
    harness.spinFor(150ms);

    ASSERT_TRUE(harness.latestChangedPixels().has_value());
    EXPECT_LT(harness.latestChangedPixels().value(), 10u);
}

int main(int argc, char ** argv)
{
    testing::InitGoogleTest(&argc, argv);
    rclcpp::init(argc, argv);
    const int result = RUN_ALL_TESTS();
    rclcpp::shutdown();
    return result;
}
