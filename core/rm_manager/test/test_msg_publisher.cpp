#include "gtest/gtest.h"

#include "rm_manager/frame_parser.hpp"
#include "rm_manager/msg_publisher.hpp"

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <thread>
#include <vector>

namespace
{

using namespace std::chrono_literals;

void append_u16(std::vector<uint8_t> & data, uint16_t value)
{
    data.push_back(static_cast<uint8_t>(value & 0xFF));
    data.push_back(static_cast<uint8_t>((value >> 8) & 0xFF));
}

void append_u32(std::vector<uint8_t> & data, uint32_t value)
{
    data.push_back(static_cast<uint8_t>(value & 0xFF));
    data.push_back(static_cast<uint8_t>((value >> 8) & 0xFF));
    data.push_back(static_cast<uint8_t>((value >> 16) & 0xFF));
    data.push_back(static_cast<uint8_t>((value >> 24) & 0xFF));
}

class MsgPublisherTest : public ::testing::Test {
protected:
    static void SetUpTestSuite()
    {
        if (!rclcpp::ok()) {
            int argc = 0;
            char ** argv = nullptr;
            rclcpp::init(argc, argv);
        }
    }

    static void TearDownTestSuite()
    {
        if (rclcpp::ok()) {
            rclcpp::shutdown();
        }
    }

    void SetUp() override
    {
        static int node_index = 0;
        node_ = std::make_shared<rclcpp::Node>("msg_publisher_test_" +
              std::to_string(node_index++));
        publisher_ = std::make_unique<RMManager::MsgPublisher>(node_.get());
        topic_prefix_ = std::string("/") + node_->get_name() + "/";
        executor_.add_node(node_);
    }

    void TearDown() override
    {
        executor_.remove_node(node_);
        publisher_.reset();
        node_.reset();
    }

    template<typename MsgT>
    auto create_subscription(
        const std::string & topic_suffix,
        std::shared_ptr<MsgT> & last_msg,
        int & count)
    {
        return node_->create_subscription<MsgT>(
            topic_prefix_ + topic_suffix,
            10,
            [&](typename MsgT::SharedPtr msg) {
                last_msg = msg;
                ++count;
            });
    }

    bool spin_until(
        const std::function<bool()> & predicate,
        std::chrono::milliseconds timeout = 500ms)
    {
        const auto deadline = std::chrono::steady_clock::now() + timeout;
        while (std::chrono::steady_clock::now() < deadline) {
            executor_.spin_some();
            if (predicate()) {
                return true;
            }
            std::this_thread::sleep_for(10ms);
        }
        executor_.spin_some();
        return predicate();
    }

    rclcpp::executors::SingleThreadedExecutor executor_;
    rclcpp::Node::SharedPtr node_;
    std::unique_ptr<RMManager::MsgPublisher> publisher_;
    std::string topic_prefix_;
};

TEST_F(MsgPublisherTest, ParsesRobotBuffsV130Layout) {
    std::shared_ptr<rm_message::msg::RobotBuffs> last_msg;
    int count = 0;
    auto sub = create_subscription("robot_buffs", last_msg, count);
    (void)sub;

    std::vector<uint8_t> payload = {
        10,
        0x34, 0x12,
        50,
        30,
        0x78, 0x56,
        0x7F
    };

    publisher_->publish(RMManager::LinkType::Referee, 0x0204, payload);

    ASSERT_TRUE(spin_until([&]() {return count == 1;}));
    ASSERT_NE(last_msg, nullptr);
    EXPECT_EQ(last_msg->hp_buff, 10);
    EXPECT_EQ(last_msg->cooling_buff, 0x1234);
    EXPECT_EQ(last_msg->defense_buff, 50);
    EXPECT_EQ(last_msg->negative_defense_buff, 30);
    EXPECT_EQ(last_msg->attack_buff, 0x5678);
    EXPECT_TRUE(last_msg->energy_125);
    EXPECT_TRUE(last_msg->energy_100);
    EXPECT_TRUE(last_msg->energy_50);
    EXPECT_TRUE(last_msg->energy_30);
    EXPECT_TRUE(last_msg->energy_15);
    EXPECT_TRUE(last_msg->energy_5);
    EXPECT_TRUE(last_msg->energy_1);
}

TEST_F(MsgPublisherTest, ParsesRfidStatusFullTunnelLayout) {
    std::shared_ptr<rm_message::msg::RFIDStatus> last_msg;
    int count = 0;
    auto sub = create_subscription("rfid_status", last_msg, count);
    (void)sub;

    uint32_t flags = 0;
    flags |= (1u << 0);
    flags |= (1u << 19);
    flags |= (1u << 20);
    flags |= (1u << 21);
    flags |= (1u << 22);
    flags |= (1u << 26);
    flags |= (1u << 27);
    flags |= (1u << 28);
    flags |= (1u << 29);
    flags |= (1u << 30);
    flags |= (1u << 31);

    std::vector<uint8_t> payload;
    append_u32(payload, flags);
    payload.push_back(0x3F);

    publisher_->publish(RMManager::LinkType::Referee, 0x0209, payload);

    ASSERT_TRUE(spin_until([&]() {return count == 1;}));
    ASSERT_NE(last_msg, nullptr);
    EXPECT_TRUE(last_msg->base_gain);
    EXPECT_TRUE(last_msg->own_supply_area_non_overlap);
    EXPECT_TRUE(last_msg->own_supply_area_overlap);
    EXPECT_TRUE(last_msg->own_assembly_gain);
    EXPECT_TRUE(last_msg->enemy_assembly_gain);
    EXPECT_TRUE(last_msg->own_road_below);
    EXPECT_TRUE(last_msg->own_road_middle);
    EXPECT_TRUE(last_msg->own_road_above);
    EXPECT_TRUE(last_msg->own_trapezoid_low);
    EXPECT_TRUE(last_msg->own_trapezoid_middle);
    EXPECT_TRUE(last_msg->own_trapezoid_high);
    EXPECT_TRUE(last_msg->enemy_road_below);
    EXPECT_TRUE(last_msg->enemy_road_middle);
    EXPECT_TRUE(last_msg->enemy_road_above);
    EXPECT_TRUE(last_msg->enemy_trapezoid_low);
    EXPECT_TRUE(last_msg->enemy_trapezoid_middle);
    EXPECT_TRUE(last_msg->enemy_trapezoid_high);
}

TEST_F(MsgPublisherTest, ParsesRadarMarkAerialBits) {
    std::shared_ptr<rm_message::msg::RadarMark> last_msg;
    int count = 0;
    auto sub = create_subscription("radar_mark", last_msg, count);
    (void)sub;

    std::vector<uint8_t> payload;
    append_u16(payload, static_cast<uint16_t>((1u << 4) | (1u << 10)));

    publisher_->publish(RMManager::LinkType::Referee, 0x020C, payload);

    ASSERT_TRUE(spin_until([&]() {return count == 1;}));
    ASSERT_NE(last_msg, nullptr);
    EXPECT_TRUE(last_msg->enemy_aerial_special_mark);
    EXPECT_TRUE(last_msg->own_aerial_special_mark);
    EXPECT_FALSE(last_msg->enemy_sentry_vulnerable);
}

TEST_F(MsgPublisherTest, ParsesSentryDecisionOffsetFourBits) {
    std::shared_ptr<rm_message::msg::SentryDecision> last_msg;
    int count = 0;
    auto sub = create_subscription("sentry_decision", last_msg, count);
    (void)sub;

    uint32_t sentry_info = 0;
    sentry_info |= 513u;
    sentry_info |= (3u << 11);
    sentry_info |= (5u << 15);
    sentry_info |= (1u << 19);
    sentry_info |= (1u << 20);
    sentry_info |= (341u << 21);

    uint16_t sentry_info_2 = 0;
    sentry_info_2 |= 1u;
    sentry_info_2 |= static_cast<uint16_t>(341u << 1);
    sentry_info_2 |= static_cast<uint16_t>(2u << 12);
    sentry_info_2 |= static_cast<uint16_t>(1u << 14);

    std::vector<uint8_t> payload;
    append_u32(payload, sentry_info);
    append_u16(payload, sentry_info_2);

    publisher_->publish(RMManager::LinkType::Referee, 0x020D, payload);

    ASSERT_TRUE(spin_until([&]() {return count == 1;}));
    ASSERT_NE(last_msg, nullptr);
    EXPECT_EQ(last_msg->allow_bullet_count, 513);
    EXPECT_EQ(last_msg->exchange_bullet_count, 3);
    EXPECT_EQ(last_msg->exchange_hp_count, 5);
    EXPECT_TRUE(last_msg->free_revive_available);
    EXPECT_TRUE(last_msg->immediate_revive_available);
    EXPECT_EQ(last_msg->immediate_revive_coin_cost, 341);
    EXPECT_TRUE(last_msg->is_out_of_combat);
    EXPECT_EQ(last_msg->remaining_exchangeable_17mm, 341);
    EXPECT_EQ(last_msg->sentry_attitude, 2);
    EXPECT_TRUE(last_msg->energy_gear_available);
}

TEST_F(MsgPublisherTest, RoutesCommandsByLinkAndPreservesRawTopics) {
    std::shared_ptr<rm_message::msg::CustomController> custom_controller_msg;
    std::shared_ptr<rm_message::msg::ClientCustomCommand> client_custom_command_msg;
    std::shared_ptr<rm_message::msg::SetVTMChannel> set_vtm_msg;
    std::shared_ptr<rm_message::msg::QueryVTMChannel> query_vtm_msg;
    std::shared_ptr<rm_message::msg::GeneralMessage> unknown_msg;
    std::shared_ptr<rm_message::msg::GeneralMessage> all_msg;
    int custom_controller_count = 0;
    int client_custom_command_count = 0;
    int set_vtm_count = 0;
    int query_vtm_count = 0;
    int unknown_count = 0;
    int all_count = 0;

    auto custom_controller_sub =
      create_subscription("custom_controller", custom_controller_msg, custom_controller_count);
    auto client_custom_command_sub =
      create_subscription("client_custom_command", client_custom_command_msg,
      client_custom_command_count);
    auto set_vtm_sub = create_subscription("set_vtm_channel", set_vtm_msg, set_vtm_count);
    auto query_vtm_sub = create_subscription("query_vtm_channel", query_vtm_msg, query_vtm_count);
    auto unknown_sub = create_subscription("unknown_command", unknown_msg, unknown_count);
    auto all_sub = create_subscription("all_messages", all_msg, all_count);
    (void)custom_controller_sub;
    (void)client_custom_command_sub;
    (void)set_vtm_sub;
    (void)query_vtm_sub;
    (void)unknown_sub;
    (void)all_sub;

    std::vector<uint8_t> data30(30, 0x11);
    std::vector<uint8_t> data12(12, 0x22);

    publisher_->publish(RMManager::LinkType::Image, 0x0302, data30);
    publisher_->publish(RMManager::LinkType::Image, 0x0311, data30);
    publisher_->publish(RMManager::LinkType::Image, 0x0F01, {4});
    publisher_->publish(RMManager::LinkType::Image, 0x0F02, {6});
    publisher_->publish(RMManager::LinkType::Referee, 0x0302, data30);
    publisher_->publish(RMManager::LinkType::Image, 0x0304, data12);

    ASSERT_TRUE(spin_until([&]() {
          return custom_controller_count == 1 &&
                 client_custom_command_count == 1 &&
                 set_vtm_count == 1 &&
                 query_vtm_count == 1 &&
                 unknown_count == 2 &&
                 all_count == 6;
    }));
    ASSERT_NE(custom_controller_msg, nullptr);
    ASSERT_NE(client_custom_command_msg, nullptr);
    ASSERT_NE(set_vtm_msg, nullptr);
    ASSERT_NE(query_vtm_msg, nullptr);
    EXPECT_EQ(custom_controller_count, 1);
    EXPECT_EQ(client_custom_command_count, 1);
    EXPECT_EQ(set_vtm_count, 1);
    EXPECT_EQ(query_vtm_count, 1);
    EXPECT_EQ(unknown_count, 2);
    EXPECT_EQ(all_count, 6);
}

TEST(FrameParserTest, RejectsLegacyA953Frame) {
    std::vector<uint8_t> payload(21, 0);
    payload[0] = 0xA9;
    payload[1] = 0x53;

    RMManager::ParsedFrame frame;
    EXPECT_EQ(
        RMManager::parse_standard_frame(payload, 0, frame),
        RMManager::FrameParseResult::kInvalidHeader);
}

}  // namespace
