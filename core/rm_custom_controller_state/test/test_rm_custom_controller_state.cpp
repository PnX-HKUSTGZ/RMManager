#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstring>
#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/twist_stamped.hpp"
#include "rm_custom_controller_state/rm_custom_controller_state.hpp"
#include "rm_message/msg/custom_controller.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/bool.hpp"

namespace rm_custom_controller_state
{
namespace
{

using namespace std::chrono_literals;

class RmCustomControllerStateTest : public ::testing::Test
{
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
        const std::vector<rclcpp::Parameter> parameters = {
            rclcpp::Parameter("arm_state_topic", "/test/custom_controller/joint_states"),
            rclcpp::Parameter(
        "joint_names",
        std::vector<std::string>{
                        "control_j0", "control_j1", "control_j2", "control_j3",
                        "control_j4", "control_j5", "control_j6"}),
            rclcpp::Parameter(
        "joint_reverse",
        std::vector<bool>{false, true, false, true, false, false, true}),
            rclcpp::Parameter("ref_topic", "/test/custom_controller/ref"),
            rclcpp::Parameter("watchdog_timeout", 1.0),
            rclcpp::Parameter("enable_chassis_cmd", true),
            rclcpp::Parameter("chassis_cmd_topic", "/test/custom_controller/chassis_cmd"),
            rclcpp::Parameter(
        "channel_mapping",
        std::vector<std::string>{"linear_x", "linear_y", "angular_z", "none"}),
            rclcpp::Parameter("channel_max", std::vector<double>{1.5, 2.0, 3.0, 1.0}),
        };

        rclcpp::NodeOptions options;
        options.use_global_arguments(false);
        options.parameter_overrides(parameters);

        dut_node_ = std::make_shared<RmCustomControllerState>(options);
        helper_node_ = std::make_shared<rclcpp::Node>("rm_custom_controller_state_test_helper");

        joint_state_sub_ = helper_node_->create_subscription<sensor_msgs::msg::JointState>(
      "/test/custom_controller/joint_states",
      10,
            [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
                last_joint_state_ = *msg;
      });

        chassis_cmd_sub_ = helper_node_->create_subscription<geometry_msgs::msg::TwistStamped>(
      "/test/custom_controller/chassis_cmd",
      10,
            [this](const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
                last_twist_ = *msg;
      });

        for (size_t i = 0; i < GPIO_NUM; ++i) {
            gpio_subs_[i] = helper_node_->create_subscription<std_msgs::msg::Bool>(
        "/rm_custom_controller_state/gpio" + std::to_string(i) + "_state",
        10,
                [this, i](const std_msgs::msg::Bool::SharedPtr msg) {
                    last_gpio_[i] = msg->data;
        });
        }

        ref_pub_ = helper_node_->create_publisher<rm_message::msg::CustomController>(
      "/test/custom_controller/ref", 10);

        executor_.add_node(dut_node_);
        executor_.add_node(helper_node_);
    }

    void TearDown() override
    {
        executor_.remove_node(helper_node_);
        executor_.remove_node(dut_node_);
        ref_pub_.reset();
        joint_state_sub_.reset();
        chassis_cmd_sub_.reset();
        for (auto & sub : gpio_subs_) {
            sub.reset();
        }
        last_joint_state_.reset();
        last_twist_.reset();
        last_gpio_.fill(std::nullopt);
        helper_node_.reset();
        dut_node_.reset();
    }

    bool SpinUntil(const std::function<bool()> & predicate, std::chrono::milliseconds timeout)
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

    void PublishControlData(const ControlData & control_data)
    {
        rm_message::msg::CustomController msg;
        std::memcpy(msg.data.data(), &control_data, sizeof(control_data));

        ASSERT_TRUE(SpinUntil(
              [this]() {return ref_pub_->get_subscription_count() > 0;}, 2s));

        ref_pub_->publish(msg);
    }

    rclcpp::executors::SingleThreadedExecutor executor_;
    std::shared_ptr<RmCustomControllerState> dut_node_;
    rclcpp::Node::SharedPtr helper_node_;
    rclcpp::Publisher<rm_message::msg::CustomController>::SharedPtr ref_pub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr chassis_cmd_sub_;
    std::array<rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr, GPIO_NUM> gpio_subs_;
    std::optional<sensor_msgs::msg::JointState> last_joint_state_;
    std::optional<geometry_msgs::msg::TwistStamped> last_twist_;
    std::array<std::optional<bool>, GPIO_NUM> last_gpio_{};
};

class RmCustomControllerStateParamValidationTest : public ::testing::Test
{
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
};

TEST_F(RmCustomControllerStateTest, PublishesDecodedJointStateAndAuxOutputs)
{
  const std::array<float, JOINT_NUM> expected_positions = {
        0.0f, 1.0f, -1.0f, 2.0f, -2.5f, 0.5f, -0.75f};

  ControlData control_data{};
  for (size_t i = 0; i < JOINT_NUM; ++i) {
        control_data.rotor_angles[i] = static_cast<uint16_t>(
            float_to_uint(expected_positions[i], POS_MIN, POS_MAX, BITS));
  }
  control_data.channel_0 = 254;
  control_data.channel_1 = 0;
  control_data.channel_2 = 254;
  control_data.channel_3 = 127;
  control_data.gpio_state = 0b10100101;

  PublishControlData(control_data);

  ASSERT_TRUE(SpinUntil(
          [this]() {
              const bool gpio_ready = std::all_of(
        last_gpio_.begin(), last_gpio_.end(),
                  [](const auto & value) {return value.has_value();});
              return last_joint_state_.has_value() && last_twist_.has_value() && gpio_ready;
    },
    2s));

  ASSERT_EQ(last_joint_state_->name.size(), JOINT_NUM);
  ASSERT_EQ(last_joint_state_->position.size(), JOINT_NUM);

  const std::array<float, JOINT_NUM> expected_output_positions = {
        0.0f, -1.0f, -1.0f, -2.0f, -2.5f, 0.5f, 0.75f};

  for (size_t i = 0; i < JOINT_NUM; ++i) {
        EXPECT_EQ(last_joint_state_->name[i], "control_j" + std::to_string(i));
        EXPECT_NEAR(last_joint_state_->position[i], expected_output_positions[i], 1e-3);
  }

  EXPECT_NEAR(last_twist_->twist.linear.x, 1.5, 1e-6);
  EXPECT_NEAR(last_twist_->twist.linear.y, -2.0, 1e-6);
  EXPECT_NEAR(last_twist_->twist.angular.z, 3.0, 1e-6);

  const std::array<bool, GPIO_NUM> expected_gpio = {true, false, true, false, false, true, false,
        true};
  for (size_t i = 0; i < GPIO_NUM; ++i) {
        ASSERT_TRUE(last_gpio_[i].has_value());
        EXPECT_EQ(*last_gpio_[i], expected_gpio[i]);
  }
}

TEST_F(RmCustomControllerStateParamValidationTest, RejectsInvalidJointNamesLength)
{
  const std::vector<rclcpp::Parameter> parameters = {
        rclcpp::Parameter("arm_state_topic", "/test/custom_controller/joint_states"),
        rclcpp::Parameter(
      "joint_names",
      std::vector<std::string>{
                    "control_j0", "control_j1", "control_j2", "control_j3",
                    "control_j4", "control_j5"}),
        rclcpp::Parameter(
      "joint_reverse",
      std::vector<bool>{false, false, false, false, false, false, false}),
        rclcpp::Parameter("ref_topic", "/test/custom_controller/ref"),
  };

  rclcpp::NodeOptions options;
  options.use_global_arguments(false);
  options.parameter_overrides(parameters);

  EXPECT_THROW(
            {
                auto node = std::make_shared<RmCustomControllerState>(options);
                (void)node;
    },
    std::exception);
}

}  // namespace
}  // namespace rm_custom_controller_state
