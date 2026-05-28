#include <gtest/gtest.h>

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/twist_stamped.hpp"
#include "remote_controller/remote_controller.hpp"
#include "rm_message/msg/remote_control.hpp"
#include "rm_message/srv/get_chassis_control_source_list.hpp"
#include "rm_message/srv/get_current_chassis_control_source.hpp"
#include "rm_message/srv/set_current_chassis_control_source.hpp"
#include "rclcpp/rclcpp.hpp"

namespace RM_REMOTE_CONTROLLER
{
namespace
{

using namespace std::chrono_literals;

rm_message::msg::RemoteControl MakeNeutralRemoteControl()
{
    rm_message::msg::RemoteControl msg;
    msg.chanal0 = 1024;
    msg.chanal1 = 1024;
    msg.chanal2 = 1024;
    msg.chanal3 = 1024;
    return msg;
}

class RemoteControllerTest : public ::testing::Test
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
        const std::vector<rclcpp::Parameter> parameters = GetParameterOverrides();

        rclcpp::NodeOptions options;
        options.use_global_arguments(false);
        options.parameter_overrides(parameters);

        dut_node_ = std::make_shared<RemoteController>(options);
        helper_node_ = std::make_shared<rclcpp::Node>("remote_controller_test_helper");

        remote_pub_ = helper_node_->create_publisher<rm_message::msg::RemoteControl>(
            "/test/remote_controller/input",
            10);

        get_current_client_ =
            helper_node_->create_client<rm_message::srv::GetCurrentChassisControlSource>(
            "/remote_controller/get_current_chassis_control_source");
        get_list_client_ =
            helper_node_->create_client<rm_message::srv::GetChassisControlSourceList>(
            "/remote_controller/get_chassis_control_source_list");
        set_current_client_ =
            helper_node_->create_client<rm_message::srv::SetCurrentChassisControlSource>(
            "/remote_controller/set_current_chassis_control_source");

        executor_.add_node(dut_node_);
        executor_.add_node(helper_node_);

        ASSERT_TRUE(SpinUntil(
            [this]() {
                return remote_pub_->get_subscription_count() > 0 &&
                       get_current_client_->service_is_ready() &&
                       get_list_client_->service_is_ready() &&
                       set_current_client_->service_is_ready();
            },
            2s));
    }

    virtual std::vector<rclcpp::Parameter> GetParameterOverrides()
    {
        return {
            rclcpp::Parameter("remote_controller_topic", "/test/remote_controller/input"),
            rclcpp::Parameter("cmd_vel_topic", "/test/remote_controller/cmd_vel"),
            rclcpp::Parameter("chasis_enable_topic", "/test/remote_controller/chassis_enable"),
            rclcpp::Parameter("arm_enable_topic", "/test/remote_controller/arm_enable"),
            rclcpp::Parameter(
                "bridge_topics",
                std::vector<std::string>{
                    "/test/remote_controller/bridge_a",
                    "/test/remote_controller/bridge_b"}),
            rclcpp::Parameter("watchdog_enabled", false),
        };
    }

    void TearDown() override
    {
        executor_.remove_node(helper_node_);
        executor_.remove_node(dut_node_);

        set_current_client_.reset();
        get_list_client_.reset();
        get_current_client_.reset();
        remote_pub_.reset();
        helper_node_.reset();
        dut_node_.reset();
    }

    bool SpinUntil(
        const std::function<bool()> & predicate,
        std::chrono::milliseconds timeout)
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

    template<typename ServiceT>
    std::shared_ptr<typename ServiceT::Response> CallService(
        const typename rclcpp::Client<ServiceT>::SharedPtr & client,
        const std::shared_ptr<typename ServiceT::Request> & request)
    {
        auto future = client->async_send_request(request);
        EXPECT_EQ(
            executor_.spin_until_future_complete(future, 2s),
            rclcpp::FutureReturnCode::SUCCESS);
        return future.get();
    }

    std::string GetCurrentSource()
    {
        auto request =
            std::make_shared<rm_message::srv::GetCurrentChassisControlSource::Request>();
        auto response = CallService<rm_message::srv::GetCurrentChassisControlSource>(
            get_current_client_,
            request);
        return response->source;
    }

    std::vector<std::string> GetSourceList()
    {
        auto request =
            std::make_shared<rm_message::srv::GetChassisControlSourceList::Request>();
        auto response = CallService<rm_message::srv::GetChassisControlSourceList>(
            get_list_client_,
            request);
        return response->sources;
    }

    std::shared_ptr<rm_message::srv::SetCurrentChassisControlSource::Response> SetCurrentSource(
        const std::string & source)
    {
        auto request =
            std::make_shared<rm_message::srv::SetCurrentChassisControlSource::Request>();
        request->source = source;
        return CallService<rm_message::srv::SetCurrentChassisControlSource>(
            set_current_client_,
            request);
    }

    void PublishRemoteControl(const rm_message::msg::RemoteControl & msg)
    {
        remote_pub_->publish(msg);
        executor_.spin_some();
        std::this_thread::sleep_for(20ms);
        executor_.spin_some();
    }

    void PressControlSourceSwitch()
    {
        PressKeyL();
    }

    void PressKeyL()
    {
        auto pressed_msg = MakeNeutralRemoteControl();
        pressed_msg.keyl = 1;
        PublishRemoteControl(pressed_msg);

        auto released_msg = MakeNeutralRemoteControl();
        released_msg.keyl = 0;
        PublishRemoteControl(released_msg);
    }

    void PressKeyR()
    {
        auto pressed_msg = MakeNeutralRemoteControl();
        pressed_msg.keyr = 1;
        PublishRemoteControl(pressed_msg);

        auto released_msg = MakeNeutralRemoteControl();
        released_msg.keyr = 0;
        PublishRemoteControl(released_msg);
    }

    void PressKeyLAndKeyR()
    {
        auto pressed_msg = MakeNeutralRemoteControl();
        pressed_msg.keyl = 1;
        pressed_msg.keyr = 1;
        PublishRemoteControl(pressed_msg);

        auto released_msg = MakeNeutralRemoteControl();
        released_msg.keyl = 0;
        released_msg.keyr = 0;
        PublishRemoteControl(released_msg);
    }

    rclcpp::executors::SingleThreadedExecutor executor_;
    std::shared_ptr<RemoteController> dut_node_;
    rclcpp::Node::SharedPtr helper_node_;
    rclcpp::Publisher<rm_message::msg::RemoteControl>::SharedPtr remote_pub_;
    rclcpp::Client<rm_message::srv::GetCurrentChassisControlSource>::SharedPtr get_current_client_;
    rclcpp::Client<rm_message::srv::GetChassisControlSourceList>::SharedPtr get_list_client_;
    rclcpp::Client<rm_message::srv::SetCurrentChassisControlSource>::SharedPtr set_current_client_;
};

class RemoteControllerSwitchListTest : public RemoteControllerTest
{
protected:
    std::vector<rclcpp::Parameter> GetParameterOverrides() override
    {
        auto parameters = RemoteControllerTest::GetParameterOverrides();
        parameters.emplace_back(
            "control_source_switch_list",
            std::vector<std::string>{
                "KEYBOARD_MOUSE",
                "/test/remote_controller/bridge_b"});
        return parameters;
    }
};

class RemoteControllerMultiSwitchKeysTest : public RemoteControllerTest
{
protected:
    std::vector<rclcpp::Parameter> GetParameterOverrides() override
    {
        auto parameters = RemoteControllerTest::GetParameterOverrides();
        parameters.emplace_back(
            "control_source_switch_keys",
            std::vector<std::string>{"KEYL", "KEYR"});
        return parameters;
    }
};

class RemoteControllerDuplicateSwitchKeysTest : public RemoteControllerTest
{
protected:
    std::vector<rclcpp::Parameter> GetParameterOverrides() override
    {
        auto parameters = RemoteControllerTest::GetParameterOverrides();
        parameters.emplace_back(
            "control_source_switch_keys",
            std::vector<std::string>{"KEYL", "KEYL", "KEYR"});
        return parameters;
    }
};

class RemoteControllerLegacySwitchKeyTest : public RemoteControllerTest
{
protected:
    std::vector<rclcpp::Parameter> GetParameterOverrides() override
    {
        auto parameters = RemoteControllerTest::GetParameterOverrides();
        parameters.emplace_back("control_source_switch_key", "KEYR");
        return parameters;
    }
};

class RemoteControllerSwitchKeysPriorityTest : public RemoteControllerTest
{
protected:
    std::vector<rclcpp::Parameter> GetParameterOverrides() override
    {
        auto parameters = RemoteControllerTest::GetParameterOverrides();
        parameters.emplace_back("control_source_switch_key", "KEYR");
        parameters.emplace_back(
            "control_source_switch_keys",
            std::vector<std::string>{"KEYL"});
        return parameters;
    }
};

class RemoteControllerParamValidationTest : public ::testing::Test
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

class RemoteControllerWatchdogTest : public ::testing::Test
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
            rclcpp::Parameter("remote_controller_topic", "/test/remote_controller/watchdog/input"),
            rclcpp::Parameter("cmd_vel_topic", "/test/remote_controller/watchdog/cmd_vel"),
            rclcpp::Parameter(
                "chasis_enable_topic",
                "/test/remote_controller/watchdog/chassis_enable"),
            rclcpp::Parameter("arm_enable_topic", "/test/remote_controller/watchdog/arm_enable"),
            rclcpp::Parameter(
                "bridge_topics",
                std::vector<std::string>{"/test/remote_controller/watchdog/bridge_a"}),
            rclcpp::Parameter("watchdog_enabled", true),
            rclcpp::Parameter("watchdog_timeout", 0.2),
        };

        rclcpp::NodeOptions options;
        options.use_global_arguments(false);
        options.parameter_overrides(parameters);

        dut_node_ = std::make_shared<RemoteController>(options);
        helper_node_ = std::make_shared<rclcpp::Node>("remote_controller_watchdog_test_helper");

        bridge_pub_ = helper_node_->create_publisher<geometry_msgs::msg::TwistStamped>(
            "/test/remote_controller/watchdog/bridge_a",
            10);
        chassis_enable_sub_ = helper_node_->create_subscription<std_msgs::msg::Bool>(
            "/test/remote_controller/watchdog/chassis_enable",
            10,
            [this](const std_msgs::msg::Bool::SharedPtr msg) {
                if (!msg->data) {
                    chassis_disable_received_ = true;
                }
            });
        set_current_client_ =
            helper_node_->create_client<rm_message::srv::SetCurrentChassisControlSource>(
            "/remote_controller/set_current_chassis_control_source");

        executor_.add_node(dut_node_);
        executor_.add_node(helper_node_);

        ASSERT_TRUE(SpinUntil(
            [this]() {
                return bridge_pub_->get_subscription_count() > 0 &&
                       set_current_client_->service_is_ready();
            },
            2s));
    }

    void TearDown() override
    {
        executor_.remove_node(helper_node_);
        executor_.remove_node(dut_node_);

        set_current_client_.reset();
        chassis_enable_sub_.reset();
        bridge_pub_.reset();
        helper_node_.reset();
        dut_node_.reset();
        chassis_disable_received_ = false;
    }

    bool SpinUntil(
        const std::function<bool()> & predicate,
        std::chrono::milliseconds timeout)
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

    void SpinFor(std::chrono::milliseconds duration)
    {
        const auto deadline = std::chrono::steady_clock::now() + duration;
        while (std::chrono::steady_clock::now() < deadline) {
            executor_.spin_some();
            std::this_thread::sleep_for(10ms);
        }
        executor_.spin_some();
    }

    std::shared_ptr<rm_message::srv::SetCurrentChassisControlSource::Response> SetCurrentSource(
        const std::string & source)
    {
        auto request =
            std::make_shared<rm_message::srv::SetCurrentChassisControlSource::Request>();
        request->source = source;

        auto future = set_current_client_->async_send_request(request);
        EXPECT_EQ(
            executor_.spin_until_future_complete(future, 2s),
            rclcpp::FutureReturnCode::SUCCESS);
        return future.get();
    }

    void PublishBridgeCommand()
    {
        geometry_msgs::msg::TwistStamped msg;
        msg.header.stamp = helper_node_->now();
        msg.header.frame_id = "base_link";
        msg.twist.linear.x = 0.1;
        bridge_pub_->publish(msg);
        SpinFor(40ms);
    }

    rclcpp::executors::SingleThreadedExecutor executor_;
    std::shared_ptr<RemoteController> dut_node_;
    rclcpp::Node::SharedPtr helper_node_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr bridge_pub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr chassis_enable_sub_;
    rclcpp::Client<rm_message::srv::SetCurrentChassisControlSource>::SharedPtr set_current_client_;
    bool chassis_disable_received_ = false;
};

TEST_F(RemoteControllerTest, ExposesInitialControlSourceAndSourceList)
{
    const std::vector<std::string> expected_sources = {
        "REMOTE_CHANNEL",
        "KEYBOARD_MOUSE",
        "/test/remote_controller/bridge_a",
        "/test/remote_controller/bridge_b",
    };

    EXPECT_EQ(GetCurrentSource(), "REMOTE_CHANNEL");
    EXPECT_EQ(GetSourceList(), expected_sources);
}

TEST_F(RemoteControllerTest, SetsCurrentControlSourceViaService)
{
    auto response = SetCurrentSource("/test/remote_controller/bridge_b");

    ASSERT_TRUE(response->success);
    EXPECT_EQ(response->current_source, "/test/remote_controller/bridge_b");
    EXPECT_EQ(response->message, "Current control source: /test/remote_controller/bridge_b");
    EXPECT_EQ(GetCurrentSource(), "/test/remote_controller/bridge_b");
}

TEST_F(RemoteControllerTest, RejectsUnknownControlSourceViaService)
{
    auto response = SetCurrentSource("/test/remote_controller/missing");

    EXPECT_FALSE(response->success);
    EXPECT_EQ(response->current_source, "REMOTE_CHANNEL");
    EXPECT_EQ(response->message, "Unknown control source: /test/remote_controller/missing");
    EXPECT_EQ(GetCurrentSource(), "REMOTE_CHANNEL");
}

TEST_F(RemoteControllerTest, CyclesControlSourcesWithSwitchButton)
{
    const std::vector<std::string> expected_sources = {
        "KEYBOARD_MOUSE",
        "/test/remote_controller/bridge_a",
        "/test/remote_controller/bridge_b",
        "REMOTE_CHANNEL",
    };

    for (const auto & expected_source : expected_sources) {
        PressControlSourceSwitch();
        ASSERT_TRUE(SpinUntil([this, &expected_source]() { return GetCurrentSource() == expected_source; }, 2s));
    }
}

TEST_F(RemoteControllerMultiSwitchKeysTest, CyclesControlSourcesWithMultipleSwitchKeys)
{
    PressKeyL();
    ASSERT_TRUE(SpinUntil([this]() { return GetCurrentSource() == "KEYBOARD_MOUSE"; }, 2s));

    PressKeyR();
    ASSERT_TRUE(SpinUntil(
        [this]() { return GetCurrentSource() == "/test/remote_controller/bridge_a"; },
        2s));
}

TEST_F(RemoteControllerDuplicateSwitchKeysTest, DeduplicatesSwitchKeys)
{
    PressKeyL();
    ASSERT_TRUE(SpinUntil([this]() { return GetCurrentSource() == "KEYBOARD_MOUSE"; }, 2s));
}

TEST_F(RemoteControllerMultiSwitchKeysTest, SimultaneousSwitchKeyEdgesCycleOnlyOnce)
{
    PressKeyLAndKeyR();
    ASSERT_TRUE(SpinUntil([this]() { return GetCurrentSource() == "KEYBOARD_MOUSE"; }, 2s));
}

TEST_F(RemoteControllerLegacySwitchKeyTest, UsesLegacySwitchKeyWhenPluralKeyIsNotConfigured)
{
    PressKeyL();
    EXPECT_EQ(GetCurrentSource(), "REMOTE_CHANNEL");

    PressKeyR();
    ASSERT_TRUE(SpinUntil([this]() { return GetCurrentSource() == "KEYBOARD_MOUSE"; }, 2s));
}

TEST_F(RemoteControllerSwitchKeysPriorityTest, PluralSwitchKeysOverrideLegacySwitchKey)
{
    PressKeyR();
    EXPECT_EQ(GetCurrentSource(), "REMOTE_CHANNEL");

    PressKeyL();
    ASSERT_TRUE(SpinUntil([this]() { return GetCurrentSource() == "KEYBOARD_MOUSE"; }, 2s));
}

TEST_F(RemoteControllerSwitchListTest, CyclesOnlyAllowedSourcesWithSwitchButton)
{
    const std::vector<std::string> expected_sources = {
        "KEYBOARD_MOUSE",
        "/test/remote_controller/bridge_b",
        "KEYBOARD_MOUSE",
    };

    EXPECT_EQ(GetCurrentSource(), "REMOTE_CHANNEL");

    for (const auto & expected_source : expected_sources) {
        PressControlSourceSwitch();
        ASSERT_TRUE(SpinUntil([this, &expected_source]() { return GetCurrentSource() == expected_source; }, 2s));
    }
}

TEST_F(RemoteControllerSwitchListTest, ServiceCanSelectSourceSkippedBySwitchButton)
{
    auto response = SetCurrentSource("/test/remote_controller/bridge_a");

    ASSERT_TRUE(response->success);
    EXPECT_EQ(GetCurrentSource(), "/test/remote_controller/bridge_a");

    PressControlSourceSwitch();
    ASSERT_TRUE(SpinUntil(
        [this]() { return GetCurrentSource() == "/test/remote_controller/bridge_b"; },
        2s));
}

TEST_F(RemoteControllerParamValidationTest, RejectsDuplicateBridgeTopics)
{
    rclcpp::NodeOptions options;
    options.use_global_arguments(false);
    options.parameter_overrides({
        rclcpp::Parameter("bridge_topics", std::vector<std::string>{"/test/bridge", "/test/bridge"}),
        rclcpp::Parameter("watchdog_enabled", false),
    });

    EXPECT_THROW(
        {
            auto node = std::make_shared<RemoteController>(options);
            (void)node;
        },
        std::exception);
}

TEST_F(RemoteControllerParamValidationTest, RejectsEmptyBridgeTopic)
{
    rclcpp::NodeOptions options;
    options.use_global_arguments(false);
    options.parameter_overrides({
        rclcpp::Parameter("bridge_topics", std::vector<std::string>{""}),
        rclcpp::Parameter("watchdog_enabled", false),
    });

    EXPECT_THROW(
        {
            auto node = std::make_shared<RemoteController>(options);
            (void)node;
        },
        std::exception);
}

TEST_F(RemoteControllerParamValidationTest, RejectsUnknownControlSourceSwitchEntry)
{
    rclcpp::NodeOptions options;
    options.use_global_arguments(false);
    options.parameter_overrides({
        rclcpp::Parameter(
            "bridge_topics",
            std::vector<std::string>{"/test/bridge"}),
        rclcpp::Parameter(
            "control_source_switch_list",
            std::vector<std::string>{"/test/missing"}),
        rclcpp::Parameter("watchdog_enabled", false),
    });

    EXPECT_THROW(
        {
            auto node = std::make_shared<RemoteController>(options);
            (void)node;
        },
        std::exception);
}

TEST_F(RemoteControllerParamValidationTest, RejectsDuplicateControlSourceSwitchEntry)
{
    rclcpp::NodeOptions options;
    options.use_global_arguments(false);
    options.parameter_overrides({
        rclcpp::Parameter(
            "control_source_switch_list",
            std::vector<std::string>{"KEYBOARD_MOUSE", "KEYBOARD_MOUSE"}),
        rclcpp::Parameter("watchdog_enabled", false),
    });

    EXPECT_THROW(
        {
            auto node = std::make_shared<RemoteController>(options);
            (void)node;
        },
        std::exception);
}

TEST_F(RemoteControllerParamValidationTest, RejectsEmptyControlSourceSwitchEntry)
{
    rclcpp::NodeOptions options;
    options.use_global_arguments(false);
    options.parameter_overrides({
        rclcpp::Parameter(
            "control_source_switch_list",
            std::vector<std::string>{""}),
        rclcpp::Parameter("watchdog_enabled", false),
    });

    EXPECT_THROW(
        {
            auto node = std::make_shared<RemoteController>(options);
            (void)node;
        },
        std::exception);
}

TEST_F(RemoteControllerParamValidationTest, RejectsEmptyControlSourceSwitchKeys)
{
    rclcpp::NodeOptions options;
    options.use_global_arguments(false);
    options.parameter_overrides({
        rclcpp::Parameter(
            "control_source_switch_keys",
            std::vector<std::string>{}),
        rclcpp::Parameter("watchdog_enabled", false),
    });

    EXPECT_THROW(
        {
            auto node = std::make_shared<RemoteController>(options);
            (void)node;
        },
        std::exception);
}

TEST_F(RemoteControllerWatchdogTest, BridgeTrafficKeepsWatchdogAliveUntilBridgeStops)
{
    auto response = SetCurrentSource("/test/remote_controller/watchdog/bridge_a");
    ASSERT_TRUE(response->success);

    for (size_t i = 0; i < 8; ++i) {
        PublishBridgeCommand();
    }
    EXPECT_FALSE(chassis_disable_received_);

    SpinFor(300ms);
    EXPECT_TRUE(chassis_disable_received_);
}

}  // namespace
}  // namespace RM_REMOTE_CONTROLLER
