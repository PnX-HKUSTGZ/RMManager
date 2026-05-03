#include "remote_controller/remote_controller.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <stdexcept>
#include <unordered_set>

namespace RM_REMOTE_CONTROLLER {
namespace {

constexpr char kRemoteChannelSource[] = "REMOTE_CHANNEL";
constexpr char kKeyboardMouseSource[] = "KEYBOARD_MOUSE";

}  // namespace

RemoteController::RemoteController(const rclcpp::NodeOptions & options)
: rclcpp::Node("remote_controller", options)
{
    // Initialize parameter listener and get parameters
    param_listener_ =
        std::make_shared<remote_controller::ParamListener>(this->get_node_parameters_interface());
    params_ = std::make_shared<remote_controller::Params>(param_listener_->get_params());

    // Output parameter values
    RCLCPP_INFO(this->get_logger(), "Parameters:");
    RCLCPP_INFO(this->get_logger(), "  cmd_vel_topic: %s", params_->cmd_vel_topic.c_str());
    RCLCPP_INFO(
        this->get_logger(), "  remote_controller_topic: %s",
        params_->remote_controller_topic.c_str());
    RCLCPP_INFO(
        this->get_logger(), "  chasis_enable_topic: %s",
        params_->chasis_enable_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "  arm_enable_topic: %s", params_->arm_enable_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "  max_x: %.2f", params_->max_x);
    RCLCPP_INFO(this->get_logger(), "  max_y: %.2f", params_->max_y);
    RCLCPP_INFO(this->get_logger(), "  max_z: %.2f", params_->max_z);
    RCLCPP_INFO(this->get_logger(), "  delta_x: %.2f", params_->delta_x);
    RCLCPP_INFO(this->get_logger(), "  delta_y: %.2f", params_->delta_y);
    RCLCPP_INFO(this->get_logger(), "  delta_z: %.2f", params_->delta_z);
    RCLCPP_INFO(this->get_logger(), "  target_bottons size: %zu", params_->target_bottons.size());
    RCLCPP_INFO(
        this->get_logger(), "  control_source_switch_key: %s",
        params_->control_source_switch_key.c_str());
    RCLCPP_INFO(this->get_logger(), "  watchdog_timeout: %.2f", params_->watchdog_timeout);
    RCLCPP_INFO(
        this->get_logger(), "  watchdog_enabled: %s",
        params_->watchdog_enabled ? "true" : "false");

    // Check button action parameter lengths.
    if (
        params_->target_bottons.size() != params_->on_state.size() ||
        params_->target_bottons.size() != params_->on_action_topic.size() ||
        params_->target_bottons.size() != params_->on_action_content.size())
    {
        RCLCPP_ERROR(
            this->get_logger(),
            "Parameter error: target_bottons, on_state, on_action_topic, on_action_content "
            "must have the same length.");
        throw std::runtime_error(
                  "Parameter error: target_bottons, on_state, on_action_topic, "
                  "on_action_content must have the same length.");
    }

    for (size_t i = 0; i < params_->target_bottons.size(); ++i) {
        if (REMOTE_CONTROL_BUTTON_MAP.find(params_->target_bottons[i]) == REMOTE_CONTROL_BUTTON_MAP.end()) {
            RCLCPP_ERROR(
                this->get_logger(), "Parameter error: unknown button name %s",
                params_->target_bottons[i].c_str());
            throw std::runtime_error(
                      "Parameter error: unknown button name " + params_->target_bottons[i]);
        }
        if (REMOTE_CONTROL_ACTION_TYPE_MAP.find(params_->on_state[i]) == REMOTE_CONTROL_ACTION_TYPE_MAP.end()) {
            RCLCPP_ERROR(
                this->get_logger(), "Parameter error: unknown action type %s",
                params_->on_state[i].c_str());
            throw std::runtime_error(
                      "Parameter error: unknown action type " + params_->on_state[i]);
        }

        auto button = REMOTE_CONTROL_BUTTON_MAP.at(params_->target_bottons[i]);
        REMOTE_CONTROL_ACTION_TYPE action_type = REMOTE_CONTROL_ACTION_TYPE_MAP.at(params_->on_state[i]);
        std::string topic = params_->on_action_topic[i];
        bool content = params_->on_action_content[i];

        button_actions_.emplace_back(
            std::make_shared<ButtonAction>(this, button, action_type, topic, content));

        RCLCPP_INFO(
            this->get_logger(),
            "Configured ButtonAction: button=%s, action_type=%s, topic=%s, content=%s",
            params_->target_bottons[i].c_str(),
            params_->on_state[i].c_str(),
            topic.c_str(),
            content ? "true" : "false");
    }

    if (REMOTE_CONTROL_BUTTON_MAP.find(params_->control_source_switch_key) == REMOTE_CONTROL_BUTTON_MAP.end()) {
        RCLCPP_ERROR(
            this->get_logger(), "Parameter error: unknown button name %s",
            params_->control_source_switch_key.c_str());
        throw std::runtime_error(
                  "Parameter error: unknown button name " + params_->control_source_switch_key);
    }

    // Keyboard config
    if (
        REMOTE_CONTROL_BUTTON_MAP.find(params_->keyboard_forward_key) == REMOTE_CONTROL_BUTTON_MAP.end() ||
        REMOTE_CONTROL_BUTTON_MAP.find(params_->keyboard_backward_key) == REMOTE_CONTROL_BUTTON_MAP.end() ||
        REMOTE_CONTROL_BUTTON_MAP.find(params_->keyboard_left_key) == REMOTE_CONTROL_BUTTON_MAP.end() ||
        REMOTE_CONTROL_BUTTON_MAP.find(params_->keyboard_right_key) == REMOTE_CONTROL_BUTTON_MAP.end() ||
        REMOTE_CONTROL_BUTTON_MAP.find(params_->keyboard_up_key) == REMOTE_CONTROL_BUTTON_MAP.end() ||
        REMOTE_CONTROL_BUTTON_MAP.find(params_->keyboard_down_key) == REMOTE_CONTROL_BUTTON_MAP.end())
    {
        RCLCPP_ERROR(this->get_logger(), "Parameter error: unknown keyboard button name");
        throw std::runtime_error("Parameter error: unknown keyboard button name");
    }

    keyboard_forward_button_ = REMOTE_CONTROL_BUTTON_MAP.at(params_->keyboard_forward_key);
    keyboard_backward_button_ = REMOTE_CONTROL_BUTTON_MAP.at(params_->keyboard_backward_key);
    keyboard_left_button_ = REMOTE_CONTROL_BUTTON_MAP.at(params_->keyboard_left_key);
    keyboard_right_button_ = REMOTE_CONTROL_BUTTON_MAP.at(params_->keyboard_right_key);
    keyboard_up_button_ = REMOTE_CONTROL_BUTTON_MAP.at(params_->keyboard_up_key);
    keyboard_down_button_ = REMOTE_CONTROL_BUTTON_MAP.at(params_->keyboard_down_key);
    keyboard_up_ratio_ = params_->keyboard_up_ratio;
    keyboard_down_ratio_ = params_->keyboard_down_ratio;

    control_source_switch_button_ = REMOTE_CONTROL_BUTTON_MAP.at(params_->control_source_switch_key);

    initializeControlSources();

    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(params_->cmd_vel_topic, 10);
    initializeBridgeTopics();

    cmd_vel_sub_ = this->create_subscription<rm_message::msg::RemoteControl>(
        params_->remote_controller_topic,
        10,
        std::bind(&RemoteController::cmdVelCallback, this, std::placeholders::_1));
    chasis_enable_pub_ = this->create_publisher<std_msgs::msg::Bool>(params_->chasis_enable_topic, 10);
    arm_enable_pub_ = this->create_publisher<std_msgs::msg::Bool>(params_->arm_enable_topic, 10);

    createControlSourceServices();

    // Initialize button state publishers for all buttons
    for (const auto & [button, name] : REMOTE_CONTROL_BUTTON_NAMES) {
        std::string topic_name = "~/button/" + name;
        button_state_publishers_[button] = this->create_publisher<std_msgs::msg::Bool>(topic_name, 10);
        RCLCPP_DEBUG(this->get_logger(), "Created button state publisher: %s", topic_name.c_str());
    }
    RCLCPP_INFO(
        this->get_logger(), "Initialized %zu button state publishers",
        button_state_publishers_.size());
    // initialize wheel state publishers
    wheel_less_400_pub_ = this->create_publisher<std_msgs::msg::Bool>("~/wheel_less_400", 10);
    wheel_bigger_1600_pub_ = this->create_publisher<std_msgs::msg::Bool>("~/wheel_bigger_1600", 10);

    last_time_ = this->now();

    // Initialize watchdog timer
    last_message_time_ = this->now();
    watchdog_triggered_ = false;

    if (params_->watchdog_enabled) {
        watchdog_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&RemoteController::watchdogCallback, this));
        RCLCPP_INFO(
            this->get_logger(), "Watchdog timer initialized with timeout: %.2f seconds",
            params_->watchdog_timeout);
    }

    RCLCPP_INFO(
        this->get_logger(),
        "RemoteController node initialized. Subscribing to %s, publishing to %s",
        params_->remote_controller_topic.c_str(),
        params_->cmd_vel_topic.c_str());
}

RemoteController::~RemoteController() {}

const std::vector<std::string> & RemoteController::getControlSources() const
{
    return control_sources_;
}

std::string RemoteController::getCurrentControlSourceName() const
{
    return control_sources_.at(current_control_source_index_);
}

size_t RemoteController::getCurrentControlSourceIndex() const
{
    return current_control_source_index_;
}

int RemoteController::findControlSourceIndex(const std::string & source) const
{
    auto it = std::find(control_sources_.begin(), control_sources_.end(), source);
    if (it == control_sources_.end()) {
        return -1;
    }
    return static_cast<int>(std::distance(control_sources_.begin(), it));
}

bool RemoteController::setCurrentControlSourceIndex(size_t index, const std::string & reason)
{
    if (index >= control_sources_.size()) {
        return false;
    }

    const auto & source = control_sources_[index];
    if (current_control_source_index_ == index) {
        RCLCPP_INFO(
            this->get_logger(), "Control source remains %s (reason: %s)", source.c_str(),
            reason.c_str());
        return true;
    }

    current_control_source_index_ = index;
    RCLCPP_INFO(
        this->get_logger(), "Switched to control source: %s (reason: %s)", source.c_str(),
        reason.c_str());
    return true;
}

bool RemoteController::setCurrentControlSourceName(
    const std::string & source,
    const std::string & reason,
    std::string * error_message)
{
    const int index = findControlSourceIndex(source);
    if (index < 0) {
        if (error_message != nullptr) {
            *error_message = "Unknown control source: " + source;
        }
        return false;
    }

    setCurrentControlSourceIndex(static_cast<size_t>(index), reason);
    return true;
}

void RemoteController::cycleToNextControlSource(const std::string & reason)
{
    const size_t next_index = (current_control_source_index_ + 1) % control_sources_.size();
    setCurrentControlSourceIndex(next_index, reason);
}

void RemoteController::markControlSourceActivity()
{
    last_message_time_ = this->now();

    if (watchdog_triggered_) {
        RCLCPP_INFO(this->get_logger(), "Remote control signal recovered. Resuming normal operation.");
        watchdog_triggered_ = false;
    }
}

void RemoteController::initializeControlSources()
{
    control_sources_.clear();
    control_sources_.push_back(kRemoteChannelSource);
    control_sources_.push_back(kKeyboardMouseSource);

    std::unordered_set<std::string> seen_sources(control_sources_.begin(), control_sources_.end());
    for (const auto & bridge_topic : params_->bridge_topics) {
        if (bridge_topic.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Parameter error: bridge_topics contains an empty topic");
            throw std::runtime_error("Parameter error: bridge_topics contains an empty topic");
        }
        if (!seen_sources.insert(bridge_topic).second) {
            RCLCPP_ERROR(
                this->get_logger(), "Parameter error: duplicate control source %s",
                bridge_topic.c_str());
            throw std::runtime_error(
                      "Parameter error: duplicate control source " + bridge_topic);
        }
        control_sources_.push_back(bridge_topic);
    }

    current_control_source_index_ = 0;

    RCLCPP_INFO(
        this->get_logger(), "Total control sources: %zu (Remote + Keyboard + %zu Bridges)",
        control_sources_.size(), params_->bridge_topics.size());
}

void RemoteController::createControlSourceServices()
{
    get_current_chassis_control_source_service_ =
        this->create_service<rm_message::srv::GetCurrentChassisControlSource>(
        "~/get_current_chassis_control_source",
        std::bind(
            &RemoteController::handleGetCurrentChassisControlSource,
            this,
            std::placeholders::_1,
            std::placeholders::_2));

    get_chassis_control_source_list_service_ =
        this->create_service<rm_message::srv::GetChassisControlSourceList>(
        "~/get_chassis_control_source_list",
        std::bind(
            &RemoteController::handleGetChassisControlSourceList,
            this,
            std::placeholders::_1,
            std::placeholders::_2));

    set_current_chassis_control_source_service_ =
        this->create_service<rm_message::srv::SetCurrentChassisControlSource>(
        "~/set_current_chassis_control_source",
        std::bind(
            &RemoteController::handleSetCurrentChassisControlSource,
            this,
            std::placeholders::_1,
            std::placeholders::_2));
}

void RemoteController::handleGetCurrentChassisControlSource(
    const std::shared_ptr<rm_message::srv::GetCurrentChassisControlSource::Request> request,
    std::shared_ptr<rm_message::srv::GetCurrentChassisControlSource::Response> response)
{
    (void)request;
    response->source = getCurrentControlSourceName();
}

void RemoteController::handleGetChassisControlSourceList(
    const std::shared_ptr<rm_message::srv::GetChassisControlSourceList::Request> request,
    std::shared_ptr<rm_message::srv::GetChassisControlSourceList::Response> response)
{
    (void)request;
    response->sources = getControlSources();
}

void RemoteController::handleSetCurrentChassisControlSource(
    const std::shared_ptr<rm_message::srv::SetCurrentChassisControlSource::Request> request,
    std::shared_ptr<rm_message::srv::SetCurrentChassisControlSource::Response> response)
{
    std::string error_message;
    response->success = setCurrentControlSourceName(request->source, "service", &error_message);
    response->current_source = getCurrentControlSourceName();

    if (response->success) {
        response->message = "Current control source: " + response->current_source;
        return;
    }

    response->message = error_message;
}

void RemoteController::cmdVelCallback(const rm_message::msg::RemoteControl::SharedPtr msg)
{
    markControlSourceActivity();

    updateButtonStates(msg);
    publishButtonStates();
    publishWheelState(msg->wheel);

    if (button_release_off_[control_source_switch_button_]) {
        cycleToNextControlSource("button");
    }

    if (getCurrentControlSourceIndex() == 0) {
        sendVel(msg);
    } else if (getCurrentControlSourceIndex() == 1) {
        keyboardSendVel(msg);
    }

    sendEnableChasis(msg);
    sendEnableArm(msg);
}

void RemoteController::sendVel(const rm_message::msg::RemoteControl::SharedPtr msg)
{
    auto twist_msg = geometry_msgs::msg::TwistStamped();
    twist_msg.header.stamp = this->now();
    twist_msg.header.frame_id = "base_link";

    float desired_x = static_cast<double>(msg->chanal2 - 1024) / 660 * params_->max_x;
    float desired_y = -static_cast<double>(msg->chanal3 - 1024) / 660 * params_->max_y;
    float desired_z = -static_cast<double>(msg->chanal0 - 1024) / 660 * params_->max_z;

    rclcpp::Time current_time = this->now();
    double time_diff = (current_time - last_time_).seconds();
    last_time_ = current_time;

    if (std::abs(desired_x) >= 1e-4 && std::abs(desired_x - last_x_) > params_->delta_x * time_diff) {
        desired_x = last_x_ + (desired_x > last_x_ ? params_->delta_x * time_diff : -params_->delta_x * time_diff);
    }
    if (std::abs(desired_y) >= 1e-4 && std::abs(desired_y - last_y_) > params_->delta_y * time_diff) {
        desired_y = last_y_ + (desired_y > last_y_ ? params_->delta_y * time_diff : -params_->delta_y * time_diff);
    }
    if (std::abs(desired_z) >= 1e-4 && std::abs(desired_z - last_z_) > params_->delta_z * time_diff) {
        desired_z = last_z_ + (desired_z > last_z_ ? params_->delta_z * time_diff : -params_->delta_z * time_diff);
    }

    twist_msg.twist.linear.x = desired_x;
    twist_msg.twist.linear.y = desired_y;
    twist_msg.twist.angular.z = desired_z;

    last_x_ = twist_msg.twist.linear.x;
    last_y_ = twist_msg.twist.linear.y;
    last_z_ = twist_msg.twist.angular.z;

    cmd_vel_pub_->publish(twist_msg);
    RCLCPP_DEBUG(
        this->get_logger(), "Published cmd_vel: linear.x=%.3f, angular.z=%.3f",
        twist_msg.twist.linear.x, twist_msg.twist.angular.z);
}

void RemoteController::sendEnableChasis(const rm_message::msg::RemoteControl::SharedPtr msg)
{
    auto enable_msg = std_msgs::msg::Bool();
    enable_msg.data = (std::to_string(msg->cut) == "1" || std::to_string(msg->cut) == "2");
    chasis_enable_pub_->publish(enable_msg);
    RCLCPP_DEBUG(
        this->get_logger(), "Published chasis_enable: %s",
        enable_msg.data ? "true" : "false");
}

void RemoteController::sendEnableArm(const rm_message::msg::RemoteControl::SharedPtr msg)
{
    auto enable_msg = std_msgs::msg::Bool();
    enable_msg.data = (std::to_string(msg->cut) == "2");
    arm_enable_pub_->publish(enable_msg);
    RCLCPP_DEBUG(this->get_logger(), "Published arm_enable: %s", enable_msg.data ? "true" : "false");
}

void RemoteController::publishWheelState(uint16_t wheel_value)
{
    auto less_400_msg = std_msgs::msg::Bool();
    less_400_msg.data = (wheel_value < 400);
    wheel_less_400_pub_->publish(less_400_msg);

    auto bigger_1600_msg = std_msgs::msg::Bool();
    bigger_1600_msg.data = (wheel_value > 1600);
    wheel_bigger_1600_pub_->publish(bigger_1600_msg);

    RCLCPP_DEBUG(
        this->get_logger(), "Published wheel states: less_400=%s, bigger_1600=%s",
        less_400_msg.data ? "true" : "false", bigger_1600_msg.data ? "true" : "false");
}

void RemoteController::updateButtonStates(const rm_message::msg::RemoteControl::SharedPtr msg)
{
    for (auto & pair : button_current_) {
        REMOTE_CONTROL_BUTTON button = pair.first;
        uint8_t current_state = pair.second;
        last_button_states_[button] = current_state;
    }

    button_current_[REMOTE_CONTROL_BUTTON::STOP] = msg->stop;
    button_current_[REMOTE_CONTROL_BUTTON::KEYL] = msg->keyl;
    button_current_[REMOTE_CONTROL_BUTTON::KEYR] = msg->keyr;
    button_current_[REMOTE_CONTROL_BUTTON::KEYB] = msg->keyb;
    button_current_[REMOTE_CONTROL_BUTTON::PRESSL] = msg->pressl;
    button_current_[REMOTE_CONTROL_BUTTON::PRESSR] = msg->pressr;
    button_current_[REMOTE_CONTROL_BUTTON::PRESSMID] = msg->pressmid;
    button_current_[REMOTE_CONTROL_BUTTON::W] = msg->w;
    button_current_[REMOTE_CONTROL_BUTTON::S] = msg->s;
    button_current_[REMOTE_CONTROL_BUTTON::A] = msg->a;
    button_current_[REMOTE_CONTROL_BUTTON::D] = msg->d;
    button_current_[REMOTE_CONTROL_BUTTON::SHIFT] = msg->shift;
    button_current_[REMOTE_CONTROL_BUTTON::CTRL] = msg->ctrl;
    button_current_[REMOTE_CONTROL_BUTTON::Q] = msg->q;
    button_current_[REMOTE_CONTROL_BUTTON::E] = msg->e;
    button_current_[REMOTE_CONTROL_BUTTON::R] = msg->r;
    button_current_[REMOTE_CONTROL_BUTTON::F] = msg->f;
    button_current_[REMOTE_CONTROL_BUTTON::G] = msg->g;
    button_current_[REMOTE_CONTROL_BUTTON::Z] = msg->z;
    button_current_[REMOTE_CONTROL_BUTTON::X] = msg->x;
    button_current_[REMOTE_CONTROL_BUTTON::C] = msg->c;
    button_current_[REMOTE_CONTROL_BUTTON::V] = msg->v;
    button_current_[REMOTE_CONTROL_BUTTON::B] = msg->b;

    for (auto & pair : button_current_) {
        REMOTE_CONTROL_BUTTON button = pair.first;
        uint8_t current_state = pair.second;
        uint8_t last_state = last_button_states_[button];

        if (current_state == 1 && last_state == 0) {
            button_release_off_[button] = true;
        } else {
            button_release_off_[button] = false;
        }
    }

    for (auto & pair : button_current_) {
        REMOTE_CONTROL_BUTTON button = pair.first;
        uint8_t current_state = pair.second;
        uint8_t last_state = last_button_states_[button];

        if (current_state == 0 && last_state == 1) {
            button_pressed_off_[button] = true;
        } else {
            button_pressed_off_[button] = false;
        }
    }

    for (auto & pair : button_release_off_) {
        REMOTE_CONTROL_BUTTON button = pair.first;
        if (pair.second) {
            button_toggled_[button] = !button_toggled_[button];
        }
    }

    for (const auto & action : button_actions_) {
        std::vector<REMOTE_CONTROL_ACTION_TYPE> trigger_types;
        auto button = action->get_button();
        if (button_release_off_[button]) {
            trigger_types.push_back(REMOTE_CONTROL_ACTION_TYPE::ON_REALSED_OFF);
        }
        if (button_pressed_off_[button]) {
            trigger_types.push_back(REMOTE_CONTROL_ACTION_TYPE::ON_PRESSED_OFF);
        }
        if (button_current_[button] == true) {
            trigger_types.push_back(REMOTE_CONTROL_ACTION_TYPE::ON_PRESSED);
        }
        if (button_current_[button] == false) {
            trigger_types.push_back(REMOTE_CONTROL_ACTION_TYPE::ON_REALSED);
        }
        if (button_toggled_[button]) {
            trigger_types.push_back(REMOTE_CONTROL_ACTION_TYPE::ON_TOGGLED);
        }

        action->execute(trigger_types);
    }
}

void RemoteController::publishButtonStates()
{
    for (const auto & [button, pressed] : button_current_) {
        auto msg = std_msgs::msg::Bool();
        msg.data = pressed;
        button_state_publishers_[button]->publish(msg);
    }
}

void RemoteController::keyboardSendVel(const rm_message::msg::RemoteControl::SharedPtr msg)
{
    auto twist_msg = geometry_msgs::msg::TwistStamped();
    twist_msg.header.stamp = this->now();
    twist_msg.header.frame_id = "base_link";

    float desired_x = button_current_[keyboard_forward_button_] ? params_->max_x :
        button_current_[keyboard_backward_button_] ? -params_->max_x : 0.0f;
    float desired_y = button_current_[keyboard_left_button_] ? params_->max_y :
        button_current_[keyboard_right_button_] ? -params_->max_y : 0.0f;
    float desired_z = std::clamp(msg->mousex * params_->keyboard_mousex_scale, -params_->max_z, params_->max_z);

    if (button_current_[keyboard_up_button_]) {
        desired_x *= keyboard_up_ratio_;
        desired_y *= keyboard_up_ratio_;
        desired_z *= keyboard_up_ratio_;
    } else if (button_current_[keyboard_down_button_]) {
        desired_x *= keyboard_down_ratio_;
        desired_y *= keyboard_down_ratio_;
        desired_z *= keyboard_down_ratio_;
    }

    rclcpp::Time current_time = this->now();
    double time_diff = (current_time - last_time_).seconds();
    last_time_ = current_time;

    if (std::abs(desired_x) >= 1e-4 && std::abs(desired_x - last_x_) > params_->delta_x * time_diff) {
        desired_x = last_x_ + (desired_x > last_x_ ? params_->delta_x * time_diff : -params_->delta_x * time_diff);
    }
    if (std::abs(desired_y) >= 1e-4 && std::abs(desired_y - last_y_) > params_->delta_y * time_diff) {
        desired_y = last_y_ + (desired_y > last_y_ ? params_->delta_y * time_diff : -params_->delta_y * time_diff);
    }
    if (std::abs(desired_z) >= 1e-4 && std::abs(desired_z - last_z_) > params_->delta_z * time_diff) {
        desired_z = last_z_ + (desired_z > last_z_ ? params_->delta_z * time_diff : -params_->delta_z * time_diff);
    }

    twist_msg.twist.linear.x = desired_x;
    twist_msg.twist.linear.y = desired_y;
    twist_msg.twist.angular.z = desired_z;

    last_x_ = twist_msg.twist.linear.x;
    last_y_ = twist_msg.twist.linear.y;
    last_z_ = twist_msg.twist.angular.z;

    if (button_toggled_[REMOTE_CONTROL_BUTTON::KEYB]) {
        return;
    }

    cmd_vel_pub_->publish(twist_msg);
    RCLCPP_DEBUG(
        this->get_logger(), "Published cmd_vel: linear.x=%.3f, angular.z=%.3f",
        twist_msg.twist.linear.x, twist_msg.twist.angular.z);
}

ButtonAction::ButtonAction(
    rclcpp::Node * node,
    REMOTE_CONTROL_BUTTON button,
    REMOTE_CONTROL_ACTION_TYPE action_type,
    std::string topic,
    bool content)
: action_type_(action_type), button_(button), content_(content)
{
    publisher_ = node->create_publisher<std_msgs::msg::Bool>(topic, 10);
}

void ButtonAction::execute(const std::vector<REMOTE_CONTROL_ACTION_TYPE> & trigger_types)
{
    if (std::find(trigger_types.begin(), trigger_types.end(), action_type_) != trigger_types.end()) {
        auto msg = std_msgs::msg::Bool();
        msg.data = content_;
        publisher_->publish(msg);
    }
}

void RemoteController::watchdogCallback()
{
    if (!params_->watchdog_enabled) {
        return;
    }

    rclcpp::Time current_time = this->now();
    double time_since_last_msg = (current_time - last_message_time_).seconds();

    if (time_since_last_msg > params_->watchdog_timeout && !watchdog_triggered_) {
        RCLCPP_WARN(
            this->get_logger(),
            "Watchdog timeout! No remote control message received for %.2f seconds. "
            "Sending disable signals.",
            time_since_last_msg);

        auto disable_msg = std_msgs::msg::Bool();
        disable_msg.data = false;
        chasis_enable_pub_->publish(disable_msg);
        arm_enable_pub_->publish(disable_msg);

        auto zero_twist = geometry_msgs::msg::TwistStamped();
        zero_twist.header.stamp = current_time;
        zero_twist.header.frame_id = "base_link";
        zero_twist.twist.linear.x = 0.0;
        zero_twist.twist.linear.y = 0.0;
        zero_twist.twist.angular.z = 0.0;
        cmd_vel_pub_->publish(zero_twist);

        last_x_ = 0.0;
        last_y_ = 0.0;
        last_z_ = 0.0;

        watchdog_triggered_ = true;
    }
}

void RemoteController::initializeBridgeTopics()
{
    for (size_t i = 0; i < params_->bridge_topics.size(); ++i) {
        auto sub = this->create_subscription<geometry_msgs::msg::TwistStamped>(
            params_->bridge_topics[i],
            10,
            [this, i](const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
                const size_t expected_index = 2 + i;
                if (current_control_source_index_ != expected_index) {
                    return;
                }

                markControlSourceActivity();
                sendBridgeVel(msg, i);
            });
        bridge_vel_subs_.push_back(sub);

        RCLCPP_INFO(
            this->get_logger(), "Subscribed to bridge topic[%zu]: %s",
            i, params_->bridge_topics[i].c_str());
    }
}

void RemoteController::sendBridgeVel(
    const geometry_msgs::msg::TwistStamped::SharedPtr msg,
    size_t bridge_index)
{
    auto twist_msg = geometry_msgs::msg::TwistStamped();
    twist_msg.header.stamp = this->now();
    twist_msg.header.frame_id = "base_link";

    float desired_x = msg->twist.linear.x;
    float desired_y = msg->twist.linear.y;
    float desired_z = msg->twist.angular.z;

    desired_x = std::clamp(desired_x, static_cast<float>(-params_->max_x), static_cast<float>(params_->max_x));
    desired_y = std::clamp(desired_y, static_cast<float>(-params_->max_y), static_cast<float>(params_->max_y));
    desired_z = std::clamp(desired_z, static_cast<float>(-params_->max_z), static_cast<float>(params_->max_z));

    rclcpp::Time current_time = this->now();
    double time_diff = (current_time - last_time_).seconds();
    last_time_ = current_time;

    if (std::abs(desired_x) >= 1e-4 && std::abs(desired_x - last_x_) > params_->delta_x * time_diff) {
        desired_x = last_x_ + (desired_x > last_x_ ? params_->delta_x * time_diff : -params_->delta_x * time_diff);
    }
    if (std::abs(desired_y) >= 1e-4 && std::abs(desired_y - last_y_) > params_->delta_y * time_diff) {
        desired_y = last_y_ + (desired_y > last_y_ ? params_->delta_y * time_diff : -params_->delta_y * time_diff);
    }
    if (std::abs(desired_z) >= 1e-4 && std::abs(desired_z - last_z_) > params_->delta_z * time_diff) {
        desired_z = last_z_ + (desired_z > last_z_ ? params_->delta_z * time_diff : -params_->delta_z * time_diff);
    }

    twist_msg.twist.linear.x = desired_x;
    twist_msg.twist.linear.y = desired_y;
    twist_msg.twist.angular.z = desired_z;

    last_x_ = twist_msg.twist.linear.x;
    last_y_ = twist_msg.twist.linear.y;
    last_z_ = twist_msg.twist.angular.z;

    cmd_vel_pub_->publish(twist_msg);
    RCLCPP_DEBUG(
        this->get_logger(),
        "Published bridge cmd_vel[%zu]: linear.x=%.3f, linear.y=%.3f, angular.z=%.3f",
        bridge_index,
        twist_msg.twist.linear.x,
        twist_msg.twist.linear.y,
        twist_msg.twist.angular.z);
}

}  // namespace RM_REMOTE_CONTROLLER
