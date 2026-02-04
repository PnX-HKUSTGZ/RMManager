#include "rm_button_action/rm_button_action.hpp"

#include <stdexcept>

namespace rm_button_action
{

RmButtonAction::RmButtonAction(const rclcpp::NodeOptions & options)
: Node("rm_button_action", options)
{
    // 获取参数
    param_listener_ = std::make_shared<ParamListener>(this->get_node_parameters_interface());
    params_ = param_listener_->get_params();

    input_topic_ = params_.input_topic;

    press_state_action_ = build_action(
        params_.press_state_action_topic,
        params_.press_state_action_type,
        params_.press_state_action_bool_value,
        params_.press_state_action_float_value);

    release_state_action_ = build_action(
        params_.release_state_action_topic,
        params_.release_state_action_type,
        params_.release_state_action_bool_value,
        params_.release_state_action_float_value);

    press_edge_actions_ = build_action_list(
        params_.press_edge_action_topics,
        params_.press_edge_action_types,
        params_.press_edge_action_bool_values,
        params_.press_edge_action_float_values,
        "press_edge_action");

    release_edge_actions_ = build_action_list(
        params_.release_edge_action_topics,
        params_.release_edge_action_types,
        params_.release_edge_action_bool_values,
        params_.release_edge_action_float_values,
        "release_edge_action");

    if (!press_edge_actions_.empty()) {
        if (params_.press_edge_start_index < 0 ||
            static_cast<size_t>(params_.press_edge_start_index) >= press_edge_actions_.size()) {
            RCLCPP_ERROR(this->get_logger(),
                "press_edge_start_index out of range: %lld (size=%zu)",
                static_cast<long long>(params_.press_edge_start_index),
                press_edge_actions_.size());
            throw std::runtime_error("press_edge_start_index out of range");
        }
        press_edge_index_ = static_cast<size_t>(params_.press_edge_start_index);
    }

    if (!release_edge_actions_.empty()) {
        if (params_.release_edge_start_index < 0 ||
            static_cast<size_t>(params_.release_edge_start_index) >= release_edge_actions_.size()) {
            RCLCPP_ERROR(this->get_logger(),
                "release_edge_start_index out of range: %lld (size=%zu)",
                static_cast<long long>(params_.release_edge_start_index),
                release_edge_actions_.size());
            throw std::runtime_error("release_edge_start_index out of range");
        }
        release_edge_index_ = static_cast<size_t>(params_.release_edge_start_index);
    }

    // 输出参数
    RCLCPP_INFO(this->get_logger(), "Input topic: %s", input_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "Press state action: type=%s, topic=%s", 
        action_type_name(press_state_action_.type), press_state_action_.topic.c_str());
    RCLCPP_INFO(this->get_logger(), "Release state action: type=%s, topic=%s", 
        action_type_name(release_state_action_.type), release_state_action_.topic.c_str());
    RCLCPP_INFO(this->get_logger(), "Press edge actions: %zu, start_index=%zu",
        press_edge_actions_.size(), press_edge_index_);
    RCLCPP_INFO(this->get_logger(), "Release edge actions: %zu, start_index=%zu",
        release_edge_actions_.size(), release_edge_index_);

    input_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        input_topic_, 10,
        std::bind(&RmButtonAction::input_callback, this, std::placeholders::_1)
    );

    RCLCPP_INFO(this->get_logger(), "RmButtonAction node initialized");
}

RmButtonAction::ActionType RmButtonAction::parse_action_type(const std::string & type) const
{
    if (type == "none") {
        return ActionType::NONE;
    }
    if (type == "bool") {
        return ActionType::BOOL;
    }
    if (type == "float32") {
        return ActionType::FLOAT32;
    }
    if (type == "float64") {
        return ActionType::FLOAT64;
    }

    RCLCPP_ERROR(this->get_logger(), "Unknown action type: %s", type.c_str());
    throw std::runtime_error("Unknown action type: " + type);
}

const char * RmButtonAction::action_type_name(ActionType type) const
{
    switch (type) {
        case ActionType::NONE:
            return "none";
        case ActionType::BOOL:
            return "bool";
        case ActionType::FLOAT32:
            return "float32";
        case ActionType::FLOAT64:
            return "float64";
        default:
            return "unknown";
    }
}

RmButtonAction::Action RmButtonAction::build_action(
    const std::string & topic,
    const std::string & type,
    bool bool_value,
    double float_value)
{
    Action action;
    action.topic = topic;
    action.type = parse_action_type(type);
    action.bool_value = bool_value;
    action.float_value = float_value;

    if (!action.enabled()) {
        return action;
    }

    if (action.type == ActionType::BOOL) {
        action.bool_pub = this->create_publisher<std_msgs::msg::Bool>(topic, 10);
    } else if (action.type == ActionType::FLOAT32) {
        action.float32_pub = this->create_publisher<std_msgs::msg::Float32>(topic, 10);
    } else if (action.type == ActionType::FLOAT64) {
        action.float64_pub = this->create_publisher<std_msgs::msg::Float64>(topic, 10);
    }

    return action;
}

std::vector<RmButtonAction::Action> RmButtonAction::build_action_list(
    const std::vector<std::string> & topics,
    const std::vector<std::string> & types,
    const std::vector<bool> & bool_values,
    const std::vector<double> & float_values,
    const std::string & list_name)
{
    if (topics.size() != types.size() ||
        topics.size() != bool_values.size() ||
        topics.size() != float_values.size()) {
        RCLCPP_ERROR(this->get_logger(),
            "%s config size mismatch: topics=%zu, types=%zu, bool_values=%zu, float_values=%zu",
            list_name.c_str(), topics.size(), types.size(), bool_values.size(), float_values.size());
        throw std::runtime_error(list_name + " config size mismatch");
    }

    std::vector<Action> actions;
    actions.reserve(topics.size());

    for (size_t i = 0; i < topics.size(); ++i) {
        actions.emplace_back(build_action(topics[i], types[i], bool_values[i], float_values[i]));
    }

    return actions;
}

void RmButtonAction::execute_action(const Action & action)
{
    if (!action.enabled()) {
        return;
    }

    if (action.type == ActionType::BOOL) {
        std_msgs::msg::Bool msg;
        msg.data = action.bool_value;
        action.bool_pub->publish(msg);
        return;
    }

    if (action.type == ActionType::FLOAT32) {
        std_msgs::msg::Float32 msg;
        msg.data = static_cast<float>(action.float_value);
        action.float32_pub->publish(msg);
        return;
    }

    if (action.type == ActionType::FLOAT64) {
        std_msgs::msg::Float64 msg;
        msg.data = action.float_value;
        action.float64_pub->publish(msg);
        return;
    }
}

void RmButtonAction::execute_trigger_action(
    const std::vector<Action> & actions,
    size_t & index,
    const std::string & trigger_name)
{
    if (actions.empty()) {
        return;
    }

    if (index >= actions.size()) {
        RCLCPP_WARN(this->get_logger(), "%s index out of range: %zu", trigger_name.c_str(), index);
        index = 0;
    }

    execute_action(actions[index]);
    index = (index + 1) % actions.size();
}

void RmButtonAction::input_callback(const std_msgs::msg::Bool::SharedPtr msg)
{
    if (msg == nullptr) {
        RCLCPP_WARN(this->get_logger(), "Received null message on input_topic");
        return;
    }

    const bool current_state = msg->data;

    if (current_state) {
        execute_action(press_state_action_);
    } else {
        execute_action(release_state_action_);
    }

    if (!have_last_state_) {
        last_state_ = current_state;
        have_last_state_ = true;
        return;
    }

    if (current_state && !last_state_) {
        execute_trigger_action(press_edge_actions_, press_edge_index_, "press_edge");
    } else if (!current_state && last_state_) {
        execute_trigger_action(release_edge_actions_, release_edge_index_, "release_edge");
    }

    last_state_ = current_state;
}

} // namespace rm_button_action

// 注册为 component
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(rm_button_action::RmButtonAction)
