#include "remote_controller/remote_controller.hpp"

namespace RM_REMOTE_CONTROLLER {

RemoteController::RemoteController(std::string name) : rclcpp::Node(name) {
    // Initialize parameter listener and get parameters
    param_listener_ = std::make_shared<remote_controller::ParamListener>(this->get_node_parameters_interface());
    params_ = std::make_shared<remote_controller::Params>(param_listener_->get_params());

    // Output parameter values
    RCLCPP_INFO(this->get_logger(), "Parameters:");
    RCLCPP_INFO(this->get_logger(), "  cmd_vel_topic: %s", params_->cmd_vel_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "  remote_controller_topic: %s", params_->remote_controller_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "  chasis_enable_topic: %s", params_->chasis_enable_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "  arm_enable_topic: %s", params_->arm_enable_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "  max_x: %.2f", params_->max_x);
    RCLCPP_INFO(this->get_logger(), "  max_y: %.2f", params_->max_y);
    RCLCPP_INFO(this->get_logger(), "  max_z: %.2f", params_->max_z);
    RCLCPP_INFO(this->get_logger(), "  delta_x: %.2f", params_->delta_x);
    RCLCPP_INFO(this->get_logger(), "  delta_y: %.2f", params_->delta_y);
    RCLCPP_INFO(this->get_logger(), "  delta_z: %.2f", params_->delta_z);

    // config button actions
    
    // 检查 长度
    if(params_->target_bottons.size() != params_->on_state.size() ||
       params_->target_bottons.size() != params_->on_action_topic.size() ||
       params_->target_bottons.size() != params_->on_action_content.size()) {
        RCLCPP_ERROR(this->get_logger(), "Parameter error: target_bottons, on_state, on_action_topic, on_action_content must have the same length.");
        throw std::runtime_error("Parameter error: target_bottons, on_state, on_action_topic, on_action_content must have the same length.");
    }

    for(size_t i = 0; i < params_->target_bottons.size(); ++i) {
        if(REMOTE_CONTROL_BUTTON_MAP.find(params_->target_bottons[i]) == REMOTE_CONTROL_BUTTON_MAP.end()) {
            RCLCPP_ERROR(this->get_logger(), "Parameter error: unknown button name %s", params_->target_bottons[i].c_str());
            throw std::runtime_error("Parameter error: unknown button name " + params_->target_bottons[i]);
        }
        if(REMOTE_CONTROL_ACTION_TYPE_MAP.find(params_->on_state[i]) == REMOTE_CONTROL_ACTION_TYPE_MAP.end()) {
            RCLCPP_ERROR(this->get_logger(), "Parameter error: unknown action type %s", params_->on_state[i].c_str());
            throw std::runtime_error("Parameter error: unknown action type " + params_->on_state[i]);
        }

        auto button = REMOTE_CONTROL_BUTTON_MAP.at(params_->target_bottons[i]);
        REMOTE_CONTROL_ACTION_TYPE action_type = REMOTE_CONTROL_ACTION_TYPE_MAP.at(params_->on_state[i]);
        std::string topic = params_->on_action_topic[i];
        bool content = params_->on_action_content[i];

        button_actions_.emplace_back(std::make_shared<ButtonAction>(this, button, action_type, topic, content));

        // 输出
        RCLCPP_INFO(this->get_logger(), "Configured ButtonAction: button=%s, action_type=%s, topic=%s, content=%s",
            params_->target_bottons[i].c_str(),
            params_->on_state[i].c_str(),
            topic.c_str(),
            content ? "true" : "false");
    }

    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(params_->cmd_vel_topic, 10);
    cmd_vel_sub_ = this->create_subscription<rm_message::msg::RemoteControl>(
        params_->remote_controller_topic, 10,
        std::bind(&RemoteController::cmdVelCallback, this, std::placeholders::_1)
    );
    chasis_enable_pub_ = this->create_publisher<std_msgs::msg::Bool>(params_->chasis_enable_topic, 10);
    arm_enable_pub_ = this->create_publisher<std_msgs::msg::Bool>(params_->arm_enable_topic, 10);

    last_time_ = this->now();

    RCLCPP_INFO(this->get_logger(), "RemoteController node initialized. Subscribing to %s, publishing to %s", 
                params_->remote_controller_topic.c_str(), params_->cmd_vel_topic.c_str());
}

RemoteController::~RemoteController() {}

void RemoteController::cmdVelCallback(const rm_message::msg::RemoteControl::SharedPtr msg) {
    updateButtonStates(msg);
    sendVel(msg);
    sendEnableChasis(msg);
    sendEnableArm(msg);
}

void RemoteController::sendVel(const rm_message::msg::RemoteControl::SharedPtr msg) {

    auto twist_msg = geometry_msgs::msg::TwistStamped();
    twist_msg.header.stamp = this->now();
    twist_msg.header.frame_id = "base_link";

    // 限制加速度
    float desired_x = static_cast<double>(msg->chanal2 - 1024) / 660 * params_->max_x;
    float desired_y = -static_cast<double>(msg->chanal3 - 1024) / 660 * params_->max_y;
    float desired_z = -static_cast<double>(msg->chanal0 - 1024) / 660 * params_->max_z;

    rclcpp::Time current_time = this->now();
    double time_diff = (current_time - last_time_).seconds();
    last_time_ = current_time;

    // 限制加速度 允许急停
    if (std::abs(desired_x) >= 1e-4 && std::abs(desired_x - last_x_) > params_->delta_x*time_diff) {
        desired_x = last_x_ + (desired_x > last_x_ ? params_->delta_x*time_diff : -params_->delta_x*time_diff);
    }
    if (std::abs(desired_y) >= 1e-4 && std::abs(desired_y - last_y_) > params_->delta_y*time_diff) {
        desired_y = last_y_ + (desired_y > last_y_ ? params_->delta_y*time_diff : -params_->delta_y*time_diff);
    }
    if (std::abs(desired_z) >= 1e-4 && std::abs(desired_z - last_z_) > params_->delta_z*time_diff) {
        desired_z = last_z_ + (desired_z > last_z_ ? params_->delta_z*time_diff : -params_->delta_z*time_diff);
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
    RCLCPP_DEBUG(this->get_logger(), "Published cmd_vel: linear.x=%.3f, angular.z=%.3f", twist_msg.twist.linear.x, twist_msg.twist.angular.z);
}

void RemoteController::sendEnableChasis(const rm_message::msg::RemoteControl::SharedPtr msg) {
    auto enable_msg = std_msgs::msg::Bool();
    enable_msg.data = (std::to_string(msg->cut) == "1" || std::to_string(msg->cut) == "2");
    chasis_enable_pub_->publish(enable_msg);
    RCLCPP_DEBUG(this->get_logger(), "Published chasis_enable: %s", enable_msg.data ? "true" : "false");
}

void RemoteController::sendEnableArm(const rm_message::msg::RemoteControl::SharedPtr msg) {
    auto enable_msg = std_msgs::msg::Bool();
    enable_msg.data = (std::to_string(msg->cut) == "2");
    arm_enable_pub_->publish(enable_msg);
    RCLCPP_DEBUG(this->get_logger(), "Published arm_enable: %s", enable_msg.data ? "true" : "false");
}

void RemoteController::updateButtonStates(const rm_message::msg::RemoteControl::SharedPtr msg) {

    // 更新按钮状态的逻辑实现
    for(auto &pair : button_current_) {
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


    // 检查某个按钮是否被刚好被按下的逻辑实现
    for(auto &pair : button_current_) {
        REMOTE_CONTROL_BUTTON button = pair.first;
        uint8_t current_state = pair.second;
        uint8_t last_state = last_button_states_[button];

        // 按钮被按下
        if (current_state == 1 && last_state == 0) {
            button_release_off_[button] = true;
        } else {
            button_release_off_[button] = false;
        }
    }

    // button_pressed_off_
    for(auto &pair : button_current_) {
        REMOTE_CONTROL_BUTTON button = pair.first;
        uint8_t current_state = pair.second;
        uint8_t last_state = last_button_states_[button];

        // 按钮被释放
        if (current_state == 0 && last_state == 1) {
            button_pressed_off_[button] = true;
        } else {
            button_pressed_off_[button] = false;
        }
    }

    // 更新button_toggled_
    for (auto &pair : button_release_off_) {
        REMOTE_CONTROL_BUTTON button = pair.first;
        if (pair.second) { // 如果按钮被按下
            button_toggled_[button] = !button_toggled_[button]; // 切换状态
        }
    }

    for(const auto& action : button_actions_) {
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

ButtonAction::ButtonAction(
    rclcpp::Node * node,
    REMOTE_CONTROL_BUTTON button,
    REMOTE_CONTROL_ACTION_TYPE action_type, 
    std::string topic, 
    bool content) 
    : action_type_(action_type), button_(button), content_(content) {
    publisher_ = node->create_publisher<std_msgs::msg::Bool>(topic, 10);
    }

void ButtonAction::execute(const std::vector<REMOTE_CONTROL_ACTION_TYPE>& trigger_types) {
    if (std::find(trigger_types.begin(), trigger_types.end(), action_type_) != trigger_types.end()) {
        auto msg = std_msgs::msg::Bool();
        msg.data = content_;
        publisher_->publish(msg);
    }
}

} // namespace RM_REMOTE_CONTROLLER