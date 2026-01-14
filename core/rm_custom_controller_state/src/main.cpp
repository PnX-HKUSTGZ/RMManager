#include "rm_custom_controller_state/rm_custom_controller_state.hpp"

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rm_custom_controller_state::RmCustomControllerState>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}