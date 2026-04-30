#include "rm_ui/ui_debugger.hpp"

#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rm_ui::RmUiDebugger>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
