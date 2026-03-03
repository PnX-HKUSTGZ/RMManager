#include "rclcpp/rclcpp.hpp"
#include "rm_custom_controller_imu/rm_custom_controller_imu.hpp"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    
    rclcpp::NodeOptions options;
    auto node = std::make_shared<rm_custom_controller_imu::RmCustomControllerImu>(options);
    
    rclcpp::spin(node);
    
    rclcpp::shutdown();
    return 0;
}
