#include "rclcpp/rclcpp.hpp"
#include "dvl_port_manager/PathfinderDVL.hpp"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    auto dvl_node = std::make_shared<dvl_port_manager::PathfinderDVL>();

    rclcpp::spin(dvl_node);

    rclcpp::shutdown();
    return EXIT_SUCCESS;
}
