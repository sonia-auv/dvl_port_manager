#include "rclcpp/rclcpp.hpp"
#include "dvl_port_manager/PathfinderDVL.hpp"
#include "dvl_port_manager/NortekDVL.hpp"
#include "dvl_port_manager/DVLProvider.hpp"
#include <cstdlib>

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    const char *env_var = std::getenv("AUV");
    if (env_var == nullptr)
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "AUV ENV Variable not set!!!");
        rclcpp::shutdown();
        return EXIT_FAILURE;
    }
    std::string auv = env_var;
    std::shared_ptr<dvl_port_manager::DVLProvider> dvl_node;
    if (auv == "AUV8")
    {
        dvl_node = std::make_shared<dvl_port_manager::PathfinderDVL>();
    }
    else if (auv == "AUV7")
    {
        dvl_node = std::make_shared<dvl_port_manager::NortekDVL>();
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "AUV ENV Variable Bad!!!");
        rclcpp::shutdown();
        return EXIT_FAILURE;
    }

    rclcpp::spin(dvl_node);

    rclcpp::shutdown();
    return EXIT_SUCCESS;
}
