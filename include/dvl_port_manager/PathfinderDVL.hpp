#pragma once
#include "dvl_port_manager/DVLProvider.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sonia_common_ros2/msg/body_velocity_dvl.hpp"
#include "std_msgs/msg/bool.hpp"
#include "dvl_port_manager/DVLDataFormat.hpp"

namespace dvl_port_manager
{
    class PathfinderDVL final : public DVLProvider
    {
    public:
        PathfinderDVL();
        ~PathfinderDVL(){};

    protected:
        void receiveDataThread() override;

    private:

        void _enableDisableDVL(const std_msgs::msg::Bool &msg);

        template <class T>
        uint16_t _calculateCheckSum(uint8_t *dvlData);

        std::string _START_STOP_CMD = "===\n";
        std::string _START_DATA_CMD = "CS\n";
        uint8_t _PATHFINDER_ID = 0x7D;

        rclcpp::Publisher<sonia_common_ros2::msg::BodyVelocityDVL>::SharedPtr _publisherBodyVelocity;
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr _publisherLeakSensor;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr _subscriptionEnableDisableDVL;

        PathfinderFormat_t _dvlData;
    };
}
