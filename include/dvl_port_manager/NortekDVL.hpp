#pragma once

#include "dvl_port_manager/DVLProvider.hpp"
#include "dvl_port_manager/DVLDataFormat.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sonia_common_ros2/msg/body_velocity_dvl.hpp"
#include "sensor_msgs/msg/fluid_pressure.hpp"
#include "sensor_msgs/msg/temperature.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/empty.hpp"

namespace dvl_port_manager
{
    class NortekDVL final : public DVLProvider
    {
    public:
        NortekDVL();
        ~NortekDVL(){};

    protected:
        void receiveDataThread() override;

    private:
        void _setDepthOffsetCallback(const std_msgs::msg::Empty &msg);

        void _fillTwistMessage(rclcpp::Time timestamp);
        void _fillFluidPresureMessage(rclcpp::Time timestamp);
        void _fillRelativeDepthMessage();
        void _fillTemperatureMessage(rclcpp::Time timestamp);

        float _convertDBarToMeters(float dBarValue);

        const float dBAR_TO_METER_OF_WATER = 10.1972f;
        const std::string _NORTEK_IP_ADDR = "192.168.0.12";
        const uint8_t _NORTEK_ID = 0xA5;

        rclcpp::Publisher<sonia_common_ros2::msg::BodyVelocityDVL>::SharedPtr _publisherSpeed;
        rclcpp::Publisher<sensor_msgs::msg::FluidPressure>::SharedPtr _publisherFluidPressure;
        rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr _publisherTemperature;
        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr _publisherRelativeDepth;
        rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr _subscriptionSetDepthOffset;

        float _depthOffset;
        NortekFormat_t _dvlData;
    };
}