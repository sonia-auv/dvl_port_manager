#pragma once

#include <std_srvs/srv/trigger.hpp>
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
        void tare(const std::shared_ptr<std_srvs::srv::Trigger::Request> request, std::shared_ptr<std_srvs::srv::Trigger::Response> response);

        void _fillTwistMessage(rclcpp::Time timestamp);
        void _fillFluidPresureMessage(rclcpp::Time timestamp);
        void _fillRelativeDepthMessage();
        void _fillTemperatureMessage(rclcpp::Time timestamp);

        float _convertDBarToMeters(float dBarValue);

        const float dBAR_TO_METER_OF_WATER = 10.1972f;
        const uint8_t _NORTEK_ID = 0xA5;

        rclcpp::Publisher<sonia_common_ros2::msg::BodyVelocityDVL>::SharedPtr _publisherSpeed;
        rclcpp::Publisher<sensor_msgs::msg::FluidPressure>::SharedPtr _publisherFluidPressure;
        rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr _publisherTemperature;
        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr _publisherRelativeDepth;

        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr _tare_srv;

        float _depthOffset;
        NortekFormat_t _dvlData;
    };
}
