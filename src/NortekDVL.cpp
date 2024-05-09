#include "dvl_port_manager/NortekDVL.hpp"
#include "dvl_port_manager/DVLDataFormat.hpp"

using std::placeholders::_1;

namespace dvl_port_manager
{
    NortekDVL::NortekDVL()
        : DVLProvider("192.168.0.12", 9002, 0, sizeof(NortekFormat_t)), _depthOffset{}
    {
        _publisherSpeed = this->create_publisher<sonia_common_ros2::msg::BodyVelocityDVL>("/provider_dvl/dvl_velocity", 10);
        _publisherFluidPressure = this->create_publisher<sensor_msgs::msg::FluidPressure>("/provider_dvl/dvl_pressure", 10);
        _publisherTemperature = this->create_publisher<sensor_msgs::msg::Temperature>("/provider_dvl/dvl_water_temperature", 10);
        _publisherRelativeDepth = this->create_publisher<std_msgs::msg::Float32>("/provider_depth/depth", 10);
        _subscriptionSetDepthOffset = this->create_subscription<std_msgs::msg::Empty>("/provider_dvl/setDepthOffset", 10, std::bind(&NortekDVL::_setDepthOffsetCallback, this, _1));
    }

    void NortekDVL::receiveDataThread()
    {
        rclcpp::Rate rate(GetSpinRate());
        while (rclcpp::ok())
        {
            _socket.ReceiveTCP();
            RCLCPP_DEBUG(this->get_logger(), "Data Received");

            getData<NortekFormat_t>(_dvlData);

            if (_dvlData.header.sync == _NORTEK_ID)
            {
                rclcpp::Time timestamp = this->now();
                _fillTwistMessage(timestamp);
                _fillFluidPresureMessage(timestamp);
                _fillTemperatureMessage(timestamp);
                _fillRelativeDepthMessage();
            }
            else
            {
                RCLCPP_WARN(this->get_logger(), "Nortek ID mismatch : %d != %u", _NORTEK_ID, _dvlData.header.sync);
            }
            rate.sleep();
        }
    }

    void NortekDVL::_setDepthOffsetCallback(const std_msgs::msg::Empty &msg)
    {
        _depthOffset = _dvlData.data.pressure;
    }

    void NortekDVL::_fillTwistMessage(rclcpp::Time timestamp)
    {
        sonia_common_ros2::msg::BodyVelocityDVL message;
        message.header.stamp = timestamp;
        message.header.frame_id = "/ENU";

        if (_dvlData.data.status.bit_field.xVelValid)
        {
            message.x_vel_btm = (double)_dvlData.data.velX;
        }
        else
        {
            message.x_vel_btm = 0.0;
        }
        if (_dvlData.data.status.bit_field.yVelValid)
        {
            message.y_vel_btm = (double)_dvlData.data.velY;
        }
        else
        {
            message.y_vel_btm = 0.0;
        }
        if (_dvlData.data.status.bit_field.z1VelValid)
        {
            message.z_vel_btm = (double)_dvlData.data.velZ1;
        }
        else
        {
            message.z_vel_btm = 0.0;
        }

        _publisherSpeed->publish(message);
    }

    void NortekDVL::_fillFluidPresureMessage(rclcpp::Time timestamp)
    {
        sensor_msgs::msg::FluidPressure message;

        message.header.stamp = timestamp;
        message.header.frame_id = "/ENU";

        message.fluid_pressure = _dvlData.data.pressure;

        _publisherFluidPressure->publish(message);
    }

    void NortekDVL::_fillRelativeDepthMessage()
    {
        std_msgs::msg::Float32 message;

        message.data = _convertDBarToMeters(_dvlData.data.pressure - _depthOffset);
        _publisherRelativeDepth->publish(message);
    }

    void NortekDVL::_fillTemperatureMessage(rclcpp::Time timestamp)
    {
        sensor_msgs::msg::Temperature message;

        message.header.stamp = timestamp;
        message.header.frame_id = "/ENU";

        message.temperature = _dvlData.data.temperature;

        _publisherTemperature->publish(message);
    }

    float NortekDVL::_convertDBarToMeters(float dBarValue)
    {
        return dBarValue * dBAR_TO_METER_OF_WATER;
    }

} // namespace dvl_port_manager
