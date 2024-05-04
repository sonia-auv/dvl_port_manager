#include "dvl_port_manager/PathfinderDVL.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

namespace dvl_port_manager
{
    PathfinderDVL::PathfinderDVL()
        : DVLProvider("192.168.0.32", 1033, 1035, sizeof(PathfinderFormat_t))
    {
        _publisherBodyVelocity = this->create_publisher<sonia_common_ros2::msg::BodyVelocityDVL>("/provider_dvl/dvl_velocity", 10);
        _publisherLeakSensor = this->create_publisher<std_msgs::msg::Bool>("/provider_dvl/dvl_leak_sensor", 10);
        _subscriptionEnableDisableDVL = this->create_subscription<std_msgs::msg::Bool>("/provider_dvl/enable_disable_dvl", 10, std::bind(&PathfinderDVL::_enableDisableDVL, this, _1));
    }

    void PathfinderDVL::_enableDisableDVL(const std_msgs::msg::Bool &msg)
    {
        if (msg.data)
        {
            _socket.Send(&_START_STOP_CMD[0]);
            rclcpp::sleep_for(5s);
            _socket.Send(&_START_DATA_CMD[0]);
            RCLCPP_INFO(this->get_logger(), "DVL Start Command Send.");
        }
        else
        {
            _socket.Send(&_START_STOP_CMD[0]);
            RCLCPP_INFO(this->get_logger(), "DVL Stop Command Send.");
        }
    }

    void PathfinderDVL::receiveDataThread()
    {
        rclcpp::Rate rate(GetSpinRate());
        while (rclcpp::ok())
        {
            _socket.ReceiveUDP();
            RCLCPP_DEBUG(this->get_logger(), "Data Received");

            getData<PathfinderFormat_t>(_dvlData);

            if (_dvlData.pd4.pathfinderDataId == _PATHFINDER_ID)
            {
                RCLCPP_DEBUG(this->get_logger(), "ID Correct");

                if (_dvlData.pd4.checksum == _calculateCheckSum<PathfinderFormat_t>(reinterpret_cast<uint8_t *>(_socket.GetRawData())))
                {
                    sonia_common_ros2::msg::BodyVelocityDVL message;

                    message.header.stamp.sec = _dvlData.pd4.secondFirstPing;
                    message.header.stamp.nanosec = _dvlData.pd4.hundredthFirstPing;
                    message.header.frame_id = "/EMU"; // PD4

                    message.x_vel_btm = ((double_t)_dvlData.pd4.xVelBtm) / 1000.0;
                    message.y_vel_btm = ((double_t)_dvlData.pd4.yVelBtm) / 1000.0;
                    message.z_vel_btm = ((double_t)_dvlData.pd4.zVelBtm) / 1000.0;
                    message.e_vel_btm = ((double_t)_dvlData.pd4.eVelBtm) / 1000.0;

                    message.velocity1 = ((double_t)_dvlData.pd4.velocity1) / 1000.0;
                    message.velocity2 = ((double_t)_dvlData.pd4.velocity2) / 1000.0;
                    message.velocity3 = ((double_t)_dvlData.pd4.velocity3) / 1000.0;
                    message.velocity4 = ((double_t)_dvlData.pd4.velocity4) / 1000.0;

                    _publisherBodyVelocity->publish(message);
                }
                else
                {
                    RCLCPP_WARN(this->get_logger(), "Bad Checksum");
                }
            }
            else
            {
                RCLCPP_WARN(this->get_logger(), "Pathfinder ID mismatch : %d", _PATHFINDER_ID);
            }
            rate.sleep();
        }
    }

    template <class T>
    uint16_t PathfinderDVL::_calculateCheckSum(uint8_t *dvlData)
    {
        float_t checksum{}, wholeChecksum, decimal;
        uint8_t sizeTotal{(sizeof(T) / sizeof(uint8_t)) - 2}; // Removing checksum value from array (-2)

        for (uint8_t i = 0; i < sizeTotal; ++i)
        {
            checksum += dvlData[i];
        }
        // ROS_INFO_STREAM("" << checksum);
        wholeChecksum = ceil(checksum / 65536.0f); // sizeof(uint16) = 2^16 = 65536
        decimal = wholeChecksum - checksum / 65536.0f;
        checksum = (1 - decimal) * 65536;

        return (uint16_t)ceil(checksum);
    }

} // namespace dvl_port_manager
