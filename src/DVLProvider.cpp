#include "dvl_port_manager/DVLProvider.hpp"

namespace dvl_port_manager
{

    DVLProvider::DVLProvider(std::string ipAddr, int tcpPort, int udpPort, size_t tram_size)
        : rclcpp::Node("dvl_port_manager"), _socket(tram_size), _rosSpinRate(20)
    {
        if (!_socket.ConnectTCP(ipAddr, tcpPort))
        {
            RCLCPP_ERROR(this->get_logger(), "Opening TCP Port for %s FAILED...", this->get_name());
            this->~DVLProvider();
        }
        RCLCPP_INFO(this->get_logger(), "Opening TCP Port for %s SUCCESSFUL...", this->get_name());
        if (!_socket.ConnectUDP(udpPort))
        {
            RCLCPP_ERROR(this->get_logger(), "Opening UDP Port for %s FAILED...", this->get_name());
            this->~DVLProvider();
        }
        RCLCPP_INFO(this->get_logger(), "Opening UDP Port for %s SUCCESSFUL...", this->get_name());
        _receiveThread = std::thread(std::bind(&DVLProvider::receiveDataThread, this));
    }
    
    DVLProvider::~DVLProvider()
    {
    }

} // namespace dvl_port_manager
