#pragma once

#include "rclcpp/rclcpp.hpp"
#include "sonia_common_cpp/EthernetSocket.h"

namespace dvl_port_manager
{
    class DVLProvider : public rclcpp::Node
    {
    public:
        DVLProvider(std::string ipAddr, int tcpPort, int udpPort, size_t tram_size);
        ~DVLProvider();

        void SetSpinRate(size_t spinRate) { _rosSpinRate = spinRate; }
        size_t GetSpinRate() { return _rosSpinRate; }

    protected:
        virtual void receiveDataThread() = 0;

        template <class T>
        inline void getData(T &x)
        {
            x = *((T *)(_socket.GetRawData()));
        }

        sonia_common_cpp::EthernetSocket _socket;

    private:
        std::thread _receiveThread;
        size_t _rosSpinRate;
    };
}
