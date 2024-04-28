#include "sonia_common_cpp/EthernetSocket.h"
#include <stdlib.h>
#include <iostream>
#include <chrono>

int main()
{
    sonia_common_cpp::EthernetSocket socket = sonia_common_cpp::EthernetSocket();
    socket.Connect("192.168.0.32", 9002);

    while (true)
    {
        std::vector<uint8_t> data = socket.GetRawData();

        std::string str(data.begin(), data.end());

        std::cout << str << std::endl;
    }
    
    
    return EXIT_SUCCESS;
}
