#include "sonia_common_cpp/EthernetSocket.h"
#include <stdlib.h>
#include <iostream>
#include <chrono>
#include <thread>

int main()
{
    sonia_common_cpp::EthernetSocket socket = sonia_common_cpp::EthernetSocket();
    socket.ConnectUDP(1034);
    socket.ConnectTCP("192.168.0.32", 1033);

    std::string str1, cmd;
    str1 = "===\n";
    std::vector<uint8_t> strv(str1.begin(), str1.end());
    socket.Send(strv);
    std::this_thread::sleep_for(std::chrono::seconds(1));
    cmd = "CS\n";
    std::vector<uint8_t> cmdv(cmd.begin(), cmd.end());
    socket.Send(cmdv);

    for (int i = 0; i < 100; i++)
    {
        std::vector<uint8_t> data = socket.GetRawData();

        std::string str(data.begin(), data.end());

        std::cout << str << std::endl;

        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    
    socket.Send(strv);

    return EXIT_SUCCESS;
}
