#include "sonia_common_cpp/EthernetSocket.h"
#include <stdlib.h>
#include <iostream>
#include <chrono>
#include <thread>

int main()
{
    sonia_common_cpp::EthernetSocket socket = sonia_common_cpp::EthernetSocket();
    socket.Connect("192.168.0.32", 9002);

    std::string str, cmd;
    str = "===\n";
    std::vector<uint8_t> strv(str.begin(), str.end());
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
    
    str = "===\n";
    socket.Send(strv);

    return EXIT_SUCCESS;
}
