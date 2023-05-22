#include <iostream>
#include <cstdint>
#include "A1_16.h"
// for sleep
#include <chrono>
#include <thread>

#include "ros/ros.h"
#include "std_msgs/UInt8.h"

int main(){
    unsigned char id = 1;
    unsigned char time = 10;
    unsigned int pos = 400;
    unsigned int now_pos;
    unsigned char reset_time = 255;
    /*
    A1_16_Ini();
    SetPositionI_JOG(id,reset_time,pos);
    std::this_thread::sleep_for(std::chrono::milliseconds(2550));
    for (pos=400; pos<=600; pos+=10){
        // std::cout << ReadPosition(id);
        SetPositionI_JOG(id,time,pos);
        std::this_thread::sleep_for(std::chrono::milliseconds(time*10));
        // now_pos = ReadPosition(id);
        // std::cout <<"now position:" << now_pos << std::endl;
    }
    // SetPositionI_JOG(id,time,pos);
    */
   A1_16_Ini();
   while(true){
        now_pos = ReadPosition(id);
        std::cout <<"now position:" << now_pos << std::endl;
   }
    uart_close();
    return 0;
}