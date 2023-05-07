#include <iostream>
#include <cstdint>
#include "A1_16.h"
// for sleep
#include <chrono>
#include <thread>

int main(){
    unsigned char id = 1;
    unsigned char time = 100;
    unsigned int pos = 0;
    unsigned int now_pos;
    unsigned char reset_time = 255;
    
    A1_16_Ini();
    SetPositionI_JOG(id,reset_time,pos);
    std::this_thread::sleep_for(std::chrono::milliseconds(2550));
    for (pos=0; pos<=1023; pos+=50){
        // std::cout << ReadPosition(id);
        SetPositionI_JOG(id,time,pos);
        std::this_thread::sleep_for(std::chrono::milliseconds(time*10));
        // now_pos = ReadPosition(id);
        // std::cout <<"now position:" << now_pos << std::endl;
    }
    // SetPositionI_JOG(id,time,pos);
    uart_close();
    return 0;
}