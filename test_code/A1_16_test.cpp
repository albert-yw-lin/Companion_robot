#include <iostream>
#include <cstdint>
#include "A1_16.h"
// for sleep
#include <chrono>
#include <thread>

int main(){
    unsigned char id = 1;
    unsigned char time = 50;
    unsigned int pos = 0;
    unsigned int now_pos;
    
    A1_16_Ini();
    SetPositionI_JOG(id,255,pos);
    std::this_thread::sleep_for(std::chrono::milliseconds(2550));
    for (pos=0; pos<=1023; pos+=50){
        // std::cout << ReadPosition(id);
        SetPositionI_JOG(id,time,pos);
        now_pos = ReadPosition(id);
        std::cout <<"now position:" << now_pos << endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(time*10));
    }
    // SetPositionI_JOG(id,time,pos);
    uart_close();
    return 0;
}