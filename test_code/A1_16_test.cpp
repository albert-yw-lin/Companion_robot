#include <iostream>
#include <cstdint>
#include "A1_16.h"
// for sleep
#include <chrono>
#include <thread>

int main(){
    unsigned char id = 1;
    unsigned char time = 10;
    unsigned int pos = 0;
    
    A1_16_Ini();
    SetPositionI_JOG(id,time*10,pos);
    std::this_thread::sleep_for(std::chrono::milliseconds(time*10*10));
    for (pos=0; pos<=1023; pos+=10){
        // std::cout << ReadPosition(id);
        SetPositionI_JOG(id,time,pos);
        std::this_thread::sleep_for(std::chrono::milliseconds(time*10));
    }
    // SetPositionI_JOG(id,time,pos);
    uart_close();
    return 0;
}