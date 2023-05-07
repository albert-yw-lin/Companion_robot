#include <iostream>
#include <cstdint>
#include "A1_16.h"

int main(){
    unsigned char id = 1;
    unsigned char time = 10;
    unsigned int pos = 0;
    
    A1_16_Ini();
    // for (pos=0; pos<=1023; pos++){
    //     // std::cout << ReadPosition(id);
    //     SetPositionI_JOG(id,time,pos);
    // }
    SetPositionI_JOG(id,time,pos);
    uart_close();
    return 0;
}