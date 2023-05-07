#include <iostream>
#include <cstdint>
#include "A1_16.h"

int main(){
    uint8_t id = 1;
    uint8_t time = 3;
    uint8_t pos = 500;
    A1_16_Ini();
    // for (int i=0; i<=1023; i++){
    //     SetPositionI_JOG(0,1,i);
    // }
    SetPositionI_JOG(id,time,pos);
    return 0;
}