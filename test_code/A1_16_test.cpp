#include <iostream>
#include <cstdint>
#include "A1_16.h"

int main(){
    unsigned char id = 1;
    unsigned char time = 255;
    unsigned int pos = 0;
    A1_16_Ini();
    // for (int i=0; i<=1023; i++){
    //     SetPositionI_JOG(0,1,i);
    // }
    SetPositionI_JOG(id,time,pos);
    return 0;
}