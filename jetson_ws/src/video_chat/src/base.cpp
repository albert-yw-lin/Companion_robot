#include <iostream>
#include <cstdint>
#include "A1_16.h"
// for sleep
#include <chrono>
#include <thread>

#include "ros/ros.h"
#include "std_msgs/UInt8.h"

int pos = 512; //0-1023
int step = 40;
unsigned char wait_time = 100;
unsigned char id = 1;


void base_callback(const std_msgs::UInt8::ConstPtr& cmd){
    if (cmd->data == 0){
        //turn left
        pos -= step;
        SetPositionI_JOG(id,wait_time,pos);
        std::this_thread::sleep_for(std::chrono::milliseconds(wait_time*10));
    }
    else if (cmd->data == 1){
        //turn right
        pos += step;
        SetPositionI_JOG(id,wait_time,pos);
        std::this_thread::sleep_for(std::chrono::milliseconds(wait_time*10));
    }
}

int main(int argc, char **argv){

    A1_16_Ini();
    SetPositionI_JOG(id,wait_time,pos);

    ros::init(argc, argv, "base");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("turn_base", 1, base_callback);
    ros::spin();

    uart_close();
    return 0;
}