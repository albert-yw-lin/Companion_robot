#!/usr/bin/env python3
# -*- coding: utf-8 -*-
from dynamixel_sdk import *                    # Uses Dynamixel SDK library
from config import *
import time

class Dynamixel:

    def __init__(self) -> None:
        ### initialization
        self.portHandler = PortHandler(DEVICENAME)
        self.packetHandler = PacketHandler(PROTOCOL_VERSION)
        self.groupSyncWrite = GroupSyncWrite(self.portHandler, self.packetHandler, ADDR_GOAL_POSITION, LEN_GOAL_POSITION)
        self.groupSyncRead = GroupSyncRead(self.portHandler, self.packetHandler, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)

        # Open port
        if self.portHandler.openPort():
            print("Succeeded to open the port")
        else:
            print("Failed to open the port, automatically terminate...")
            quit()

        # Set port baudrate
        if self.portHandler.setBaudRate(BAUDRATE):
            print("Succeeded to change the baudrate")
        else:
            print("Failed to change the baudrate, automatically terminate...")
            quit()

        for id in DXL_ID:
            ### torque disable to change settings in EEPROM section
            dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, id, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
            self.check_txrx(dxl_comm_result, dxl_error, ADDR_TORQUE_ENABLE)

            ### initiailze motor posiotion
            self.sync_write_pos(self, POS_INIT)
            time.sleep(2) # enough time to get to position

            ### set position limits of each motor
            dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(self.portHandler, id, ADDR_MIN_POSITION_LIMIT, POS_LIMIT[id][0]) # min
            self.check_txrx(dxl_comm_result, dxl_error, ADDR_MIN_POSITION_LIMIT)
            dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(self.portHandler, id, ADDR_MAX_POSITION_LIMIT, POS_LIMIT[id][1]) # max
            self.check_txrx(dxl_comm_result, dxl_error, ADDR_MAX_POSITION_LIMIT)

            ### set profiles(acceleration, velocity) of each motor
            dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(self.portHandler, id, ADDR_PROFILE_ACCELERATION, PROFILE_ACCELERATION)
            self.check_txrx(dxl_comm_result, dxl_error, ADDR_PROFILE_ACCELERATION)
            dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(self.portHandler, id, ADDR_PROFILE_VELOCITY, PROFILE_VELOCITY)
            self.check_txrx(dxl_comm_result, dxl_error, ADDR_PROFILE_VELOCITY)

            ### set moving threshold
            dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(self.portHandler, id, ADDR_MOVING_THRESHOLD, MOVING_THRESHOLD)
            self.check_txrx(dxl_comm_result, dxl_error, ADDR_MOVING_THRESHOLD)

            # Enable Dynamixel#id Torque
            dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, id, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
            self.check_txrx(dxl_comm_result, dxl_error, ADDR_TORQUE_ENABLE)

            # Add parameter storage for Dynamixel#id present position value
            dxl_addparam_result = self.groupSyncRead.addParam(id)
            self.check_groupSync(id, dxl_addparam_result, mode='r')

    def check_txrx(self, dxl_comm_result, dxl_error=0, addr=-1):
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error), end="\t")
            print("target address: %d"%addr)
 
    def check_groupSync(self, id, result, mode='r'):
        if result != True:
            if mode == 'r':
                print("[ID:%03d] groupSyncRead failed" % id)
                print("automatically terminate...")
                quit()
            elif mode == 'w':
                print("[ID:%03d] groupSyncWrite failed" % id)
                print("automatically terminate...")
                quit()

    def sync_write_pos(self, goal_positions):
        ### add parameter
        for id in DXL_ID:
            param_goal_position = [DXL_LOBYTE(DXL_LOWORD(goal_positions[id])), 
                                   DXL_HIBYTE(DXL_LOWORD(goal_positions[id])), 
                                   DXL_LOBYTE(DXL_HIWORD(goal_positions[id])), 
                                   DXL_HIBYTE(DXL_HIWORD(goal_positions[id]))] # DO NOT cnage the order of these four
            dxl_addparam_result = self.groupSyncWrite.addParam(id, param_goal_position)
            self.check_groupSync(id, dxl_addparam_result, mode='w')
        
        ### start moving to the goal position
        dxl_comm_result = self.groupSyncWrite.txPacket()
        self.check_txrx(dxl_comm_result)
        
        # Clear syncwrite parameter storage
        self.groupSyncWrite.clearParam()

    def sync_read_pos(self):
        positions = [] # list of present positions
        dxl_comm_result = self.groupSyncRead.txRxPacket()
        self.check_txrx(dxl_comm_result)
        for id in DXL_ID:
            dxl_getdata_result = self.groupSyncRead.isAvailable(id, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)
            self.check_groupSync(id, dxl_getdata_result, mode='r')
            positions.append(self.groupSyncRead.getData(id, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION))
        return positions
    
    def close(self):
        self.groupSyncRead.clearParam()
        for id in DXL_ID:
            dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, id, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
            self.check_txrx(dxl_comm_result, dxl_error, ADDR_TORQUE_ENABLE)
        self.portHandler.closePort()

