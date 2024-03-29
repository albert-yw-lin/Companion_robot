#!/usr/bin/env python3
from dynamixel import Dynamixel
from config import *
from dynamixel_config import POS_LIMIT, POS_INIT

import rospy
from std_msgs.msg import Float64MultiArray, UInt8MultiArray, UInt8

### ROS head_arm node to control dynamixel motors based on dynamixel motor module
class Head_arm:
    def __init__(self) -> None:
        self.motor = Dynamixel()
        self.motor_pos = POS_INIT

        self.turn_base = UInt8()
        self.turn_base_pub = rospy.Publisher('turn_base', UInt8, queue_size=3)
        
        rospy.init_node('head_arm', anonymous=True)
        rospy.Subscriber('face_center', Float64MultiArray, self.face_center_callback)
        rospy.Subscriber('pose', UInt8MultiArray, self.pose_callback)
        self.turn_base_rate = rospy.Rate(10)


    def face_center_callback(self, face_center):
        # rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
        face_center_x, face_center_y = face_center.data
        error_x = FACE_CENTER_X-face_center_x
        error_y = face_center_y-FACE_CENTER_Y
        add_motor_head_x = int(error_x*(62.2/360)*4095*P_GAIN_X)
        if(error_y>=0):add_motor_head_y = int(error_y*(48.8/360)*4095*P_GAIN_Y_DOWN)
        else:add_motor_head_y = int(error_y*(48.8/360)*4095*P_GAIN_Y_UP)

        motor_present_pos = self.motor.sync_read_pos()

        self.motor_pos[ID_HEAD_X] = motor_present_pos[ID_HEAD_X] + add_motor_head_x
        self.motor_pos[ID_HEAD_Y] = motor_present_pos[ID_HEAD_Y] + add_motor_head_y
        self.motor.sync_write_pos(self.motor_pos)

        ### check if base should move
        if motor_present_pos[ID_HEAD_X] >= POS_LIMIT[ID_HEAD_X][1]- TURN_THRESHOLD : self.turn_base.data = TURN_RIGHT
        elif motor_present_pos[ID_HEAD_X] <= POS_LIMIT[ID_HEAD_X][0] + TURN_THRESHOLD : self.turn_base.data = TURN_LEFT
        else:self.turn_base.data = TURN_STOP
        self.turn_base_pub.publish(self.turn_base)
        self.turn_base_rate.sleep()



    def pose_callback(self, pose):
        # rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
        shoulder_L, arm_L, shoulder_R, arm_R = pose.data
        self.motor_pos[ID_SHOULDER_L] = self.mapping(shoulder_L, SHOULDER_MIN, SHOULDER_MAX, POS_LIMIT[ID_SHOULDER_L][0], POS_LIMIT[ID_SHOULDER_L][1])
        self.motor_pos[ID_ARM_L] = self.mapping(arm_L, ARM_MIN, ARM_MAX, POS_LIMIT[ID_ARM_L][0], POS_LIMIT[ID_ARM_L][1])  # min:0, max:1, different with others!
        self.motor_pos[ID_SHOULDER_R] = self.mapping(shoulder_R, SHOULDER_MIN, SHOULDER_MAX, POS_LIMIT[ID_SHOULDER_R][1], POS_LIMIT[ID_SHOULDER_R][0])
        self.motor_pos[ID_ARM_R] = self.mapping(arm_R, ARM_MIN, ARM_MAX, POS_LIMIT[ID_ARM_R][1], POS_LIMIT[ID_ARM_R][0])

    def mapping(self, value, old_min, old_max, new_min, new_max):
        return int((value - old_min) * (new_max - new_min) / (old_max - old_min) + new_min)

if __name__ == '__main__':
    try:
        head_arm = Head_arm()
        rospy.spin()

    except KeyboardInterrupt:
        pass

    except rospy.ROSInterruptException:
        pass

    except Exception as error_code:
        print(error_code)   

    finally:
        rospy.signal_shutdown('Shutting down head_arm')
        head_arm.motor.close()
        print("\n #####################################\n", \
                 "### Closing Head_arm normally ... ###\n", \
                 "#####################################")



