#!/usr/bin/env python3
from dynamixel import Dynamixel
from config import *
from dynamixel_config import POS_LIMIT

import rospy
from std_msgs.msg import UInt8MultiArray

class Arm:
    def __init__(self) -> None:
        self.motor = Dynamixel(IDS_ARM)
        rospy.init_node('arm', anonymous=True)
        rospy.Subscriber('pose', UInt8MultiArray, self.pose_callback)

    def pose_callback(self, pose):
        # rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
        shoulder_L, arm_L, shoulder_R, arm_R = pose.data
        shoulder_L = self.mapping(shoulder_L, SHOULDER_MIN, SHOULDER_MAX, POS_LIMIT[ID_SHOULDER_L][1], POS_LIMIT[ID_SHOULDER_L][0])
        arm_L = self.mapping(arm_L, ARM_MIN, ARM_MAX, POS_LIMIT[ID_ARM_L][0], POS_LIMIT[ID_ARM_L][1])  ### min:0, max:1, different with others!
        shoulder_R = self.mapping(shoulder_R, SHOULDER_MIN, SHOULDER_MAX, POS_LIMIT[ID_SHOULDER_R][1], POS_LIMIT[ID_SHOULDER_R][0])
        arm_R = self.mapping(arm_R, ARM_MIN, ARM_MAX, POS_LIMIT[ID_ARM_R][1], POS_LIMIT[ID_ARM_R][0])
        self.motor.sync_write_pos([shoulder_L, arm_L, shoulder_R, arm_R])

    def mapping(self, value, old_min, old_max, new_min, new_max):
        return (value - old_min) * (new_max - new_min) / (old_max - old_min) + new_min

if __name__ == '__main__':
    try:
        arm = Arm()
        rospy.spin()

    # except KeyboardInterrupt:
    #     pass
    # except rospy.ROSInterruptException:
    #     pass

    except Exception as error_code:
        print(error_code)   

    finally:
        rospy.signal_shutdown('Shutting down arm')
        arm.motor.close()
        print("##################################\n", \
              "### Closing Arm normally ...###\n", \
              "##################################")



