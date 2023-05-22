#!/usr/bin/env python3
from dynamixel import Dynamixel
from config import *

import rospy
from std_msgs.msg import Float64MultiArray

class Head:
    def __init__(self) -> None:
        self.motor = Dynamixel(IDS_HEAD)
        self.face_center_x, self.face_center_y = (FACE_CENTER_X,FACE_CENTER_Y)
        self.motor_pos = None
        rospy.init_node('head', anonymous=True)
        rospy.Subscriber('face_center', Float64MultiArray, self.face_center_callback)

    def face_center_callback(self, face_center):
        # rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
        self.face_center_x, self.face_center_y = face_center.data
        error_x = FACE_CENTER_X-self.face_center_x
        error_y = self.face_center_y-FACE_CENTER_Y
        add_motor_x = int(error_x*(62.2/360)*4095*P_GAIN_X)
        if(error_y>=0):add_motor_y = int(error_y*(48.8/360)*4095*P_GAIN_Y_DOWN)
        else:add_motor_y = int(error_y*(48.8/360)*4095*P_GAIN_Y_UP)
        self.motor_pos = self.motor.sync_read_pos()
        self.motor_pos[0] += add_motor_x
        self.motor_pos[1] += add_motor_y
        self.motor.sync_write_pos(self.motor_pos)

if __name__ == '__main__':
    try:
        head = Head()
        rospy.spin()

    # except KeyboardInterrupt:
    #     pass
    # except rospy.ROSInterruptException:
    #     pass

    except Exception as error_code:
        print(error_code)   

    finally:
        rospy.signal_shutdown('Shutting down head')
        head.motor.close()
        print("################################\n \
               ### Closing Head normally ...###\n \
               ################################")



