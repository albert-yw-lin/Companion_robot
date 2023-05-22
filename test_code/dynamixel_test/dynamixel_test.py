from dynamixel import Dynamixel
from config import *
import time

motor = Dynamixel()
while(1):

    pos = motor.sync_read_pos()
    print(pos)