from dynamixel import Dynamixel
from config import *

motor = Dynamixel()
motor.sync_write_pos([0,0,0,0,0,0])
pos = motor.sync_read_pos()
print(pos)