from dynamixel import Dynamixel
from config import *
import time

motor = Dynamixel()
motor.sync_write_pos([500,500,500,500,500,500])
time.sleep(5)
pos = motor.sync_read_pos()
print(pos)