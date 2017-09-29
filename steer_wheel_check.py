#!/usr/bin/env python3
from ev3dev.ev3 import *
from math import *
import time

mC = MediumMotor('outC')
mC.position = 0
i = 12 / 20
POSITION = 280
count = 0
count_old = -1

def calibration():
	while True:
		count = mC.position
		if count == count_old - 4 or count == count_old + 4 or count == count_old:
			break
		mC.run_to_abs_pos(position_sp = POSITION, speed_sp = 100)
		count_old = count
		time.sleep(0.2)

	while mC.position != count - 140: 
		mC.run_to_abs_pos(position_sp = count - 140,speed_sp = 100)


"""
def steer_wheel_check():
	print(mC.position)
	mC.run_to_abs_pos(position_sp = 40, speed_sp = 100)
	time.sleep(0.05)
	print(mC.position)
	mC.run_to_abs_pos(position_sp = -40, speed_sp = 100)
	time.sleep(0.05)
	print(mC.position)
	mC.run_to_abs_pos(position_sp = 0, speed_sp = 100)
	time.sleep(0.05)
	print(mC.position)

steer_wheel_check()
"""