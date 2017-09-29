#!/usr/bin/env python3
from ev3dev.ev3 import *
from math import *
import time

RADIUS = 0.022
LENGTH = 0.3

mA = LargeMotor('outA')
mB = LargeMotor('outB')
mA.polarity = 'inversed'
mB.polarity = 'inversed'

mC = MediumMotor('outC')
i = 12 / 20

gs = GyroSensor()

rotate_A_old = rotate_B_old = theta = dt = e0 = eD = eI = 0

data_x = open('data_x.txt', 'w')
data_y = open('data_y.txt', 'w')

def pi_mod(n):
	return -(abs(n) % pi) if abs(n) > pi else n

def front_wheels_angle():
	phi = radians(i * mC.position)
	return phi

def forward_vel():
	global rotate_A_old, rotate_B_old, dt,rotate_A_new, rotate_B_new
	rotate_A_new = mA.position
	rotate_B_new = mB.position
	vel = radians(((rotate_A_new - rotate_A_old) + (rotate_B_new - rotate_B_old)) / 2) * RADIUS / dt if dt != 0.0 else 0
	rotate_A_old = rotate_A_new
	rotate_B_old = rotate_B_new
	return vel

"""def robot_angle(vel, phi):
	global theta, dt
	d_theta = (vel / LENGTH) * tan(phi) 
	theta += d_theta * dt
	return theta"""

def run_motors(goal_orientation, dist_left):
	global i
	K1 = 50
	K2 = 150
	KI = 2
	KD = 2
	K3 = 4
	MAX = 100
	
	uI = KI * eI * dt
	uD = KD * (eD / dt)
	uP = K2 * dist_left
	u2 = uP + uD + uI 
	u1 = K1 * goal_orientation
	if(abs(goal_orientation) > 50):
		goal_orientation = copysign(1, goal_orientation) * 50
	if (abs(u1) > MAX ):
		u1 = copysign(1, u1) * MAX
	if (abs(u2) > MAX ):
		u2 = copysign(1, u2) * MAX
	mA.run_direct(duty_cycle_sp = u2)
	mB.run_direct(duty_cycle_sp = u2)
	mC.run_to_abs_pos(position_sp = (1 / i) * goal_orientation, speed_sp = 100)

def main():
	global dt, e0, eD, e1, eI
	start_time = time.time()
	x = y = x_old = y_old = theta = theta_old = 0
	gs.mode = 'GYRO-RATE'
	gs.mode = 'GYRO-ANG'
	mA.position = mB.position = mC.position = 0
	dist_traveled = 0

	goal_x = float(input('x = '))
	goal_y = float(input('y = '))
	goal_dist = dist_left = sqrt(goal_x ** 2 + goal_y ** 2)
	goal_orientation = pi_mod(atan2(goal_y, goal_x))	
	last_time = time.time() - start_time

	while dist_left > 0.04:
		v = forward_vel()
		phi = front_wheels_angle()
		theta += gs.angle
		theta = radians(theta)
		theta = pi_mod(theta)
		current_time = time.time() - start_time
		dt = current_time - last_time
		print( 'goal dist: {0} dist left: {1} x: {2} y: {3}'.format(goal_dist, dist_left, x, y))
		print('wheel angle: {0} robot_angle: {1}'.format(phi, degrees(theta)))

		x += v * cos(theta) * dt
		y += v * sin(theta) * dt
		
		data_x.write(str(x) + '\n')
		data_y.write(str(y) + '\n')

		dist_traveled += sqrt((x - x_old) ** 2 + (y - y_old) ** 2)

		x_old = x
		y_old = y
		goal_dist = sqrt((goal_x - x) ** 2 + (goal_y - y) ** 2)

		orientation_diff = degrees(atan2((goal_y - y), (goal_x - x)) - theta)
		print ('orientation_diff: ', orientation_diff, '\n')
		dist_left = goal_dist 
		e1 = dist_left
		eD += dist_left
		eI = e1 - e0
		e0 = e1
		last_time = current_time
		run_motors(orientation_diff, dist_left)

	mA.stop(stop_action = 'brake')
	mB.stop(stop_action = 'brake')
	mC.stop(stop_action = 'brake')
	
	data_x.close()
	data_y.close()

if __name__ == "__main__":
	main()