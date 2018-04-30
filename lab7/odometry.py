#!/usr/bin/env python3

'''
Stater code for Lab 7.

'''

import cozmo
from cozmo.util import degrees, Angle, Pose, distance_mm, speed_mmps, radians
import math
import time
import sys
sys.path.insert(0, '../lab6')
from pose_transform import get_relative_pose

# time adjustment for drive_wheels duration to compensate for friction/lag time
time_adjust = 0.5

# Wrappers for existing Cozmo navigation functions

def cozmo_drive_straight(robot, dist, speed):
	"""Drives the robot straight.
		Arguments:
		robot -- the Cozmo robot instance passed to the function
		dist -- Desired distance of the movement in millimeters
		speed -- Desired speed of the movement in millimeters per second
	"""
	robot.drive_straight(distance_mm(dist), speed_mmps(speed)).wait_for_completed()

def cozmo_turn_in_place(robot, angle, speed):
	"""Rotates the robot in place.
		Arguments:
		robot -- the Cozmo robot instance passed to the function
		angle -- Desired distance of the movement in degrees
		speed -- Desired speed of the movement in degrees per second
	"""
	robot.turn_in_place(degrees(angle), speed=degrees(speed)).wait_for_completed()

def cozmo_go_to_pose(robot, x, y, angle_z):
	"""Moves the robot to a pose relative to its current pose.
		Arguments:
		robot -- the Cozmo robot instance passed to the function
		x,y -- Desired position of the robot in millimeters
		angle_z -- Desired rotation of the robot around the vertical axis in degrees
	"""
	robot.go_to_pose(Pose(x, y, 0, angle_z=degrees(angle_z)), relative_to_robot=True).wait_for_completed()

# Functions to be defined as part of the labs

def get_front_wheel_radius():
	"""Returns the radius of the Cozmo robot's front wheel in millimeters."""
	
	# ####
	# Tried different distances in the cozmo_drive_stright function until the
	# front wheel rotates about 1 revolution. 
	# That distance is about 87mm. So radius is 87/2pi = 13.85
	# ####
	return 13.85

def get_distance_between_wheels():
	"""Returns the distance between the wheels of the Cozmo robot in millimeters."""
	
	# ####
	# Made cozmo drive in circle with drive_wheels(50, 20). Timed the time it took
	# to complete one circle. With the time and speeds I can calculate the 
	# circumferences of the inner and outer treads and their respective radius.
	# The distance between wheels is the difference between the radii.
	#
	# P.S. This is a pretty poor way to do it. I noticed cozmo took longer and longer
	# to complete a circle
	# ####
	return 90

# positive ang_deg rotates cozmo to the left. negative to the right	
def rotate_front_wheel(robot, angle_deg):
	"""Rotates the front wheel of the robot by a desired angle.
		Arguments:
		robot -- the Cozmo robot instance passed to the function
		angle_deg -- Desired rotation of the wheel in degrees
	"""
	r =  get_front_wheel_radius()
	distance = 2 * math.pi * r * (angle_deg/360)
	cozmo_drive_straight(robot, distance, 40)

# positive dist is forward, poitive speed is forward
# if either dist or speed is negative, cozmo will go backward
# if both are negative cozmo would go forward instead
def my_drive_straight(robot, dist, speed):
	"""Drives the robot straight.
		Arguments:
		robot -- the Cozmo robot instance passed to the function
		dist -- Desired distance of the movement in millimeters
		speed -- Desired speed of the movement in millimeters per second
	"""
	t = abs(dist/speed) + time_adjust
	if dist < 0:
		speed = -speed
	robot.drive_wheels(speed, speed, duration=t)
	return t

def my_turn_in_place(robot, angle, speed):
	"""Rotates the robot in place.
		Arguments:
		robot -- the Cozmo robot instance passed to the function
		angle -- Desired distance of the movement in degrees
		speed -- Desired speed of the movement in degrees per second
	"""
	diameter = get_distance_between_wheels()
	circumference = math.pi * diameter
	distance = (abs(angle) / 360) * circumference
	t = distance/speed + time_adjust
	if angle < 0:
		speed = -speed
	robot.drive_wheels(-speed, speed, duration=t)
	time.sleep(0.2)

def my_go_to_pose1(robot, x, y, angle_z):
	"""Moves the robot to a pose relative to its current pose.
		Arguments:
		robot -- the Cozmo robot instance passed to the function
		x,y -- Desired position of the robot in millimeters
		angle_z -- Desired rotation of the robot around the vertical axis in degrees
	"""
	angle = math.degrees(math.atan(y/x))
	my_turn_in_place(robot, angle, 30)
	time.sleep(0.2) # a short sleep is necessary for action to work)
	distance = math.sqrt(x**2 + y**2)
	my_drive_straight(robot, distance, 30)
	time.sleep(0.2)
	my_turn_in_place(robot, (angle_z-angle), 30)
	time.sleep(0.2)

def find_rho(delta_x, delta_y):
	return math.sqrt(delta_x**2 + delta_y**2)
	
def find_alpha(delta_x, delta_y, curr_angle, debug = False):
	atan_angle = math.atan(delta_y/delta_x)	
	if delta_x >= 0 and delta_y >= 0:
		if debug:
			print("######### + +")
		alpha = atan_angle - curr_angle
	elif delta_x < 0 and delta_y >= 0:
		if curr_angle >= 0:
			if debug:
				print("######### - + curr_angle>0")
			alpha = math.pi + atan_angle - curr_angle
		else:
			if debug:
				print("######### - + curr_angle<0")
			alpha = atan_angle - curr_angle - math.pi
	elif delta_x < 0 and delta_y < 0:
		if curr_angle <= 0:
			if debug:
				print("######### - - curr_angle<0")
			alpha =  atan_angle - curr_angle - math.pi
		else:
			if debug:
				print("######### - - curr_angle>0")
			alpha =  math.pi - curr_angle + atan_angle
	else:
		if debug:
			print("######### + -")
		alpha = atan_angle - curr_angle
	return alpha

def find_eta(final_angle, curr_angle):
	eta = (final_angle - curr_angle) % (2*math.pi)
	if eta > math.pi:
			eta = eta - (2*math.pi)
	return eta
	
def my_go_to_pose2(robot, x, y, angle_z, debug = False):
	"""Moves the robot to a pose relative to its current pose.
		Arguments:
		robot -- the Cozmo robot instance passed to the function
		x,y -- Desired position of the robot in millimeters
		angle_z -- Desired rotation of the robot around the vertical axis in degrees
	"""
	# ####
	# I implemented Correll 3.4 and 3.5, Combining equations 3.64, 3.65, 3.66 and 3.67.
	# Since I know the current cordinates and angle and my goal's coordinate and
	# angle (translate from cozmo place and world plane), I plugged them in equation 3.65.
	# I plugged the results of 3.65 in 3.66 and 3.67, using 1 for p1, p2 & p3.
	# Then I plugged in the resulting x_dot and theta_dot to 3.64 to get phi_L and phi_R,
	# multiply that by r and get speeds for L and R.
	#
	# I have to modified the alpha equation in 3.65 significantly. Depends on which 
	# direction cozmo, alpha is calculated differently. Eta equation is also modified
	# to account for sign and magnitude
	#
	# With those modification I am able to go to pose (100, 100, 90) (or its mirror pose) 
	# from any pose and repeatedly. However I failed at go to pose (100, 100, 45)
	# ####

	r = get_front_wheel_radius()
	b = get_distance_between_wheels()
	p1 = 2
	p2 = 1
	p3 = 1
	curr_angle = robot.pose.rotation.angle_z.radians
	
	# convert relative frame to world frame
	final_x = robot.pose.position.x + x*math.cos(curr_angle) - y*math.sin(curr_angle)
	final_y = robot.pose.position.y + x*math.sin(curr_angle) + y*math.cos(curr_angle)
	final_angle = (curr_angle + math.radians(angle_z)) % (2*math.pi)
	# flip to negative degree if it is greater than 180
	if final_angle > math.pi:
		final_angle = final_angle - (2*math.pi)
		
	if debug:
		print("Final Position", final_x, final_y, math.degrees(final_angle))
		
	delta_x = final_x - robot.pose.position.x
	delta_y = final_y - robot.pose.position.y
	
	# Perform equations modified 3.65
	rho = find_rho(delta_x, delta_y)
	alpha = find_alpha(delta_x, delta_y, curr_angle, debug)
	eta = find_eta(final_angle, curr_angle)
		
	# allow 5% error from ditance and angle of final position
	distance_error = rho * 0.05
	angle_error = eta * 0.05
	prev_rho = rho
	if debug:
		print("POSE:", robot.pose.position.x, robot.pose.position.y, robot.pose.rotation.angle_z.degrees)
		print("delta_x", delta_x, "delta_y", delta_y, "curr_angle", curr_angle, "angle", math.degrees(math.atan(delta_y/delta_x)))
		print("rho", rho, "alpha", math.degrees(alpha), "eta", math.degrees(eta))
	
	while (abs(rho) > distance_error or abs(eta) > angle_error):
		# Equations 3.66, 3.67 & 3.64
		x_dot = p1 * rho
		theta_dot = p2 * alpha + p3 * eta
		speed_l = (2*x_dot - theta_dot*b) / (r)
		speed_r = (2*x_dot + theta_dot*b) / (r)
		if debug:
			print("drive", speed_l, speed_r)
		
		# break out of loop if we are not really moving anymore
		if speed_l < 2 and speed_r < 2:
			break
		
		robot.drive_wheels(speed_l, speed_r)	
		time.sleep(0.2)
		
		curr_angle = robot.pose.rotation.angle_z.radians
		delta_x = final_x - robot.pose.position.x
		delta_y = final_y - robot.pose.position.y
		prev_rho = rho
		rho = find_rho(delta_x, delta_y)
		
		# break out of loop if we are moving further from goal
		if (rho > prev_rho*1.05):
			break
			
		alpha = find_alpha(delta_x, delta_y, curr_angle, debug)
		eta = find_eta(final_angle, curr_angle)
		
		if debug:
			print("POSE:", robot.pose.position.x, robot.pose.position.y, robot.pose.rotation.angle_z.degrees)
			print("delta_x", delta_x, "delta_y", delta_y, "curr_angle", math.degrees(curr_angle), "angle", math.degrees(math.atan(delta_y/delta_x)))
			print("rho", rho, "alpha", math.degrees(alpha), "eta", math.degrees(eta))
			print("Final Position", final_x, final_y, math.degrees(final_angle))
			print("====================================")
		

def my_go_to_pose3(robot, x, y, angle_z, debug = False):
	"""Moves the robot to a pose relative to its current pose.
		Arguments:
		robot -- the Cozmo robot instance passed to the function
		x,y -- Desired position of the robot in millimeters
		angle_z -- Desired rotation of the robot around the vertical axis in degrees
	"""
	# decide if the pose is behind cozmo
	if x > 0:
		my_go_to_pose2(robot, x, y, angle_z)
	else:
		'''delta_x = x - robot.pose.position.x
		delta_y = y - robot.pose.position.y
		angle = 90
		if delta_x is not 0:
			angle += abs(math.atan(delta_y/delta_x))
		if debug:
			print("delta_x", delta_x, "delta_y", delta_y, "angle", angle)'''
		
		curr_x = robot.pose.position.x
		curr_y = robot.pose.position.y
		curr_angle = robot.pose.rotation.angle_z.radians
		
		# translate goal to world frame
		world_x = curr_x + x*math.cos(curr_angle) - y*math.sin(curr_angle)
		world_y = curr_y + x*math.sin(curr_angle) + y*math.cos(curr_angle)
		world_angle = (curr_angle + math.radians(angle_z)) % (2*math.pi)
		if debug:
			print("world_x", world_x, "world_y", world_y, "world_angle", math.degrees(world_angle))
			
		# rotate to look back before go_to_pose
		my_turn_in_place(robot, 180, 30)
		if debug:
			print(robot.pose)
			
		# translate goal to new reference frame
		new_pose = get_relative_pose(Pose(world_x, world_y, 0, angle_z=radians(world_angle)), robot.pose)
		if debug:
			print(new_pose)
		my_go_to_pose2(robot, new_pose.position.x, new_pose.position.y, new_pose.rotation.angle_z.degrees, debug)
	

def run(robot: cozmo.robot.Robot):

	print("***** Front wheel radius: " + str(get_front_wheel_radius()))
	print("***** Distance between wheels: " + str(get_distance_between_wheels()))

	## Example tests of the functions

	#print(robot.pose)
	cozmo_drive_straight(robot, 87, 30)
	cozmo_turn_in_place(robot, 10, 30)
	cozmo_go_to_pose(robot, 100, 100, 45)
	#print(robot.pose)

	rotate_front_wheel(robot, 20)
	my_drive_straight(robot, 80, 30)
	my_turn_in_place(robot, 360, 20)
	#print(robot.pose)
	my_go_to_pose1(robot, 100, 100, 0)
	my_go_to_pose2(robot, 100, 100, 90)
	my_go_to_pose3(robot, -100, 100, 90)
	#print(robot.pose)

if __name__ == '__main__':

	cozmo.run_program(run)
