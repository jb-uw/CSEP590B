#!/usr/bin/env python3

'''
This is starter code for Lab 6 on Coordinate Frame transforms.

'''

import asyncio
import cozmo
import numpy as np
import time
import math
from cozmo.util import degrees, radians

def get_relative_pose(object_pose, reference_frame_pose):
    xo = object_pose.position.x
    yo = object_pose.position.y
    xr = reference_frame_pose.position.x
    yr = reference_frame_pose.position.y
    angle_r = reference_frame_pose.rotation.angle_z.radians
    x1 = xo-xr
    y1 = yo-yr
    x = (x1*math.cos(angle_r)) + (y1*math.sin(angle_r))
    y = (y1*math.cos(angle_r)) - (x1*math.sin(angle_r))
    new_angle = object_pose.rotation.angle_z.degrees - reference_frame_pose.rotation.angle_z.degrees
    return cozmo.util.Pose(x, y, 0, angle_z=degrees(new_angle))
    
    
def get_world_pose(object_pose, reference_frame_pose):
    # convert relative frame to world frame
    x = object_pose.position.x
    y = object_pose.position.y
    angle_z = object_pose.rotation.angle_z.radians
    curr_angle = reference_frame_pose.rotation.angle_z.radians
    final_x = reference_frame_pose.position.x + x*math.cos(curr_angle) - y*math.sin(curr_angle)
    final_y = reference_frame_pose.position.y + x*math.sin(curr_angle) + y*math.cos(curr_angle)
    final_angle = (curr_angle + angle_z) % (2*math.pi)
    # flip to negative degree if it is greater than 180
    if final_angle > math.pi:
        final_angle = final_angle - (2*math.pi)
    return cozmo.util.Pose(final_x, final_y, 0, angle_z=radians(final_angle))

    
"""
def find_relative_cube_pose(robot: cozmo.robot.Robot):
	'''Looks for a cube while sitting still, prints the pose of the detected cube
	in world coordinate frame and relative to the robot coordinate frame.'''

	robot.move_lift(-3)
	robot.set_head_angle(degrees(0)).wait_for_completed()
	cube = None

	while True:
		try:
			cube = robot.world.wait_for_observed_light_cube(timeout=30)
			time.sleep(5)
			if cube:
				print("Robot pose: %s" % robot.pose)
				print("Cube pose: %s" % cube.pose)
				print("Cube pose in the robot coordinate frame: %s" % get_relative_pose(cube.pose, robot.pose))
				print("\n")
		except asyncio.TimeoutError:
			print("Didn't find a cube")


if __name__ == '__main__':

	cozmo.run_program(find_relative_cube_pose)
"""