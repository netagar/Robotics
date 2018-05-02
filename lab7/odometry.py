#!/usr/bin/env python3

'''
Stater code for Lab 7.

'''

import cozmo
from cozmo.util import degrees, Angle, Pose, distance_mm, speed_mmps
import math
import time

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
def get_front_wheel_radius(robot):
	"""Returns the radius of the Cozmo robot's front wheel in millimeters."""
	# ####
	# Empirically determine the radius of the robot's front wheel using the
	# cozmo_drive_straight() function.  You can write a separate script for doing
	# experiments to determine the radius.  This function should return the radius
	# in millimeters.  Write a comment that explains how you determined it and any
	# computation you do as part of this function.
	# ####
	# *** Logic to calculate the front wheel radius ***
	# The robot completes one revolution of the front wheel in going a distance of
	# 86mm
	# Therefore, the radius of the front wheel is 86 / (2 * pi)
	return 86 / (2 * math.pi)

	pass

def get_distance_between_wheels():
	"""Returns the distance between the wheels of the Cozmo robot in millimeters."""
	# ####
	# Empirically determine the distance between the wheels of the robot
	# using
	# robot.drive_wheels() function.  Write a comment that explains how you
	# determined
	# it and any computation you do as part of this function.
	# ####
	# *** Logic to calculate the front wheel radius ***
	# Let us say the robot is moved by keeping right wheel fixed so that it moves in a circle
	# Here I am moving it at 100 mmps applied to the left wheel for 6s such that it comes back
	# to its original position. 
	# Total distance covered by the robot =  600 mmps * 6s = 600 mm

	#robot.drive_wheels(100, 0, None, None, 6)

	# Since the robot moves in a circle with a radius equal to the distance between the wheels
	# Therefore, the distance between the wheels = 600 / (2 * pi)

	return 600 / (2 * math.pi)
	pass

def rotate_front_wheel(robot, angle_deg):
	"""Rotates the front wheel of the robot by a desired angle.
		Arguments:
		robot -- the Cozmo robot instance passed to the function
		angle_deg -- Desired rotation of the wheel in degrees
	"""
	# ####
	# ####

	angle_rad = math.radians(angle_deg)
	dist_to_cover = get_front_wheel_radius(robot) * angle_rad
	cozmo_drive_straight(robot, dist_to_cover, 10)

def my_drive_straight(robot, dist, speed):
	"""Drives the robot straight.
		Arguments:
		robot -- the Cozmo robot instance passed to the function
		dist -- Desired distance of the movement in millimeters
		speed -- Desired speed of the movement in millimeters per second
	"""
	# ####
	# Implement your version of a driving straight function using the
	# robot.drive_wheels() function.
	# ####
	#print("dist=" + str(dist) + "dist_mm=" + str(distance_mm(dist)))
	#print("speed=" + str(speed) + "speed_mm=" + str(speed_mmps(speed)))
	time_to_run = math.fabs(dist) / speed

	#print("time=" + str(time_to_run))

	if dist > 0 :
		robot.drive_wheels(speed, speed, None, None, time_to_run + 1.0)
	else:
		robot.drive_wheels(-speed, -speed, None, None, time_to_run + 1.0)
	time.sleep(time_to_run + 1)

	pass

def my_turn_in_place(robot, angle, speed):
	"""Rotates the robot in place.
		Arguments:
		robot -- the Cozmo robot instance passed to the function
		angle -- Desired distance of the movement in degrees
		speed -- Desired speed of the movement in degrees per second
	"""
	# ####
	# Implement your version of a rotating in place function using the
	# robot.drive_wheels() function.
	# ####

	angle_abs = math.fabs(angle)
	angle_rad = math.radians(angle_abs)

	dist_to_cover = angle_rad * get_distance_between_wheels()

	time_to_run = dist_to_cover / speed

	if angle > 0:
		robot.drive_wheels(0, speed, None, None, time_to_run + 0.6)
	else:
		robot.drive_wheels(speed, 0, None, None, time_to_run + 0.6)

	time.sleep(time_to_run + 1)

	pass

def my_go_to_pose1(robot, x, y, angle_z):
	"""Moves the robot to a pose relative to its current pose.
		Arguments:
		robot -- the Cozmo robot instance passed to the function
		x,y -- Desired position of the robot in millimeters
		angle_z -- Desired rotation of the robot around the vertical axis in degrees
	"""
	# ####
	# Implement a function that makes the robot move to a desired pose
	# using the my_drive_straight and my_turn_in_place functions.  This should
	# include a sequence of turning in place, moving straight, and then turning
	# again at the target to get to the desired rotation (Approach 1).
	# ####

	my_turn_in_place(robot, angle_z, 100)

	dis = math.fabs(math.sqrt(math.pow(x,2) + math.pow(y,2)))

	my_drive_straight(robot, dis, 100)

	pass

def my_go_to_pose2(robot, x, y, angle_z):
	"""Moves the robot to a pose relative to its current pose.
		Arguments:
		robot -- the Cozmo robot instance passed to the function
		x,y -- Desired position of the robot in millimeters
		angle_z -- Desired rotation of the robot around the vertical axis in degrees
	"""
	# ####
	# Implement a function that makes the robot move to a desired pose
	# using the robot.drive_wheels() function to jointly move and rotate the
	# robot to reduce distance between current and desired pose (Approach 2).
	# ####
	pass

def my_go_to_pose3(robot, x, y, angle_z):
	"""Moves the robot to a pose relative to its current pose.
		Arguments:
		robot -- the Cozmo robot instance passed to the function
		x,y -- Desired position of the robot in millimeters
		angle_z -- Desired rotation of the robot around the vertical axis in degrees
	"""
	# ####
	# TODO: Implement a function that makes the robot move to a desired pose
	# as fast as possible.  You can experiment with the built-in Cozmo function
	# (cozmo_go_to_pose() above) to understand its strategy and do the same.
	# ####
	pass

def cozmo_program(robot: cozmo.robot.Robot):

	#robot.move_head(-5)
	# Tell the lift motor to start lowering the lift (at 5 radians per second)
	#robot.move_lift(-5)
	# Tell Cozmo to drive the left wheel at 25 mmps (millimeters per second),
	# and the right wheel at 50 mmps (so Cozmo will drive Forwards while also
	# turning to the left
	#robot.drive_wheels(50, -50)
	#robot.drive_wheels(10, 50)
	

	#get_front_wheel_radius(robot)

	#time.sleep(8)

	print("***** Front wheel radius: " + str(get_front_wheel_radius(robot)))
	print("***** Distance between wheels: " + str(get_distance_between_wheels()))

	## Example tests of the functions

	#cozmo_drive_straight(robot, 62, 50)
	#cozmo_turn_in_place(robot, 90, 30)
	#cozmo_go_to_pose(robot, 100, 100, 45)

	#rotate_front_wheel(robot, 180)
	#my_drive_straight(robot, -62, 50)
	#my_turn_in_place(robot, -90, 30)

	#my_go_to_pose1(robot, 100, 100, 45)
	#my_go_to_pose2(robot, 100, 100, 45)
	#my_go_to_pose3(robot, 100, 100, 45)


#if __name__ == '__main__':

	
#cozmo.run_program(cozmo_program)



