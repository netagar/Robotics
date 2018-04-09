#!/usr/bin/env python3

import asyncio
import sys
import math

import cv2
import numpy as np

sys.path.insert(0, '../lab4')
import find_ball

import cozmo
from cozmo.util import degrees, distance_mm, speed_mmps, Pose

try:
	from PIL import ImageDraw, ImageFont
except ImportError:
	sys.exit('run `pip3 install --user Pillow numpy` to run this example')


# Define a decorator as a subclass of Annotator; displays battery voltage
class BatteryAnnotator(cozmo.annotate.Annotator):
	def apply(self, image, scale):
		d = ImageDraw.Draw(image)
		bounds = (0, 0, image.width, image.height)
		batt = self.world.robot.battery_voltage
		text = cozmo.annotate.ImageText('BATT %.1fv' % batt, color='green')
		text.render(d, bounds)

# Define a decorator as a subclass of Annotator; displays the ball
class BallAnnotator(cozmo.annotate.Annotator):

	ball = None

	def apply(self, image, scale):
		d = ImageDraw.Draw(image)
		bounds = (0, 0, image.width, image.height)

		if BallAnnotator.ball is not None:

			#double size of bounding box to match size of rendered image
			BallAnnotator.ball = np.multiply(BallAnnotator.ball,2)

			#define and display bounding box with params:
			#msg.img_topLeft_x, msg.img_topLeft_y, msg.img_width, msg.img_height
			box = cozmo.util.ImageBox(BallAnnotator.ball[0]-BallAnnotator.ball[2],
									  BallAnnotator.ball[1]-BallAnnotator.ball[2],
									  BallAnnotator.ball[2]*2, BallAnnotator.ball[2]*2)
			cozmo.annotate.add_img_box_to_image(image, box, "green", text=None)

			BallAnnotator.ball = None


async def run(robot: cozmo.robot.Robot):
	'''The run method runs once the Cozmo SDK is connected.'''

	#add annotators for battery level and ball bounding box
	robot.world.image_annotator.add_annotator('battery', BatteryAnnotator)
	robot.world.image_annotator.add_annotator('ball', BallAnnotator)

	try:

		while True:

			await robot.set_head_angle(degrees(0)).wait_for_completed()
			await robot.set_lift_height(0.0).wait_for_completed()
			#get camera image
			event = await robot.world.wait_for(cozmo.camera.EvtNewRawCameraImage, timeout=30)

			#convert camera image to opencv format
			opencv_image = cv2.cvtColor(np.asarray(event.image), cv2.COLOR_RGB2GRAY)

			#find the ball
			ball = find_ball.find_ball(opencv_image, False)

			#set annotator ball
			BallAnnotator.ball = ball

			## TODO: ENTER YOUR SOLUTION HERE
			if ball is not None:

				min_dis = math.fabs((ball[0] - ball[2]))
				if min_dis < ball[2]:
					await robot.set_lift_height(1.0).wait_for_completed()
					break
				await robot.go_to_pose(Pose(ball[0] - ball[2],0, 0, angle_z=degrees(0)), relative_to_robot=True).wait_for_completed()

			if ball is None:
				await robot.drive_straight(distance_mm(-100), speed_mmps(50)).wait_for_completed()

	except KeyboardInterrupt:
		print("")
		print("Exit requested by user")
	except cozmo.RobotBusy as e:
		print(e)



if __name__ == '__main__':
	cozmo.run_program(run, use_viewer = True, force_viewer_on_top = True)

