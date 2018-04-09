#!/usr/bin/env python3

# Joyce Brombaugh
# Lab 4

import cv2
import sys
import copy

import numpy as np

try:
	from PIL import Image, ImageDraw, ImageFont
except ImportError:
	sys.exit('install Pillow to run this code')


def find_ball(opencv_image, debug=False):
	"""Find the ball in an image.
		
		Arguments:
		opencv_image -- the image
		debug -- an optional argument which can be used to control whether
				debugging information is displayed.
		
		Returns [x, y, radius] of the ball, and [0,0,0] or None if no ball is found.
	"""	
	## TODO: INSERT YOUR SOLUTION HERE
	circles = None
	# Blur image first
	opencv_image = cv2.medianBlur(opencv_image,5)
	if debug:
		# display the canny image for debugging
		edges = cv2.Canny(opencv_image,50,100)
		pil_image = Image.fromarray(edges)
		pil_image.show()
	# Apply HughCircles	
	circles=cv2.HoughCircles(opencv_image,cv2.HOUGH_GRADIENT,2,100,param1=250,param2=60)
	
	if circles is not None:
		if debug:
			print(str(len(circles[0])) + " circle(s) found")
			display_circles(opencv_image, circles[0], circles[0][0])
		return circles[0][0]
	else:
		if debug:
			print("No circle found")
		return None


def display_circles(opencv_image, circles, best=None):
	"""Display a copy of the image with superimposed circles.
		
	   Provided for debugging purposes, feel free to edit as needed.
	   
	   Arguments:
		opencv_image -- the image
		circles -- list of circles, each specified as [x,y,radius]
		best -- an optional argument which may specify a single circle that will
				be drawn in a different color.  Meant to be used to help show which
				circle is ranked as best if there are multiple candidates.
		
	"""
	#make a copy of the image to draw on
	circle_image = copy.deepcopy(opencv_image)
	circle_image = cv2.cvtColor(circle_image, cv2.COLOR_GRAY2RGB, circle_image)
	
	for c in circles:
		# draw the outer circle
		cv2.circle(circle_image,(c[0],c[1]),c[2],(255,255,0),2)
		# draw the center of the circle
		cv2.circle(circle_image,(c[0],c[1]),2,(0,255,255),3) 
		# write coords
		cv2.putText(circle_image,str(c),(c[0],c[1]),cv2.FONT_HERSHEY_SIMPLEX,
					.5,(255,255,255),2,cv2.LINE_AA)				
	
	# highlight the best circle in a different color
	if best is not None:
		# draw the outer circle
		cv2.circle(circle_image,(best[0],best[1]),best[2],(0,0,255),2)
		# draw the center of the circle
		cv2.circle(circle_image,(best[0],best[1]),2,(0,0,255),3) 
		# write coords
		cv2.putText(circle_image,str(best),(best[0],best[1]),cv2.FONT_HERSHEY_SIMPLEX,
					.5,(255,255,255),2,cv2.LINE_AA)            
		
	# display the image
	pil_image = Image.fromarray(circle_image)
	pil_image.show()  
	  
if __name__ == "__main__":

	filenames = ("test01.bmp", "test03.bmp", "test05.bmp", "test07.bmp", "test09.bmp",
				 "test11.bmp", "test13.bmp", "test15.bmp", "test17.bmp", "test19.bmp",
				 "test21.bmp", "test23.bmp", "test25.bmp", "test27.bmp", "test29.bmp",
				 "test31.bmp", "test33.bmp", "test35.bmp", "test37.bmp", "test39.bmp",
				 "test41.bmp", "test43.bmp", "test45.bmp", "test47.bmp", "test49.bmp",
				 "test51.bmp", "test53.bmp", "test55.bmp", "test57.bmp", "test59.bmp",
				 "test61.bmp", "test63.bmp", "test65.bmp", "test67.bmp", "test69.bmp",
				 "test71.bmp", "test73.bmp", "test75.bmp", "test77.bmp", "test79.bmp",
				 "test81.bmp", "test83.bmp", "test85.bmp", "test87.bmp", "test89.bmp",
				 "test91.bmp", "test93.bmp", "test95.bmp", "test97.bmp", "test99.bmp")
	for file in filenames:
		print(str(file) + ":")
		opencv_image = cv2.imread("./imgs/" + file, cv2.COLOR_GRAY2RGB)

		if opencv_image is not None:
			ball = find_ball(opencv_image, True)
			print(str(ball))
		
		input("Press Enter for next image. Ctrl-C to quit. ")
