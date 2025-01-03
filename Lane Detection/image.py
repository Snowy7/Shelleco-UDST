﻿import numpy as np
import cv2 as cv
import constants as constant
import roi as ro
import motor as mt
import os

# get the path of the current file
current_path = os.path.dirname(os.path.abspath(__file__))
# get the path of the project folder
project_path = os.path.abspath(os.path.join(current_path, os.pardir))
# get the path of the images folder
image_path = os.path.join(project_path, 'images')
# get the path of the image
image_path = os.path.join(image_path, '4.jpg')

frame = cv.imread(image_path)

# resize image
frame = cv.resize(frame, (constant.WIDTH, constant.HEIGHT))

gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
blur = cv.GaussianBlur(gray, (5, 5), 0)
canny = cv.Canny(blur, 50, 200)
colored_blur = cv.GaussianBlur(frame, (5, 5), 0)
hsv = cv.cvtColor(colored_blur, cv.COLOR_BGR2HSV)

# define range of white color in HSV
lower_white = np.array([0, 0, 250])
upper_white = np.array([179, 60, 255])

# returns binary image highlighting white lines
mask = cv.inRange(hsv, lower_white, upper_white)

lines = cv.HoughLinesP(canny, rho=1, theta=np.pi/180, threshold=50, minLineLength=50, maxLineGap=20)

# Create a blank image to draw lines
line_image = np.zeros_like(canny)

if lines is not None:
    for line in lines:
        x1, y1, x2, y2 = line[0]
        cv.line(line_image, (x1, y1), (x2, y2), (255, 255, 255), 5)

# Combine the line image with the mask
combined_binary = cv.bitwise_or(line_image, mask)

# combine canny and hsv binary frames to have more robust detection (in our case hsv was working perfect)
#combined_binary = cv.bitwise_or(canny, mask)

roi = combined_binary[
      constant.ROI_HEIGHT_LOWER_BOUND:constant.ROI_HEIGHT_UPPER_BOUND,
      constant.ROI_WIDTH_LOWER_BOUND:constant.ROI_WIDTH_UPPER_BOUND]

ro.draw_legend(frame)
roi_center = ro.find_roi_center()
ro.draw_roi_center(frame, roi_center)

print('roi_center:', roi_center)
roi_lane_center = ro.find_roi_lane_center(roi, roi_center, frame)
ro.draw_roi_lane_center(frame, roi_lane_center)

direction = mt.handle_motor_operations(roi_center, roi_lane_center)
ro.draw_direction_text(frame, direction)

cv.imshow('mask', mask)
cv.imshow('roi', roi)
cv.imshow('canny', canny)
cv.imshow('combined_binary', combined_binary)
cv.imshow('original', frame)

# show one frame at a time (press space to show next frame)
key = cv.waitKey(0)
while key not in [ord('q'), ord(' ')]:
    key = cv.waitKey(0)

# When everything done, release the capture
cv.destroyAllWindows()