import numpy as np
import cv2 as cv
import constants as constant
import roi as ro
import motor as mt

cameraIndex = 0
cap = cv.VideoCapture(cameraIndex)

if not cap.isOpened():
    print('Cannot open camera')
    exit()
    
started = False

while True:
    ret, frame = cap.read()

    if not ret:
        print('Can\'t receive frame. Exiting ...')
        continue
    # resize frame to 640x480
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

    roi = combined_binary[
        constant.ROI_HEIGHT_LOWER_BOUND:constant.ROI_HEIGHT_UPPER_BOUND,
        constant.ROI_WIDTH_LOWER_BOUND:constant.ROI_WIDTH_UPPER_BOUND]



    ro.draw_legend(frame)
    roi_center = ro.find_roi_center()
    ro.draw_roi_center(frame, roi_center)

    roi_lane_center = ro.find_roi_lane_center(roi, roi_center)
    ro.draw_roi_lane_center(frame, roi_lane_center)

    direction = mt.handle_motor_operations(roi_center, roi_lane_center, started)
    ro.draw_direction_text(frame, direction)

    
    cv.imshow('roi', roi)
    cv.imshow('original', frame)
    
    # show buttons
    cv.namedWindow("Car Controller")
    cv.createButton("Left", mt.left)
    cv.createButton("Right", mt.right)
    cv.createButton("Forward", mt.forward)
    cv.createButton("Stop", mt.stop)
    
    
    # wait for key press and check if it's 'q', then exit
    key = cv.waitKey(1)
    if key == ord('q'):
        break

# When everything done, release the capture
cap.release()
cv.destroyAllWindows()
#mt.stop()