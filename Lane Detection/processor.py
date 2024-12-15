import numpy as np
import cv2 as cv
import constants as constant
import roi as ro
import motor as mt

def lane_following(frame, started=False):
    frame = cv.resize(frame, (constant.WIDTH, constant.HEIGHT))

    # Step 1: Grayscale conversion
    gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)

    # Step 2: Gaussian Blur
    blur = cv.GaussianBlur(gray, (5, 5), 0)

    # Step 3: Canny Edge Detection
    canny = cv.Canny(blur, 50, 200)

    # Step 4: HSV Mask
    hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
    lower_white = np.array([0, 0, 250])
    upper_white = np.array([179, 60, 255])
    mask = cv.inRange(hsv, lower_white, upper_white)

    # Step 5: Hough Lines
    lines = cv.HoughLinesP(canny, rho=1, theta=np.pi / 180, threshold=50, minLineLength=50, maxLineGap=20)
    line_image = np.zeros_like(canny)

    if lines is not None:
        for line in lines:
            x1, y1, x2, y2 = line[0]
            cv.line(line_image, (x1, y1), (x2, y2), (255, 255, 255), 5)

    # Step 6: Combine Lines and Mask
    combined_binary = cv.bitwise_or(line_image, mask)

    # Check ROI and further processing
    roi = combined_binary[
        constant.ROI_HEIGHT_LOWER_BOUND:constant.ROI_HEIGHT_UPPER_BOUND,
        constant.ROI_WIDTH_LOWER_BOUND:constant.ROI_WIDTH_UPPER_BOUND
    ]
    cv.imshow("ROI", roi)
    
    ro.draw_legend(frame)
    roi_center = ro.find_roi_center()
    ro.draw_roi_center(frame, roi_center)

    roi_lane_center = ro.find_roi_lane_center(roi, roi_center)
    ro.draw_roi_lane_center(frame, roi_lane_center)

    direction = mt.handle_motor_operations(roi_center, roi_lane_center, started)
    ro.draw_direction_text(frame, direction)

    return direction, frame, canny, line_image, mask, combined_binary
