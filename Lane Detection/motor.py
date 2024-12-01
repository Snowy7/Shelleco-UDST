import constants as constant

def handle_motor_operations(roi_center, roi_lane_center):
    roi_center_x = roi_center[0]
    roi_lane_center_x = roi_lane_center[0]
    center_difference = roi_center_x - roi_lane_center_x

    direction = 'Stop'
    direction_keys = constant.DIRECTION_THRESHOLDS.keys()

    for direction_key in direction_keys:
        if center_difference in constant.DIRECTION_THRESHOLDS.get(direction_key):
            direction = direction_key
            break

    # if direction == 'Forward':
    #     go_forward()
    # elif direction == 'Right':
    #     turn_right()
    # elif direction == 'Left':
    #     turn_left()
    # else:
    #     stop_motor()

    return direction