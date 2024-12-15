# Region of interest (ROI) parameters
ROI_WIDTH_LOWER_BOUND = 0
ROI_WIDTH_UPPER_BOUND = 637
ROI_HEIGHT_LOWER_BOUND = 320
ROI_HEIGHT_UPPER_BOUND = 400

# Frame size
WIDTH = 640
HEIGHT = 480

# Colors
RED_COLOR = [0, 0, 255]  # ROI center color
GREEN_COLOR = [0, 255, 0]  # Lane center color
WHITE_COLOR = [255, 255, 255]  # Legend color

# Direction thresholds, we can adjust these values according to our needs
DIRECTION_THRESHOLDS = {
    'Forward': range(-20, 20),
    'Right': range(-ROI_WIDTH_UPPER_BOUND, -20),
    'Left': range(20, ROI_WIDTH_UPPER_BOUND)
}