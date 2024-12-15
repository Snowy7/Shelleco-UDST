# Lane Detection Project

This project implements a lane detection system using OpenCV and Python. The system processes video frames to detect lane lines and provides directional guidance based on the detected lanes.

## Project Structure

- `main.py`: Main script to process real-time camera frames for lane detection. (CLI Version)
- `roi.py`: Contains functions for handling Region of Interest (ROI) operations.
- `motor.py`: Contains functions for handling motor operations based on lane detection.
- `image.py`: Script to process a single image for lane detection.
- `constants.py`: Contains constant values used throughout the project.
- `readme.md`: Project documentation.
- `gui.py`: Main script to process real-time camera frames for lane detection with simple GUI Contrls. (GUI Version)

## Dependencies

- Python 3.x
- OpenCV
- NumPy

## Installation

1. Clone the repository:
    ```sh
    git clone https://github.com/Snowy7/lane-detection.git
    cd lane-detection
    ```

2. Install the required packages:
    ```sh
    pip install -r requirements.txt
    ```

## Usage

### Running the Lane Detection on Video

To run the lane detection on a video, execute the `main.py` script:
```sh
  python gui.py
```
or for the debug version
```sh
  python main.py
```

### Running the Lane Detection on an Image

To run the lane detection on a single image, execute the `image.py` script:
```sh
  python image.py
```

## Configuration

The `constants.py` file contains various configuration parameters such as ROI bounds, colors, and direction thresholds. You can adjust these parameters to fit your specific use case.

## Functions

### `roi.py`

- `find_roi_center()`: Finds the center of the ROI.
- `draw_roi_center(frame, roi_center)`: Draws the center of the ROI on the frame.
- `find_roi_lane_center(roi, roi_center, frame=None)`: Finds the center of the lane within the ROI.
- `draw_roi_lane_center(frame, roi_lane_center)`: Draws the center of the lane on the frame.
- `draw_direction_text(frame, direction)`: Draws the direction text on the frame.
- `draw_legend(frame)`: Draws the legend on the frame.

### `motor.py`

- `handle_motor_operations(roi_center, roi_lane_center)`: Determines the direction based on the ROI center and lane center.

## License

```TODO: Add license information ```