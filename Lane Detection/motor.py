import constants as constant
import RPi.GPIO as GPIO
import gpiod
from time import sleep
import tkinter

## add your servo BOARD PIN number ##
servo_pin = 11
forward_pin = 27

GPIO.setmode(GPIO.BOARD)
GPIO.setup(servo_pin, GPIO.OUT)

pwm=GPIO.PWM(servo_pin, 50)
pwm.start(0)

## edit these duty cycle % values ##
left_pwm = 11.5
neutral_pwm = 7
right_pwm = 4.1
#### that's all folks ####

print("Starting servo motor")


def forward():
    pwm.ChangeDutyCycle(neutral_pwm)

def right():
    pwm.ChangeDutyCycle(right_pwm)
    
def left():
    pwm.ChangeDutyCycle(left_pwm)
    
def SetVoltage(PIN, value):
    chip = gpiod.Chip('gpiochip4')
    pin = chip.get_line(PIN)
    pin.request(consumer="LED", type=gpiod.LINE_REQ_DIR_OUT)
    pin.set_value(value)
    pin.release()

if __name__ == "__main__":
    root = tkinter.Tk()
    root.title("Car Controller")
    root.geometry("300x300")
    left_button = tkinter.Button(root, text="Left", command=left)
    left_button.pack()
    right_button = tkinter.Button(root, text="Right", command=right)
    right_button.pack()
    forward_button = tkinter.Button(root, text="Forward", command=forward)
    forward_button.pack()
    root.mainloop()

def stop(pause=False):
    SetVoltage(forward_pin, 0)
    if pause:
        return
    pwm.stop()
    GPIO.cleanup()


def handle_motor_operations(roi_center, roi_lane_center, started=False):
    roi_center_x = roi_center[0]
    roi_lane_center_x = roi_lane_center[0]
    center_difference = roi_center_x - roi_lane_center_x

    direction = 'Stop'
    direction_keys = constant.DIRECTION_THRESHOLDS.keys()

    for direction_key in direction_keys:
        if center_difference in constant.DIRECTION_THRESHOLDS.get(direction_key):
            direction = direction_key
            break
    
    if not started:
        stop(True)
        return direction
    
    SetVoltage(forward_pin, 1)
    if direction == 'Forward':
         forward()
    elif direction == 'Right':
         right()
    elif direction == 'Left':
         left()
    else:
         stop()

    return direction

