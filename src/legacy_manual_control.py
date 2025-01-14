#!/usr/bin/env python3
import sys
import serial
import time
import math
from math import cos,sin,tan
import re
import json
import xlrd
import numpy as np
from pynput import keyboard

def flush(serial_port):
    serial_port.reset_input_buffer()
    serial_port.reset_output_buffer()
    time.sleep(0.001)

def send_command(serial_port, input_string, delay_time):
    # to_microcontroller_msg = f'{input_string}\n'
    to_microcontroller_msg = '          ' + '{}\n'.format(input_string)
    serial_port.write(to_microcontroller_msg.encode('UTF-8'))
    if delay_time < 0:
        delay_time = 0
    time.sleep(delay_time/1000)

def tensegrity_run(device_name):
    global keep_going
    global serial_port
    # A welcome message
    print("Running serial_tx_cmdline node with device: " + device_name)
    # create the serial port object, non-exclusive (so others can use it too)
    serial_port = serial.Serial(port=device_name, baudrate=115200, timeout=1) # flush out any old data
    flush(serial_port)
    # finishing setup.
    print("Opened port. Ctrl-C to stop.")
    
    # If not using ROS, we'll do an infinite loop:
    #while keep_going and not rospy.is_shutdown():
    while keep_going:
        flush(serial_port)
        line = serial_port.read_until(b'\n')# read data string sent from central
        line = str(line)#.encode('utf-8')# convert to string
        line = line.split('*')[-1] # remove padding
        if not line[0] == 'q' and not line[0] == 'S':
            print(line)

    # Nicely shut down this script.
    print("\nShutting down serial_tx_cmdline...")
    send_command(serial_port, "s", 0)
    sys.exit()

def onpress(key):
    global keep_going
    global max_speed
    global pressed_key
    global num_motors
    global serial_port
    global awaiting_command
    if key == keyboard.KeyCode.from_char('q'):
        keep_going = False
    elif key == keyboard.KeyCode.from_char('s'):
        keep_going = False
    elif key == keyboard.KeyCode.from_char('f'):
        if pressed_key in range(num_motors):
            if awaiting_command:
                send_command(serial_port, "d " + str(pressed_key+1) + " " + str(max_speed), 0)
                awaiting_command = False
    elif key == keyboard.KeyCode.from_char('b'):
        if pressed_key in range(num_motors):
            if awaiting_command:
                send_command(serial_port, "d " + str(pressed_key+1) + " " + str(-max_speed), 0)
                awaiting_command = False
    elif key == keyboard.KeyCode.from_char('0'):
        pressed_key = 0
    elif key == keyboard.KeyCode.from_char('1'):
        pressed_key = 1
    elif key == keyboard.KeyCode.from_char('2'):
        pressed_key = 2
    elif key == keyboard.KeyCode.from_char('3'):
        pressed_key = 3
    elif key == keyboard.KeyCode.from_char('4'):
        pressed_key = 4
    elif key == keyboard.KeyCode.from_char('5'):
        pressed_key = 5

def onrelease(key):
    global awaiting_command
    if key == keyboard.KeyCode.from_char('f'):
        send_command(serial_port, "s", 0)
        awaiting_command = True
    elif key == keyboard.KeyCode.from_char('b'):
        send_command(serial_port, "s", 0)
        awaiting_command = True

if __name__ == '__main__':
    ## keyboard listener for quitting
    keep_going = True
    awaiting_command = True
    my_listener = keyboard.Listener(on_press=onpress,on_release=onrelease)
    my_listener.start()

    pressed_key = 6
    max_speed = 80# set duty cycle as 99 for the max speed, resolution can be improved by changing the bits in C++ code 
    num_motors = 6# set number of motors
    command = [0] * num_motors
    # flip = [-1,-1,-1,1,1,-1]
    serial_port = ''

    try:
        # the 0-th arg is the name of the file itself, so we want the 1st.
        # tensegrity_run(sys.argv[1])
        tensegrity_run('/dev/ttyACM0')
    except KeyboardInterrupt:
        # why is this here?
        pass