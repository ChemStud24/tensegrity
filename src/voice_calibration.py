import sys
import json
import rospy
import numpy as np
from pynput import keyboard
from tensegrity.msg import TensegrityStamped
# talk to me
from gtts import gTTS
from playsound import playsound

# short_lengths = range(180,80,-20)
short_lengths = range(220,70,-30)
long_lengths = range(250,350,20)

def speak(text):
    tts = gTTS(text)
    tts.save('/home/willjohnson/catkin_ws/src/tensegrity/calibration/tts.mp3')
    playsound('/home/willjohnson/catkin_ws/src/tensegrity/calibration/tts.mp3')

def calibrate(sensors,short_lengths,long_lengths,filename='../calibration/one-man calibration.json'):
    global quit

    # open the previous calibration file
    data = json.load(open(filename))
    m = np.array(data.get('m'))
    b = np.array(data.get('b'))

    all_caps = []
    all_lengths = []
    for sensor in sensors:
        print('\nCalibrating sensor ' + sensor.capitalize() + '...')
        speak('Calibrating sensor ' + sensor.capitalize())
        rospy.sleep(2)

        # figure out if it's a short sensor or a long sensor
        if letter2number(sensor) < 6:
            lengths = short_lengths
        else:
            lengths = long_lengths

        # calibrate at five lengths
        cap = []
        print('Set length to...')
        speak('Set length to')
        for i,length in enumerate(lengths):

            if quit:
                return

            # speak instructions
            print(str(length) + ' mm')
            speak(str(length) + ' millimeters')
            rospy.sleep(0.5)
            speak('Measuring')

            # measure capacitance
            cap.append(sensor_listener.capacitance[letter2number(sensor)])
            print('At length ' + str(length) + ' mm, sensor ' + sensor.capitalize() + ' has a capacitance of ' + str(cap[i]) + ' pF\n')

            speak('Next')
        # perform the linear fit
        fit = np.polyfit(lengths,cap,1)
        m[letter2number(sensor)] = fit[0]
        b[letter2number(sensor)] = fit[1]

    # save calibration results
    data = {'m':np.ndarray.tolist(m),'b':np.ndarray.tolist(b)}
    json.dump(data,open(filename,'w'))


def letter2number(sensor):
    dictionary = {l:n for n,l in enumerate('abcdefghi')}
    return dictionary.get(sensor)

def number2letter(sensor):
    dictionary = dict(enumerate('abcdefghi'))
    return dictionary.get(sensor)

def on_press(key):
    global quit
    if key == keyboard.KeyCode.from_char('q'):
        quit = True

class SensorListener:

    def __init__(self,num_sensors=9):
        self.capacitance = [0]*num_sensors
        self.strain_topic = "/control_msg"
        strain_sub = rospy.Subscriber(self.strain_topic,TensegrityStamped,self.callback)

    def callback(self,strain_msg):
        self.capacitance = [sensor.capacitance for sensor in strain_msg.sensors]

if __name__ == '__main__':
    rospy.init_node('Calibrator')
    sensor_listener = SensorListener()
    keyboard_listener = keyboard.Listener(on_press=on_press)
    keyboard_listener.start()
    move_on = False
    quit = False

    # which sensors should be calibrated?
    if len(sys.argv) > 1:
        sensors = sys.argv[1]
    else:
        sensors = 'abcdefghi'

    calibrate(sensors,short_lengths,long_lengths)