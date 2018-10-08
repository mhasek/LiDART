#!/usr/bin/env python

import csv
import rospy
from std_msgs.msg import Bool
from std_msgs.msg import Int32
from sensor_msgs.msg import Joy
from enum import Enum
import rospkg 
import pdb
 
FRAME_RESET = 40

frame_cntr = 0

class Direction(Enum):
    LEFT = 1
    CENTER = 2
    RIGHT = 3
    STOP = 4

# Create a dictionary that maps strings to driving strategies for reading CSV file
directiondict = {
    "turn_right": Direction.RIGHT,
    "hallway": Direction.CENTER,
    "turn_left": Direction.LEFT
}

rospack = rospkg.RosPack()
file_name = rospack.get_path('lidart_wall_following')+'/scripts/CSV/Test.csv'
global csv_array

counter = 0
prev_flag = False
current_command = Direction.CENTER
pub_command = rospy.Publisher('track_command', Int32, queue_size=10)

# Generic CSV reader - creates list of directions composed of "Left, Right, Center" (for now)
def import_csv(file_name):
    instruction_array = []
    with open(file_name) as csvfile:
        reader = csv.reader(csvfile)
        for row in reader:
            for instruction in row:
                instruction_array.append(directiondict[instruction.lower()])
    return instruction_array

# Moves to next direction in csv list and publishes it
# If the callback recieves "False" publishes "hallway", which also maps to 3
def intersection_callback(intersection_flag):
    global counter
    global prev_flag
    global current_command
    global frame_cntr
    
    msg = Int32()

    frame_cntr += 1

    # only read a new line in csv when switching from hallway to intersection
    if (intersection_flag.data):  
        if (not prev_flag):
            if counter >= len(csv_array):
                counter = 0
            
            if frame_cntr > FRAME_RESET:
                counter += 1
                frame_cntr = 0

            current_command = csv_array[counter] # 0,1,2
            msg.data = current_command

        print "Hi I'm at an intersection"
    else:
        current_command = directiondict["hallway"]
        print "Hi I'm in a hallway"

    prev_flag = intersection_flag.data
    msg.data = current_command
    print msg.data
    pub_command.publish(msg.data.value)

def Teleop_callback(data):
    global counter

    x_button = bool(data.buttons[0])
    a_button = bool(data.buttons[1])
    b_button = bool(data.buttons[2])
    y_button = bool(data.buttons[3])

    if x_button and (not y_button) and (not b_button):
        csv_array = [Direction.LEFT, Direction.STOP]
        counter = 0
    elif y_button and (not x_button) and (not b_button):
        csv_array = [Direction.CENTER,Direction.STOP]
        counter = 0
    elif b_button and (not x_button) and (not y_button):
        csv_array = [Direction.RIGHT,Direction.STOP]
        counter = 0

if __name__ == '__main__':
    # csv_array
    csv_array = import_csv(file_name)
    rospy.init_node('track_parser_node', anonymous=True)
    rospy.Subscriber("intersection", Bool, intersection_callback)
    rospy.Subscriber("vesc/joy",Joy,Teleop_callback)
    rospy.spin()