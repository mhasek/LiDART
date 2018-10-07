#!/usr/bin/env python

import csv
import rospy
from std_msgs.msg import Bool
from std_msgs.msg import Int32
from enum import Enum
import rospkg 
import pdb

class Direction(Enum):
    LEFT = 1
    CENTER = 2
    RIGHT = 3

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
    
    msg = Int32()

    # only read a new line in csv when switching from hallway to intersection
    if (intersection_flag.data):  
        if (not prev_flag):
            if counter >= len(csv_array):
                print("You have arrived at your destination")
                counter = 0
                
            current_command = csv_array[counter] # 0,1,2
            msg.data = current_command
            counter = counter + 1

        print "Hi I'm at an intersection"
        print current_command
    else:
        current_command = directiondict["hallway"]
        print "Hi I'm in a hallway"
        print current_command

    prev_flag = intersection_flag.data
    msg.data = current_command
    print msg.data
    pub_command.publish(msg.data.value)

if __name__ == '__main__':
    # csv_array
    csv_array = import_csv(file_name)
    rospy.init_node('track_parser_node', anonymous=True)
    rospy.Subscriber("intersection", Bool, intersection_callback)
    rospy.spin()