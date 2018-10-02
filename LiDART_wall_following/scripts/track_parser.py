import csv
import rospy
from std_msgs.msg import Bool
from std_msgs.msg import Int32
from enum import Enum

class Direction(Enum):
    LEFT = 1
    CENTER = 2
    RIGHT = 3

# Create a dictionary that maps strings to driving strategies for reading CSV file
directiondict = {
    "left": Direction.LEFT,
    "right": Direction.RIGHT,
    "center": Direction.CENTER,
    "straight": Direction.CENTER,
    "hallway": Direction.CENTER,
    "forward": Direction.CENTER
}

file_name = "CSV/Test.csv"

global csv_array
counter = 0
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
    if (intersection_flag):
        if counter >= len(csv_array):
            print("You have arrived at your destination")
            return
        current_command = csv_array[counter]
        counter = counter + 1
        pub_command.publish(current_command)
    else:
        pub_command.publish(directiondict["hallway"])


if __name__ == '__main__':
    # csv_array
    csv_array = import_csv(file_name)
    rospy.init_node('pid_controller_node', anonymous=True)
    rospy.Subscriber("intersection", Bool, intersection_callback)
    rospy.spin()