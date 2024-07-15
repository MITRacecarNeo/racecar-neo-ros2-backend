import subprocess
import datetime
import argparse

def record_ros2_bag(filename):
    #file name generation based on time (easier to recognize)
    timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    bag_filename = f"{filename}_{timestamp}.bag"

    #Command that starts recording all ROS2 topics
    command = ["ros2", "bag", "record", "-a", "-o", bag_filename]

    try:
        subprocess.run(command, check=True) #Runs the command using subprocess and waits for it to finish. Cntrl+C to stop recording
        print(f"Recording started. Bag file will be saved as {bag_filename}")
    except subprocess.CalledProcessError as e:
        print(f"An error occurred while trying to record the bag file: {e}")

if __name__ == "__main__":
    filename = input("Enter the base name for the bag file: ") #user prompt for filename
    record_ros2_bag(filename)