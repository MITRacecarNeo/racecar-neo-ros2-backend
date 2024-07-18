import subprocess
import datetime
import argparse

# def run_alias(alias_name):
#     """Run a bash alias by sourcing the .bashrc first."""
#     command = f"source ~/.bashrc && {alias_name}"
#     try:
#         subprocess.call(['/bin/bash', '-i', '-c', command])
#         print(f"Alias '{alias_name}' executed successfully.")
#     except subprocess.CalledProcessError as e:
#         print(f"An error occurred while executing alias '{alias_name}': {e}")

def run_alias(alias_name, session_name):
    """Run a bash alias in a new tmux window."""
    command = f"bash -i -c 'source ~/.bashrc && {alias_name}'"
    try:
        # Start a new tmux window and run the alias command
        subprocess.call(['tmux', 'new-window', '-t', session_name, '-n', alias_name, command])
        print(f"Alias '{alias_name}' executed successfully in a new tmux window.")
    except subprocess.CalledProcessError as e:
        print(f"An error occurred while executing alias '{alias_name}' in a new tmux window: {e}")



def record_ros2_bag(filename):
    #file name generation based on time (easier to recognize)
    timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    bag_filename = f"{filename}_{timestamp}.bag"
    
     # Create a new tmux session
    session_name = "ros_bag_session"
    subprocess.call(['tmux', 'new-session', '-d', '-s', session_name])
    
    #Calling the aliases
    # run_alias('ydlidar')
    # run_alias('teleop')
    run_alias('ydlidar', session_name)
    run_alias('teleop', session_name)

    #Command that starts recording all ROS2 topics
    command = ["ros2", "bag", "record", "-a", "-o", bag_filename]

    try:
        subprocess.run(command, check=True) #Runs the command using subprocess and waits for it to finish. Cntrl+C to stop recording
        print(f"Recording started. Bag file will be saved as {bag_filename}")
    except subprocess.CalledProcessError as e:
        print(f"An error occurred while trying to record the bag file: {e}")

if __name__ == "__main__":
    filename = input("Enter the name for the bag fil that will be tied to the time: ") #user prompt for filename
    record_ros2_bag(filename)