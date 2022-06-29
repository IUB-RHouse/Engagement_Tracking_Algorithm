# Engagement Tracking Algorithm
### Unfinished - Only working for Azure Kinect and not for the Boson
## Setup instructions (Azure Kinect)
NOTE: You must be using Ubuntu 18.04 (The Azure Kinect does not currently support Ubuntu 20.04 or newer, and neither does it's ROS driver)
1. Install ROS Melodic using [this link](https://wiki.ros.org/melodic/Installation/Ubuntu) and set it up using [this link](https://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment)
2. Install the [Azure Kinect SDK](https://docs.microsoft.com/en-us/azure/kinect-dk/sensor-sdk-download) onto your Linux system. Make sure to follow ALL instructions, there are multiple webpages with instructions required for it to work on Linux.
3. Install the [Azure Kinect ROS Driver](https://github.com/microsoft/Azure_Kinect_ROS_Driver) using the Building Guide (near the bottom of the README). Clone that repo in `catkin_ws/src/` and then run *catkin_make* in `catkin_ws/` For usage information and a list of topics go to [this link](https://github.com/microsoft/Azure_Kinect_ROS_Driver/blob/melodic/docs/usage.md)
4. The Azure Kinect ROS Driver is set to run at 5 FPS. Other options include 15 and 30 FPS. The Engagement Tracking Algorithm defaults at 15 FPS, so we must change the Azure Kinect ROS Driver to match. To do this, go into `catkin_ws/src/Azure_Kinect_ROS_Driver/launch/`
5. Then, using your favorite text editor (such as Vim or Nano) edit the `driver.launch` file. You will need to change the arg named "fps" which is currently set to 5. Set it to 15 (or whatever you are planning to use) and then save and exit the file.
6. Inside the `catkin_ws/` folder (not one of it's subfolders) run *source devel/setup.bash*
    - **NOTE**: You will need to do this every time you reboot the computer.

## Setup instructions (Engagement Algorithm)
1. Clone this repo by clicking the colored "Code" button on this page and copying the link. Go into catkin_ws/src/ and type in *git clone \[link\]*
2. In `catkin_ws/` run *catkin_make*
3. After that, run *source devel/setup.bash*
4. Everytime you plan to run this program, open three terminals (or three tabs in a terminal)
    1. In the first one run the *roscore* command
    2. In the second one go to `catkin_ws/src/Azure_Kinect_ROS_Driver/launch/` and run *roslaunch driver.launch*
        - If you get an error saying `Resource not found: azure_kinect_ros_driver`, run step 6 of the Azure Kinect Setup Instructions
    3. In the third one go to `catkin_ws/src/Engagement_Tracking_Algorithm/`. You will use this terminal to run this algorithm. Please read "Recording Instructions" or "Analysis Instructions" for how to run this program.

## Recording Instructions
1. In the root of this folder (which should be `~/catkin_ws/src/Engagement_Tracking_Algorithm)`, open a terminal and run *python scripts/listener.py -r \[Duration (in seconds, optional)\]*
- Examples of commands to run this program
    - *python scripts/listener.py -r 5*
        - This will record the camera RGB output for 5 seconds and then quit the program
        - NOTE: You may end the program early by hitting *ctrl+c*
    - *python scripts/listener.py -r*
        - This will record the camera RGB output until someone hits *ctrl+c* on the keyboard to exit the program
2. The output video will be in the output folder after the recording is finished
3. 15fps is what is coded into this script, however, the Azure ROS Driver default is 5. You must change the default in the Azure Kinect ROS Driver driver.launch file to 15. Other options are 5, and 30. Make sure the number in this file matches the one in the driver.launch
    - If you followed the setup instructions for the Azure Kinect then these should already match, however, if you wish to change to 5 or 30 FPS make sure to change both this script and the Azure Kinect ROS Driver
    - If you are unsure whether or not the Azure Kinect and this program have matching FPS', run a sample recording and see if it seems too fast or too slow. If it is too fast/slow then follow the Azure Kinect Setup Instructions in this README starting on instruction 4

## Analysis Instructions
1. In the root of this folder (which should be `~/catkin_ws/src/Engagement_Tracking_Algorithm/`), open a terminal and run *python scripts/listener.py -a \[Duration (in seconds, optional)\]*
- Examples of commands to run this program
    - *python scripts/listener.py -a 5*
        - This will run the analysis program for 5 seconds on the camera input and then quit the program
        - NOTE: You may end the program early by hitting *ctrl+c*
    - *python scripts/listener.py -a*
        - This will run the analysis program until someone hits *ctrl+c* on the keyboard to exit the program.
NOTE: There is currently no method of analysis AND recording at the same time.
2. 15fps is what is coded into this script, however, the Azure ROS Driver default is 5. You must change the default in the Azure Kinect ROS Driver driver.launch file to 15. Other options are 5, and 30. Make sure the number in this file matches the one in the driver.launch
    - If you followed the setup instructions for the Azure Kinect then these should already match, however, if you wish to change to 5 or 30 FPS make sure to change both this script and the Azure Kinect ROS Driver
    - If you are unsure whether or not the Azure Kinect and this program have matching FPS', run a sample recording and see if it seems too fast or too slow. If it is too fast/slow then follow the Azure Kinect Setup Instructions in this README starting on instruction 4
