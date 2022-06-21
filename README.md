# Engagement Queue Tracking Algorithm
## Setup instructions
### Unfinished
NOTE: You must be using Ubuntu 18.04 (The Azure Kinect does not currently support Ubuntu 20.04 or newer, and neither does it's ROS driver)
1. Install ROS Melodic using [this link](https://wiki.ros.org/melodic/Installation/Ubuntu) and [this link](https://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment) for setting it up
2. Install the [Azure Kinect SDK](https://docs.microsoft.com/en-us/azure/kinect-dk/sensor-sdk-download) onto your Linux system. Make sure to follow ALL instructions, there are multiple webpages with instructions required for it to work on Linux.
3. Install the [Azure Kinect ROS Driver](https://github.com/microsoft/Azure_Kinect_ROS_Driver) using the Building Guide (near the bottom of the README). Clone that repo in catkin_ws/src/ and then run "catkin_make" in catkin_ws/ For usage information and a list of topics go to [this link](https://github.com/microsoft/Azure_Kinect_ROS_Driver/blob/melodic/docs/usage.md)
4. INSERT INFO FOR BOSON
5. Clone this repo into catkin_ws/src/ and run "catkin_make" in catkin_ws/ 

## Recording Instructions
1. In the root of this folder, open a terminal and run "python scripts/listener.py \[Mode\] \[Seconds\]" and replace \[Seconds\] with the number of seconds you want the program to run.
- Possible modes:
    - "-r" for recording
    - "-a" for analysis (currently only displays the camera feed to the screen)
2. Do not ctrl+C the program in recorder mode, this will corrupt the output video file.
3. The output video will be in the output folder after the recording is finished.
4. 15fps is what is coded into this script, however, the Azure ROS Driver default is 5. You must change the default in the Azure Kinect ROS Driver driver.launch file to 15. Other options are 5, and 30, make sure the number in this file matches the one in the driver.launch. 

### Known bugs
1. RGB values are reversed (FIXED)
2. Output is .avi instead of .mp4
3. Requires a duration instead of being quit with the keyboard
4. Program is currently set for recording only, no analysis, no facial detection, no live output (FIXED)