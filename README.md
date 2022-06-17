# Engagement Queue Tracking Algorithm
## Setup instructions
NOTE: You must be using Ubuntu 18.04 (The Azure Kinect does not currently support Ubuntu 20.04 or newer, and neither does it's ROS driver)
1. Install ROS Melodic using [this link](https://wiki.ros.org/melodic/Installation/Ubuntu) and [this link](https://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment) for setting it up
2. Install the [Azure Kinect SDK](https://docs.microsoft.com/en-us/azure/kinect-dk/sensor-sdk-download) onto your Linux system. Make sure to follow ALL instructions, there are multiple webpages with instructions required for it to work on Linux.
3. Install the [Azure Kinect ROS Driver](https://github.com/microsoft/Azure_Kinect_ROS_Driver) using the Building Guide (near the bottom of the README). Clone that repo in catkin_ws/src/ and then run "catkin_make" in catkin_ws/ For usage information and a list of topics go to [this link](https://github.com/microsoft/Azure_Kinect_ROS_Driver/blob/melodic/docs/usage.md)
4. INSERT INFO FOR BOSON
5. Clone this repo into catkin_ws/src/ and run "catkin_make" in catkin_ws/ 

Unfinished