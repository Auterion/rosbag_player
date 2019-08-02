# Rosbag player
This repository contains a node which helps better visaulize the log data generated by the PX4 avoidance repository. Mainly the camera image are projected into 3D space for better orientation.

To use this package clone *Auterion/rviz_textured_quads* and build the branch *rosbag_player*. This package contains the RVIZ plugin necessary for the image visalization. 

Then, clone this repository, build it and use it by calling: *roslaunch rosbag_player rosbag_player.launch "bag_name":="/path/to/file/bag_name"*
