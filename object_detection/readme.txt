============================================================
== Object Detection using LiDAR point cloud 
============================================================

Object detection, tracking, and classification using LiDAR point cloud

- Current implementation: ROS with Python
- Classifier is not optimized yet


============================================================
== What's included 
============================================================

object_detection
+-- include
|   +-- object_detection (empty)
+-- launch
|   +-- object_detection.launch
+-- rviz_cfg
|   +-- obj_detect.rviz
+-- scripts
|   +-- classification_svm.py
|   +-- connected.py
|   +-- features_extraction.py
|   +-- object_detection_node.py
|   +-- occupancy_grid.py
|   +-- scan_conversion.py
|   +-- segmentation.py
|   +-- trained_classifier_4classes.pkl
+-- src (empty; c++ goes here)
+-- CMakeLists.txt
+-- package.xml
+-- readme.txt (this file)
+-- requirements_python36.txt


============================================================
== Getting Started 
============================================================

Prerequisites:

-Ubuntu 16.04 with ROS Kinetic (Installation instruction: http://wiki.ros.org/kinetic/Installation/Ubuntu)
-Catkin workspace (how to create catkin workspace: http://wiki.ros.org/catkin/Tutorials/create_a_workspace)
-Python 3.6 (tested with Python 3.6, but 3.5 should work)
-Phtyon libraries: numpy, scipy, scikit-learn, etc. 
 (requirements file (requirements_python36.txt) is provided just in case; create your virtual environment, and run: pip install -r requirements_python36.txt)

How to build with catkin:

$ unzip ros_object_detection.zip -d ~/catkin_ws/src/
$ cd ~/catkin_ws
$ catkin_make
$ source ~/catkin_ws/devel/setup.bash

Python scripts are under ~/catkin_ws/src/object_detection/scripts


============================================================
== How to run
============================================================

$ roslaunch object_detection object_detection.launch

(This launches three separete programs, which can be run in three separete terminals as follows:
$ roscore
$ rosrun object_detection object_detection_node.py
$ rosrun rviz rviz -d ~/catkin_ws/src/object_detection/rviz_cfg/obj_detect.rviz
)

In second terminal, play sample rosbag file including LiDAR data (VLP-32C):
$ rosbag play sample_rosbag.bag

Note: Please find the links below for ROSBAG with Velodyne LiDAR Data and Camera

      https://app.box.com/s/nueax6f2ylcspf796xk13xnoxb2zra71

      https://velodyne-my.sharepoint.com/:f:/p/algorithmteam/EruWWshc4gtCqxHYCNYy4JEBq3GGg8QM8RpPssV6AbxjsA?e=qrbAeq
      * 2017-10-01-19-54-57_Velodyne-VLP-16-Data.pcap file and _2017-12-12-21-58-53_CESDemo.bag file are VLP-32 bag file.
      * 2017-10-16-13-23-15_Alameda_Marina.pcap file is VLP-16 bag file.

