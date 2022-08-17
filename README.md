# Introduction
This packge provides auto-docking ability to any differential drive robot with a camera that could precisely spin horizontally to any degree. The system uses vision with AprilTag to locate the position and orientation of the wireless charging staton in the 3D space. Control signals are then published using ROS to motor drivers. No ROS packages were used by system, meaning it is easily adaptable to other communication protocols (ie: mqtt) with a slight modification to the publisher/subscriber related lines in the code.

# Requirements
  - Tag must be visible when robot is docked, if camera cannot see the tag after docking, place the tag further behind the charging station.
  - Tag must be vertical to the ground (no tilting forward/backward), tag could be rotated but preferably it is parallel to the charging station.
  - Make sure there is only one tag in vision while docking (make a list message around TagInfo.msg to include multiple tag detection).
  - Avoid strong light shinning on the tag, it may affect detection accuracy.

# Setup
## Calibration (monocular camera)
http://wiki.ros.org/camera_calibration/Tutorials/MonocularCalibration

Print checkerboard at https://markhedleyjones.com/projects/calibration-checkerboard-collection

Install rosdep
```
# ROS Noetic
sudo apt-get install python3-rosdep

# ROS Melodic and earlier
sudo apt-get install python-rosdep
```

Install camera_calibration package
```
rosdep install camera_calibration
```

Run streaming and calibration nodes
  - `--size` refers to the number of internal corner, as described in the OpenCV documentation (i.e. the 8x6 checkerboard contains 9x7 squares)
  - `--square` refers to the side length of squares measuered in meters
```
# terminal 1
roscore

# terminal 2
cd src
python3 calibration_stream.py

# terminal 3, only source ros, don't source ws
source /opt/ros/noetic/setup.bash
rosrun camera_calibration cameracalibrator.py --size 8x6 --square 0.108
```

Obtain calibration result
```
cd src/resources
mkdir tmp
mv /tmp/calibrationdata.tar.gz tmp
tar -xvf tmp/calibrationdata.tar.gz -C tmp
mv tmp/ost.yaml ptz_calibration.yaml
rm -r tmp
```

## OpenCV
https://docs.opencv.org/4.x/d2/de6/tutorial_py_setup_in_ubuntu.html

Build OpeCV from source to get GStreamer support
```
sudo apt-get install cmake
sudo apt-get install gcc g++

# to support python2
sudo apt-get install python-dev python-numpy

# to support python3
sudo apt-get install python3-dev python3-numpy

# GTK support for GUI features, Camera support (v4l), Media Support (ffmpeg, gstreamer), etc.
sudo apt-get install libavcodec-dev libavformat-dev libswscale-dev
sudo apt-get install libgstreamer-plugins-base1.0-dev libgstreamer1.0-dev

# to support gtk2
sudo apt-get install libgtk2.0-dev

# to support gtk3
sudo apt-get install libgtk-3-dev

# optional dependencies
sudo apt-get install libpng-dev
sudo apt-get install libjpeg-dev
sudo apt-get install libopenexr-dev
sudo apt-get install libtiff-dev
sudo apt-get install libwebp-dev
sudo apt-get install libjasper-dev
```

Download OpenCV source code
```
git clone https://github.com/opencv/opencv.git
```

Configure and install CV
```
cd opencv
mkdir build
cd build

# configure build & install parameters
cmake ../

# build the files
make

# install to system
sudo make install
```

These lines should show up in CMake output, indicate installation path of CV. Make sure to include this in python path so cv2 could be imported properly.
```
Python 2:
  Interpreter:                 /usr/bin/python2.7 (ver 2.7.18)
  Libraries:                   /usr/lib/x86_64-linux-gnu/libpython2.7.so (ver 2.7.18)
  numpy:                       /usr/lib/python2.7/dist-packages/numpy/core/include (ver 1.16.5)
  install path:                lib/python2.7/dist-packages/cv2/python-2.7

Python 3:
  Interpreter:                 /usr/bin/python3 (ver 3.8.10)
  Libraries:                   /usr/lib/x86_64-linux-gnu/libpython3.8.so (ver 3.8.10)
  numpy:                       /usr/lib/python3/dist-packages/numpy/core/include (ver 1.17.4)
  install path:                lib/python3.8/site-packages/cv2/python-3.8
```

## AprilTag
https://github.com/AprilRobotics/apriltag \
https://github.com/duckietown/lib-dt-apriltags

Use 36h11 tag family for detection, id:0 example at [src/resources/36h11_0.jpeg](src/resources/36h11_0.jpeg)

Install python binding (by DuckyTown) of the originial AprilTag library written in c (by APRIL robotics lab)
```
pip3 install dt-apriltags
```

Alternatively, you could build AprilTag library from source
```
git clone https://github.com/AprilRobotics/apriltag.git
cd apriltag
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --target install
sudo ldconfig
```

# System Overview

## AprilTag Detection Result
After providing calibration data, tag size and tag family, the system will be able to detect 3D pose of the tag, including orientation (roll, pitch, yaw) and position (x, y, z) data with camera as the origin. However, not all data are used in the system, for orientation, only yaw (also refered to as alpha in the code) is used, and for position, z is irrelavant since height is not used when lining up the robot. It is also worth noting that x is the depth axis (forward is positive) and y is the parallel axis (left is positive).

## ROS Plot
drawing

## System Diagram
drawing

## Docking Phases
drawing

# Debugging and Testing
Tag must be in vision at all time during p3, if not the docking system will abort. So if optical camera is placed to the right side of the robot when backing up, tag should also be placed on the left side of the charging station, and vice versa. If tag is not visible when robot is close to statin, change the positioning of the tag, do not tilt the camera. This means the relative position of the tag to the station could vary between different setups. Thus testing is needed to calibrate the distance parameters.

In `__init__()` in docking_oop.py, you should find

```
# distance constants in phase 2
self.p2_y_first = -0.11
self.p2_y_second = 0.21

# distance constants in phase 3
self.p3_stop_x = 0.765
```

`p2_y_first`, `p2_y_second` are the y axis (left/right) offset in phase 2 for first and second alignment respectively. They are used to make the robot line up with the charging station at the end of p2. To adjust y offset, from the tag, facing direction of charging, increase offset to make robot move right, decrease offset to make robot move left. \
`p3_stop_x` is the x value from tag detection when robot is charging. It is used to stop robot from running into the charging station. Increase to let robot come closer to station, decrease to stop robot further away.

# Design Breakdown
words