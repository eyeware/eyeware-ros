# GazeSense™ ROS

The GazeSense™ bridge exposes its real-time eye and attention tracking data into the Robot Operating System (ROS).

## Description

GazeSense™ is an application developed by Eyeware Tech SA (<http://www.eyeware.tech>) that provides 3D eye tracking by relying on consumer 3D sensors. GazeSense™ allows to define virtual 3D objects with respect to the camera and world coordinate systems and measure the attention of people towards the objects.

The bridge currently implements an example in which the attention sensing objects are defined in terms of 3D primitives (eg. planes, cylinders, points). The tracking parameters, like head pose, as well as the measurement of attention towards the virtual 3D objects, are then published as the topic *gs_persons*. Other ROS nodes can subscribe receiving this feed information. Currently, the GazeSense™ bridge does not yet support reading RGBD camera data from a ROS node. Should this feature be of help for you, reach out to us to let us know via an email to products@eyeware.tech or by simply submitting your request in our Trello Eyeware Products Board / Ideas & Requests (<https://trello.com/b/HLiqqYs4/eyeware-products>).
Markers are also provided for visualization within rviz (<http://wiki.ros.org/rviz>).

## Dependencies

You will need, besides the usual dependencies of a python ROS node, pyzmq (<https://github.com/zeromq/pyzmq>). You can install it through pip or conda.

However, if you would like to avoid installing directly into your main python or avoid conflicts between virtual environments, conda and ROS, an alternative is to build a temporary virtual environment and then point your python path to it, as follows:

```
virtualenv gazesense_temp -p python2
pip install pyzmq==16.0
export PYTHONPATH=YOUR_VIRTUAL_ENV/lib/python2.7/site-packages:$PYTHONPATH
```

## Building the node

```bash
source  /opt/ros/<YOUR_ROS_DISTRO>/setup.bash
mkdir -p ~/gazesense_ws/src
catkin_init_workspace ~/gazesense_ws/src
git -C ~/gazesense_ws/src clone github/eyeware-ros
catkin_make -C ~/gazesense_ws/
```

## Running the GazeSense ROS bridge node

```bash
export PYTHONPATH=PATH_TO_GAZESENSE_INSTALL_FOLDER/API/python
roslaunch gazesense_bridge gazesense_ros_bridge.launch host_ip:=192.168.1.12 frame_id:=camera_link

```
*host_ip* is the IP of the machine running GazeSense. 

<!-- 
## How to change the user (Not yet supported)

```bash
rosservice call /change_user "K1"
``` -->
