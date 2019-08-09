# Installation

It has only one way at this time (build from source in Catkin workspace) to install the packages in this repository.

## Prerequisites

* Ubuntu 16.04.6 LTS
* ROS Kinetic Kame
* (Optional) Nvidia GPU Driver
* realsense D435 driver [link](https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md#installing-the-packages)
* pip: `$ sudo apt install python-pip`


## Building from Source

### Retrive the sources

    $ cd $HOME/catkin_ws/src
    $ git clone https://github.com/byeongkyu/living_lab_robot_v2


### Install Dependencies (ROS packages)

    $ cd $HOME/catkin_ws/
    $ wstool init src
    $ wstool merge -t src ./src/living_lab_robot_v2/doc/living_lab_robot.rosinstall
    $ cd src
    $ wstool update


### Install dependency packages

    $ cd $HOME/catkin_ws/src
    $ rosdep install --from-paths . --ignore-src -r -y
    $ find -name 'requirements.txt' | xargs -L 1 sudo pip install -U -r


### Build

    $ cd $HOME/catkin_ws
    $ catkin build or catkin_make

