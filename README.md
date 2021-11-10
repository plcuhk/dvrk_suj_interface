# Overview
This package provide a control interface for SUJs 

# Dependency
- Software: This package requires the installation of ROS.
- hardware: This package requires the SUJ control board from CUHK-BMT
# install
open a terminal, type
```sh
cd ~
mkdir -p ros-ws/src
cd ros-ws/src
git clone https://github.com/CUHK-BRME/dvrk_suj_interface.git
cd ..
catkin_make
echo 'source ~/ros-ws/devel/setup.bash' >> ~/.bashrc 
```

# run 
- Hardware: connect 3 USB from SUJ control board to your PC
- Software:

1. open one terminal ,type
```sh
sudo adduser $USER dialout
```
and log out your computer after this.




2. open one terminal, type
```sh
roscore
```
oepen another terminal, type
```sh
rosrun dvrk_suj_interface dvrk_suj_publisher_v2.py
```


# Test
To test if your program is reading from sensors correctly, you can open a terminal and type
```sh
rostopic echo -c /dvrk/SUJ/ECM/set_position_joint
```
