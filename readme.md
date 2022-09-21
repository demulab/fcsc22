## install
- smach
  - sudo apt install ros-melodic-smach-ros ros-melodic-smach-viewer 
- Lidar
  - sudo apt install ros-melodic-urg-nodegi
- PS4con
  - sudo pip install ds4drv
  - sudo apt install ros-melodic-joy
- Realsense
  - https://demulab.esa.io/posts/11
- Scout mini omni
  - sudo apt install -y ros-melodic-teleop-twist-keyboard
  - sudo apt install --no-install-recommends libasio-dev
  - sudo apt install ros-melodic-pointcloud-to-laserscan
  - sudo apt-get install -y ros-melodic-move-base
- moveit
  - sudo apt-get install ros-melodic-moveit* 
- rosserial
  - sudo apt-get install ros-melodic-rosserial-arduino
  - sudo apt-get install ros-melodic-rosserial

## xArm6
- cd ~/your_ws/src/OPL22/
- git clone -b HappyDaisy git@github.com:demulab/xArm.git
- cd xArm
- git pull
- git submodule sync
- git submodule update --init --remote
- rosdep update
- rosdep check --from-paths . --ignore-src --rosdistro melodic
- cd ../../../
- catkin build xarm_gripper
- catkin build

## scout mini omni
- cd ~/your_ws/src/OPL22
- git clone git@github.com:demulab/scout_mini_omni.git
- catkin build
```
sudo apt install --no-install-recommends libasio-dev
```
https://github.com/christianrauch/msp/issues/4
![image](https://user-images.githubusercontent.com/42795206/158987097-937f1845-b693-4c22-ba0b-e35aed75b102.png)
```
sudo apt install ros-melodic-pointcloud-to-laserscan
```
![image](https://user-images.githubusercontent.com/42795206/158987417-b25ee522-a6eb-4791-86b8-420f3312c2e0.png)

## darknet_ros
- cd ~/your_ws/src/OPL22
- git clone --recursive https://github.com/leggedrobotics/darknet_ros.git
- catkin build
