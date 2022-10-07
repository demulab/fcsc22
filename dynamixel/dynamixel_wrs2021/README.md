## Description  
WRS2021機体で使用するエンドエフェクタを制御するためのパッケージ  
  
## Install  
DYNAMIXELパッケージのインストール  
```  
$ cd ~/xxx_ws/src
$ git clone https://github.com/ROBOTIS-GIT/dynamixel-workbench.git  
$ git clone https://github.com/ROBOTIS-GIT/dynamixel-workbench-msgs.git  
$ git clone https://github.com/ROBOTIS-GIT/DynamixelSDK.git
```  
  
本パッケージのインストール
```  
$ cd ~/xxx_ws/src
$ git clone https://github.com/demulab/dynamixel_wrs2021.git
```  
  
## Start Up  
- モータを制御するドライバの起動  
`$ roslaunch dynamixel_wrs2021 motor_setup.launch`  
- モータ制御プログラムの起動  
`$ rosrun dynamixel_wrs2021 motor_controller.py`  
  
## Usage  
他ノードからの使用方法  
|Communication|Name|Type|  
|:--|:--|:--|  
|Topic通信|/servo/endeffector|Bool|  
  
開く:rostopic pub /servo/endeffector std_msgs/String OPEN  
閉じる:rostopic pub /servo/endeffector std_msgs/String CLOSE  
  
