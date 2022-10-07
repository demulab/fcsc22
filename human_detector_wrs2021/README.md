# sawyer_human_detector
陳列廃棄タスクの人検知プログラム
# 使い方
・2Dlidarのデバイス名を/ttyACM_URG0と/ttyACM_URG1にUDEVで設定し、launchファイルを起動

roslaunch sawyer_human_detector twolidar.launch

・音声用のノード立ち上げ

rosrun sawyer_human_detector sawyer_emergency_sound.py

・人検知用のノード立ち上げ

rosrun sawyer_human_detector sawyer_human_detector_1.py

rosrun sawyer_human_detector sawyer_human_detector_2.py

rosrun sawyer_human_detector sawyer_emergency_controll.py
