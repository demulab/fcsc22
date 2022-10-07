#!/usr/bin/env python
# -*- coding: utf-8 -*-

# pythonでROSのソフトウェアを記述するときにimportするモジュール
import rospy

# 自分で定義したmessageファイルから生成されたモジュール
from sawyer_ir.msg import Ir

def talker():
    Ircommander = rospy.Publisher('ir_shlef_controller', Ir, queue_size=1)
    rospy.init_node('irmagician_switch',anonymous=True)
    r = rospy.Rate(10)
    ircommand = Ir()
    print("ir_switch active")
    while not rospy.is_shutdown():
        key = raw_input()
        if key == 'a':
           ircommand.irsignal = "high_shelf_open"
           print("high shelf open")

        elif key == 'b':
           ircommand.irsignal = "high_shelf_close"
           print("high shelf close")

        elif key == 'c':
           ircommand.irsignal = "middle_shelf_open"
           print("middle shelf open")

        elif key == 'd':
           ircommand.irsignal = "middle_shelf_close"
           print("middle shelf close")

        elif key == 'e':
           ircommand.irsignal = "low_shelf_open"
           print("low shelf open")

        elif key == 'f':
           ircommand.irsignal = "low_shelf_close"
           print("low shelf close")
        else:
           print("input a, b, c, d, e, f")

        Ircommander.publish(ircommand)
        r.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:pass
