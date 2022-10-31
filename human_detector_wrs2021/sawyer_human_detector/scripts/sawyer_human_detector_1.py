#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import tf
import time
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
import cv2 
import sys
import numpy as np
from enum import Enum
import math
from matplotlib import pyplot as plt
import copy
from sawyer_human_detector.msg import Emergency

def DEG2RAD(DEG): return math.pi*DEG/180.0
def RAD2DEG(RAD): return 180.0*RAD/math.pi

g_find_leg_radius = 20.0                #検出範囲の中心座標から半径[px]
kExtendRadius = 100.0                    #見失ったときの検出範囲[px]
kOrignalRadius = g_find_leg_radius      
kMagnificationWorldImagePos = 10.0      #画像上で物体同士が重ならないようにするための世界座標の倍率
kUpdateLastImageCount  = 30.0           #比較する世界座標系の画像を更新するループ回数　one loop 30[ms]

kTrackingMaxDistance = 2.5                #検出距離の最大
kTrackingMinDistance = 0.1                #検出距離の最小
kTrackingAngle       = 125              #探す範囲は正面のこの角度[deg]
kDefaultDetectPosX	= 250.0         #検出範囲のxの初期の中心座標[px]
kDefaultDetectPosY = 250.0              #検出範囲のyの初期の中心座標[px]
kLostTime          = 60.0               #人を完全に見失ったと判断するループ回数、one loop 30[ms]
kLegBetweenDistance = 0.6               #人の足だと判断する足候補感の距離[m]

kImageWidth=500                         #[px]
kImageHeight=500                        #[px]

kMToPixel = 50.0                        #mをpixilへ変換 1[m] = 50 pixel 1[pixel] = 2[cm]

#白
firstColor = 255

#初期化時に塗りつぶす
lidar_image = np.zeros((kImageWidth,kImageHeight,3),np.uint8)
lidar_gray_image = np.zeros((kImageWidth,kImageHeight),np.uint8)+firstColor

#脚候補の世界系座標
world_pose_candidata_leg_image = np.zeros((500,500),np.uint8)+firstColor

#ライダーからのデータを世界座標系にした画像
world_pose_image = np.zeros((500,500),np.uint8)+firstColor
last_world_pose_image = np.zeros((500,500),np.uint8)+firstColor
erode_last_world_pose_image = np.zeros((500,500),np.uint8)+firstColor
opening_last_world_pose_image = np.zeros((500,500),np.uint8)+firstColor

#世界座標系の画像の差分画像
substraction_world_pose_image = np.zeros((500,500),np.uint8)+firstColor

#検出範囲外の脚候補の位置を世界座標系にして描写する画像
static_object_world_pose_image = np.zeros((500,500),np.uint8)+firstColor
erode_static_object_world_pose_image = np.zeros((500,500),np.uint8)+firstColor
opening_static_object_world_pose_image = np.zeros((500,500),np.uint8)+firstColor

#LiDAR画像に縮小処理
lidar_erode_image = np.zeros((500,500),np.uint8)

#人検知に用いる変数
human_distance = 999.0
human_candidate_distance = 999.0
human_angle = 0

#色
red = (0,0,255)
blue = (255,0,0)
green = (0,255,0)

SARE = 0
DANGER = 119

#「人についてのクラス」
#人までの距離や角度の情報をメンバとして宣言

class Pose():
    x = 0.0
    y = 0.0
    z = 0.0
    theta = 0.0

class Object():
    #初期化コンストラクタ
    def __init__(self):
        #c++ではprivate
        self._x_ = 0.0
        self._y_ = 0.0
        self._z_ = 0.0
        self._theta_ = 0.0

        #ローカル座標、ワールド座標
        self.local = Pose()
        self.world = Pose()

        #反射強度の最小、最大値[0:255]
        self.intensity_min = 0
        self.intensity_max = 255

        #距離
        self.distance = 0.0
        self.last_distance = 0.0
        #角度[rad/s]
        self.angle = 0.0
        self.last_angle = 0.0

        #半径
        self.radius = 0.0
        #画像座標系での位置
        self.image_pos = Pose()

        #yamada
        #１時刻前の人の位置
        self.last_human_x = kDefaultDetectPosX
        self.last_human_y = kDefaultDetectPosY
        #予測される人の位置
        self.image_expect_human_x = self.image_expect_human_y = 0.0
        #動体と予測した物体の位置
        self.dynamic_image_pos = Pose()

        #動体かの判断
        self.judge_dynamic = False

    def getX(self): return self._x_
    def getY(self): return self._y_
    def getZ(self): return self._z_
    def getTheta(self): return self._theta_
    def setX(self,x): self._x_ = x
    def setY(self,y): self._y_ = y
    def setZ(self,z): self._z_ = z
    def setTheta(self,theta): self._theta_ = theta

human = Object()

#「ロボットやセンサについてのクラス」
# LiDARの情報やロボットの情報をメンバとして宣言

class Robot():
    #初期化コンストラクタ
    def __init__(self):
        #人を見失った回数
        self.lost_count = 0

        #人を見失っているか
        self.human_lost = True
        #位置[m],向き[rad]
        self.robot_x = self.robot_y = 0.0
        self.robot_theta = 0.0
        #hokuyo LiDAR UTM-30LX
        self.laser_distance = np.zeros(1081)
        self.laser_last_distance = np.zeros(1081)
        self.laser_intensities = np.zeros(1081)
        self.laser_last_intensities = np.zeros(1081)

        self.human_approach_angle = 0.0
        self.human_approach_distance_cos = 0.0
        self.human_approach_distance_sin = 0.0
        self.restart = False
        self.comeback = False
        self.restart_count = 0
        self.comeback_count = 0
        self.approach_count = 0
        self.detector_count = 0
        self.sleep_time = 0

        #画像のサイズを超えて書き込もうとした際、その座標を中心からの座標に更新するための値
        self.displace_x = self.displace_y = 0.0
        #このカウントがkUpdateLastImageCountと同じになればlast_world_pose_imageを更新、ver2
        self.last_image_count = 0
        self.step = 0

        self.laser_sub = rospy.Subscriber("/lidar1/scan1",LaserScan,self.laserCallback)
        self.laser_sub = rospy.Subscriber("sawyer_to_lidar",Emergency,self.robotMecanumMoveJudge)
        #public
        #ローカル座標
        self.local = Pose()
        self.world = Pose()
        self.dataCount = 0
        self.laser_angle_min = 0
        self.laser_angle_max = 0
        self.tracking_command = "false"

    '''
    laser_callbackの重い部分
    '''

    def laser_cycle(self):
        #callback関数に一度でも入っていたら処理をする
        if not (self.dataCount == 0):
            #LiDARのデータを画像に変換
            self.changeToPicture(self.dataCount,self.laser_angle_min,self.laser_angle_max)
            
            self.laser_last_distance = self.laser_distance.copy
            self.laser_last_intensities = self.laser_intensities.copy

            self.prepWindow()

    def welcomeMessage(self):
        rospy.loginfo("Tracking me program by demura lab.,KIT")
        rospy.loginfo(kTrackingMaxDistance)
        rospy.loginfo(kTrackingMinDistance)
        rospy.loginfo(kTrackingAngle)

    '''
    脚の探索範囲の中心位置を初期位置に戻す関数
    human_obj   ： 1時刻前の人の一を使うので引数にする
    時刻前の脚の位置を初期化する
    '''

    def defaultPos(self,human_obj):
        #グローバル宣言
        global static_object_world_pose_image,opening_static_object_world_pose_image
        #1時刻前の座標
        human_obj.last_human_x = kDefaultDetectPosX
        human_obj.last_human_y = kDefaultDetectPosY
        #1時刻前の速度
        human_obj.last_linear_speed = 0.0

        static_object_world_pose_image = np.zeros((500,500),np.uint8)+firstColor
        opening_static_object_world_pose_image = np.zeros((500,500),np.uint8)+firstColor

    '''
    画像に世界座標系にしたライダーからのデータを書き込む関数
    point_x point_y 書き込む座標
    画像より大きい座標を書き込む場合、初期位置(画像の中心)にまでずらして書き込む
    ROS似合わせているので、xが縦軸、yが横軸
    '''
    def writeWorldPoseImages(self,point_x,point_y):
        #グローバル関数使用宣言
        global static_object_world_pose_image,world_pose_image

        if point_x < 0:
            #静止物体の位置を示す画像の初期化
            static_object_world_pose_image = np.zeros((500,500),np.uint8)+firstColor
            #初期位置(画像の中心)からの座標にずらす
            self.displace_x += kImageHeight/2.0
            point_x += self.displace_x
            #世界座標系にしたLiDARデータを書き込む
            world_pose_image[int(point_x)][int(point_y)] = 0 
        elif point_x > kImageHeight:
            static_object_world_pose_image = np.zeros((500,500),np.uint8)+firstColor
            self.displace_x -= kImageHeight/2.0
            point_x -= self.displace_x
            world_pose_image[int(point_x)][int(point_y)] = 0
        elif (point_x >= 0) and (point_x <= kImageHeight) and (point_y < 0):
            static_object_world_pose_image = np.zeros((500,500),np.uint8)+firstColor
            self.displace_y += kImageWidth/2.0
            point_y += self.displace_y
            world_pose_image[int(point_x)][int(point_y)] = 0
        elif (point_x >= 0) and (point_x <= kImageHeight) and (point_y > kImageWidth):
            static_object_world_pose_image = np.zeros((500,500),np.uint8)+firstColor
            self.displace_y -= kImageWidth/2.0
            point_y -= self.displace_y
            world_pose_image[int(point_x)][int(point_y)] = 0
        else:
            #世界座標系に変換したLiDARからのデータを書き込む
            world_pose_image[int(point_x)][int(point_y)] = 0

    '''
    opencvのAPIを使うためにLiDARデータを画像に変換する関数
    dataCount：走査線数
    laser_angle_min,laser_angle_max LiDARの走査線の一番右(最小)と一番左(最大)
    lidar_image(500*500[px])の範囲に反応があったら白(255),なければ黒(0)をlidar_gray_imageの各要素に入力する
    '''

    def changeToPicture(self,dataCount,laser_angle_min,laser_angle_max):
        #グローバル関数使用宣言
        global lidar_gray_image,world_pose_candidata_leg_image,world_pose_image,last_world_pose_image
        global opening_last_world_pose_image

        #画像を黒一色で初期化
        lidar_gray_image = np.zeros((500,500),np.uint8)

        #中央の走査線番号
        center = dataCount/6*4
        #kTrackingAngle(前方90度)の走査線数
        #270
        search_lines = int(1080*(kTrackingAngle/270.0))

        local = Pose()
        world = Pose()

        for j in range(int(center - search_lines/2),int(center + search_lines/2)+1):
            #捜査線の角度
            angle = (laser_angle_max-laser_angle_min)*j/float(dataCount)+laser_angle_min-1.5708
            #画像座標系に変換
            tmp = int(kMToPixel * self.laser_distance[j] * math.cos(angle))
            #rosは進行方向がx,横がy
            x = int(kImageWidth/2 - kMToPixel * self.laser_distance[j] * math.sin(angle))   #x軸は左右反転
            y = int(kImageHeight/2 - tmp)                                                   #y軸は上下反転

            #VER2
            #ローカル座標系はROSに合わせて進行方向がx,左方向がy
            local.y = float(x - kImageWidth/2)/float(kMToPixel)
            local.x = float(y - kImageHeight/2)/float(kMToPixel)
            #ワールド座標系に変換
            self.localToWorld(local,world)

            #cols:横画素数500,rows：縦画素数500
            if (0 <= x) and (x < lidar_image.shape[1]) and (0 <= y) and (y < lidar_image.shape[0]):
                #6000は反射強度の最大値
                value = int(self.laser_intensities[j]*255.0/6000.0)
                if value > 255: 
                    value = 255
                if value < 0:
                    value = 0

                #グレースケール画像に正規化した反射強度valueを代入
                lidar_gray_image[int(y),int(x)] = value

                #VER2
                #世界座標系
                point_x = kMagnificationWorldImagePos * world.x + kImageWidth/2 + self.displace_x
                point_y = kMagnificationWorldImagePos * world.y + kImageHeight/2 + self.displace_y
                self.writeWorldPoseImages(point_x,point_y)

        #VER2
        #動体を検出するための処理
        #差分をとるワールド座標系の画像をlast_image_countごとに更新
        if self.last_image_count >= kUpdateLastImageCount:
            last_world_pose_image = copy.deepcopy(world_pose_image)
            #画像を収縮して膨張、ノイズを除去
            opening_last_world_pose_image=cv2.morphologyEx(last_world_pose_image,cv2.MORPH_OPEN,np.ones((3,3),np.uint8),anchor = (-1,-1),iterations = 1)
            self.last_image_count = 0
            #画像を初期化
            world_pose_image = np.zeros((500,500),np.uint8)+firstColor
            world_pose_candidata_leg_image = np.zeros((500,500),np.uint8)+firstColor

        self.last_image_count+=1

    '''
    LiDARデータを受け取るコールバック関数
    data:トピック/lidar2/scan2をサブスクライブしている
    LiDARの捜査線数や距離、反射強度の値を変数に代入。LiDARデータをchangeToPictureに渡す
    '''

    def laserCallback(self,data):
        #self.step = -1
        if self.step < 0 or self.step >= 1 and self.step < 80:
            self.comeback_count = 0
            self.a = 0
            #捜査線数
            self.dataCount = np.size(data.ranges)
            #LiDARの一番右の走査線-2.356194[rad] -135度
            self.laser_angle_min = data.angle_min
            #LiDARの一番右の走査線 2.356194[rad]  135度
            self.laser_angle_max = data.angle_max
            #最大60[m] 最小0.023[m]
            self.laser_distance = np.array([999 if (value <= data.range_min or value >= data.range_max) else value for value in data.ranges])
            #反射強度
            self.laser_intensities = np.array([-999 if self.laser_distance[i] == 999 else data.intensities[i] for i in range(self.dataCount)])

    '''
    ローカル座標系をワールド座標系に変換
    local_pose:ローカル座標
    world_pose:ワールド座標
    ワールド座標系での物体の位置＝回転行列*ローカル座標の物体の位置 - ローカル座標の原点(ワールド座標でのロボットの位置)
    '''

    def localToWorld(self,local_pose,world_pose):
        world_pose.x = local_pose.x * math.cos(self.robot_theta) - local_pose.y * math.sin(self.robot_theta) - self.robot_x
        world_pose.y = local_pose.x * math.sin(self.robot_theta) + local_pose.y * math.cos(self.robot_theta) - self.robot_y
        world_pose.theta = self.robot_theta

    '''
    入力画像から脚候補の数を探索する関数
    input_image     : 入力画像
    objects         : 脚候補のオブジェクト
    display_image   : 出力画像
    color           : 色
    contour_min     : 輪郭長            ：最小[px]
    contour_max     : 輪郭長            ：最大[px]
    width_min       : 外接矩形          ：最小[px]
    width_max       : 外接矩形          ：最大[px]
    ratio_min       : 矩形横縦          ：最小[px]
    ratio_max       : 矩形横縦          ：最大[px]
    m00_min         : 0次モーメント     ：最小[px]
    m00_max         : 0次モーメント     ：最大[px]
    m01_min         : 1次モーメント     ：最小[px]
    m01_max         : 1次モーメント     ：最大[px]
    diff_x          : 外接円と重心の差
    diff_y          : 外接円と重心の差
    各パラメータを調節して脚候補となるオブジェクトを調節できる
    脚候補は輪郭で判断する、条件に合致した輪郭を脚候補として数え合計の値を返す
    動体を検出する処理を追加、予めわかっている脚に似た物体を脚候補から除外
    '''

    def findLegs(self,input_image,objects,display_image,color,contour_min, contour_max,width_min,width_max,ratio_min,ratio_max,m00_min,m00_max,m10_min,m10_max,diff_x,diff_y):
        #グローバルの使用宣言
        global lidar_gray_image,substraction_world_pose_image,world_pose_candidata_leg_image,opening_last_world_pose_image

        #輪郭の探索
        image, contours, hierarchy = cv2.findContours(input_image,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE,offset = (0,0))
        
        #脚候補の数
        object_num = 0

        for cn in range(0,len(contours)):
            tmp = 0
            count = 0
            intensity = 0
            #最小外接円の中心
            center = Pose()
            #輪郭の最小外接円を取得
            (center.x,center.y),radius = cv2.minEnclosingCircle(contours[cn])
            #ロボットより後ろは除外
            if center.y - kImageHeight/2.0 > 0:
                continue

            #輪郭の長さにより除外
            if not ((len(contours[cn]) >= contour_min) and (len(contours[cn]) <= contour_max)):
                continue

            #kTrackingMaxDistance * kMToPixelより遠い物体は検出しない
            if kTrackingMaxDistance * float(kMToPixel) < kImageHeight/2.0 - center.y:
                continue
            
            #外接する長方形を求める
            rect_x,rect_y,rect_w,rect_h = cv2.boundingRect(contours[cn])

            #長方形の底面による除外
            if not ((rect_w >= width_min) and (rect_w <= width_max)):
                continue

            #縦横比による除外
            if not rect_w == 0:
                ratio = float(rect_h)/float(rect_w)
                if not ((ratio >= ratio_min) and (ratio <= ratio_max)):
                    continue

            #面積による除外
            mom = cv2.moments(contours[cn])
            
            if not ((mom['m00'] > m00_min) and (mom['m00'] < m00_max)):
                continue
            if not ((mom['m10'] > m10_min) and (mom['m10'] < m10_max)):
                continue

            #重心による判定
            #脚(円柱)の断面はu字なので重心のy座標が苑より下になる
            #x座標は中心近辺。中心からずれている脚は追従しない

            point_x = mom['m10']/mom['m00']
            point_y = mom['m01']/mom['m00']

            

            if center.y - point_y > diff_y:
                continue

            #反射強度による除外
            for i in range(rect_y,rect_y + rect_h):
                for j in range(rect_x,rect_x + rect_w):
                    #(i,j)での反射強度
                    tmp = lidar_gray_image[i,j]
                    if not tmp == 0:
                        count+=1
                        intensity += tmp

            #輪郭の平均輝度
            if not count == 0: 
                intensity /= count 
            else:
                intensity = 0

            #反射強度による除外
            if not ((intensity > human.intensity_min) and (intensity < human.intensity_max)):
                continue

            objects[object_num].radius = radius
            objects[object_num].image_pos.x = center.x
            objects[object_num].image_pos.y = center.y
                        
            #ローカル座標系はROS似合わせて進行方向がx,左方向がy
            objects[object_num].local.y = (center.x - kImageWidth/2)/float(kMToPixel)
            objects[object_num].local.x = (center.y - kImageHeight/2)/float(kMToPixel)

            #脚候補のオブジェクトをワールド座標系に変換
            self.localToWorld(objects[object_num].local,objects[object_num].world)
            objects[object_num].setX(objects[object_num].world.x)
            objects[object_num].setY(objects[object_num].world.y)
            objects[object_num].setTheta(objects[object_num].world.theta)

            #VER2
            world_x = kMagnificationWorldImagePos * objects[object_num].world.x + kImageWidth/2 + self.displace_x
            world_y = kMagnificationWorldImagePos * objects[object_num].world.y + kImageHeight/2 + self.displace_y

            #ROSに合わせているので、行がx、列がy
            #座標の画素値が0の場合それは静止物体と判断できる
            if opening_static_object_world_pose_image[int(world_x)][int(world_y)] == 0:
                continue
            
            #脚候補オブジェクトの世界座標系での位置の画素値を0にする。
            world_pose_candidata_leg_image[int(world_x)][int(world_y)] = 0

            #脚の可能性が高い領域に対して矩形で表示
            cv2.rectangle(display_image,(rect_x,rect_y),(rect_x+rect_w,rect_y+rect_h),(0,255,0),1)

            object_num += 1

        #VER2
        #動体を検出するための処理
        #1時刻前の画像と現在の画像の差分
        substraction_world_pose_image = ~(opening_last_world_pose_image - world_pose_candidata_leg_image)

        #動体の位置を検出
        for i in range(0,object_num):
            #脚候補のオブジェクトが動体かどうか
            objects[i].judge_dynamic = False
            world_x = int(kMagnificationWorldImagePos * objects[i].world.x + kImageWidth/2 + self.displace_x)
            world_y = int(kMagnificationWorldImagePos * objects[i].world.y + kImageHeight/2 + self.displace_y)
            
            #この座標の画素値が0であれば動体
            pixel_value = substraction_world_pose_image[int(world_x)][int(world_y)]

            if pixel_value == 0:
                #動体であればlidar_imageに描画する
                cv2.circle(lidar_image, (int(objects[i].image_pos.x), int(objects[i].image_pos.y)), 3, (200,200,0), -1,16)
                #動体の座標
                objects[i].dynamic_image_pos.x = objects[i].image_pos.x
                objects[i].dynamic_image_pos.y = objects[i].image_pos.y
                objects[i].judge_dynamic = True

        return object_num

    '''
    人の位置を推定する関数(ローカル座標系）
    object_num  : 脚候補の数
    object      : 脚候補のオブジェクト
    human_obj   : 人の位置 
    脚候補のオブジェクトから人の角度や距離を推定する
    脚の条件は2個の脚候補同士の距離が0.6[m]より離れていないこと、検出範囲の中に脚の重心が存在すること。
    移動量から脚の位置を予測
    動体を検出、予めわかっている脚に似た静止物体の位置を画像に描画
    '''
    def calcHumanPoseVer2(self,object_num, object, human_obj):
        #グローバルの宣言
        global lidar_image,static_object_world_pose_image,erode_static_object_world_pose_image
        global opening_static_object_world_pose_image,g_find_leg_radius,approach_count

        '''
        人の位置推定アルゴリズム
        物体数0：ロスト
        物体数1：ロスト
        物体数2以上：2個の脚の中心、検出範囲から外れているまたは、2つの重心が60[cm]以上離れていると除外する
        '''

        leg1_distance = 999999999.0
        leg2_distance = 999999999.0
        led1_num = 999.0
        leg1_point = Pose()
        leg2_point = Pose()
        #yamada,ver2
        last_diff_distance = 999.0

        if object_num == 0 or object_num == 1:
            human_obj.distance = 999.0
            human_obj.angle = 999.0
            human_obj.local.x = 999.0
            human_obj.local.y = 999.0
            human_obj.setX(999.0)
            human_obj.setY(999.0)
            human_obj.setTheta(999.0)
            human_obj.image_pos.x = 999.0
            human_obj.image_pos.y = 999.0
        else:
            #2個以上
            #1個目の脚を探索\
            for i in range(0,object_num):
                #オブジェクトが動体かどうか、ver
                if object[i].judge_dynamic == True:
                    #画像の中心位置からオブジェクトまでの距離
                    image1_distance = (object[i].dynamic_image_pos.x - kImageWidth/2)**2 + (object[i].dynamic_image_pos.y - kImageHeight/2)**2
                    #予測した座標からオブジェクトまでの距離
                    obj_radius = math.sqrt((object[i].dynamic_image_pos.x - human_obj.image_expect_human_x)**2 + (object[i].dynamic_image_pos.y - human_obj.image_expect_human_y)**2)
                else:
                    #動体が検出できなかった場合
                    image1_distance = (object[i].image_pos.x - kImageWidth/2)**2 + (object[i].image_pos.y - kImageHeight/2)**2
                    obj_radius = math.sqrt((object[i].image_pos.x - human_obj.image_expect_human_x)**2 + (object[i].image_pos.y - human_obj.image_expect_human_y)**2)

                #座標が検出範囲の中にあるか
                if obj_radius < g_find_leg_radius:                
                    led1_num = i
                    leg1_distance = image1_distance
                    #動体かどうか
                    if object[i].judge_dynamic == True:
                        leg1_point.x = object[i].dynamic_image_pos.x
                        leg1_point.y = object[i].dynamic_image_pos.y
                    else:
                        leg1_point.x = object[i].image_pos.x
                        leg1_point.y = object[i].image_pos.y
                else:
                    #検出範囲外の脚候補は静止物体であると判断
                    if not object[i].judge_dynamic == True:
                        world_x = int(kMagnificationWorldImagePos * object[i].world.x + kImageWidth/2 + self.displace_x)
                        world_y = int(kMagnificationWorldImagePos * object[i].world.y + kImageHeight/2 + self.displace_y)
                        
                        #静止物体の位置の画素値をゼロにする
                        static_object_world_pose_image[world_x][world_y] = 0

                        opening_static_object_world_pose_image=cv2.morphologyEx(static_object_world_pose_image,cv2.MORPH_OPEN,np.ones((3,3),np.uint8),anchor = (-1,-1),iterations = 1)

            #二個目の足を探索
            for i in range(0,object_num):
                #一個目の脚は飛ばす
                if i==led1_num:
                    continue

                #オブジェクトが動体かどうか
                if object[i].judge_dynamic == True:
                    image2_distance = (object[i].dynamic_image_pos.x-kImageWidth/2)**2 + (object[i].dynamic_image_pos.y - kImageHeight/2)**2
                    obj_radius = math.sqrt((object[i].dynamic_image_pos.x - human_obj.image_expect_human_x)**2+(object[i].dynamic_image_pos.y - human_obj.image_expect_human_y)**2)

                    #leg1までの距離
                    diff_distance = math.sqrt((object[i].dynamic_image_pos.x - leg1_point.x)**2 + (object[i].dynamic_image_pos.y - leg1_point.y)**2)
                else:
                    image2_distance = (object[i].image_pos.x-kImageWidth/2)**2+(object[i].image_pos.y - kImageHeight/2)**2
                    obj_radius = math.sqrt((object[i].image_pos.x - human_obj.image_expect_human_x)**2+(object[i].image_pos.y - human_obj.image_expect_human_y)**2)

                    #leg1までの距離
                    diff_distance = math.sqrt((object[i].image_pos.x - leg1_point.x)**2+(object[i].image_pos.y - leg1_point.y)**2)
                
                #脚候補が予測した範囲の中にあるときのみ脚と判断する
                if obj_radius < g_find_leg_radius:
                    leg2_distance = image2_distance
                    #動体かどうか
                    if object[i].judge_dynamic == True:
                        #leg1から一番近い脚候補のオブジェクトをleg2とする
                        if diff_distance < last_diff_distance:
                            leg2_point.x = object[i].dynamic_image_pos.x
                            leg2_point.y = object[i].dynamic_image_pos.y
                            last_diff_distance = diff_distance
                    else:
                        if diff_distance < last_diff_distance:
                            leg2_point.x = object[i].image_pos.x
                            leg2_point.y = object[i].image_pos.y
                            last_diff_distance = diff_distance

            #脚の可能性が高いものの中心の座標
            tmp_ave_x = (leg1_point.x + leg2_point.x)/2.0
            tmp_ave_y = (leg1_point.y + leg2_point.y)/2.0

            #脚の中心座標を予測
            #1時刻前の脚と座標の差分を求める
            image_diff_human_x = math.fabs(tmp_ave_x - human_obj.last_human_x)
            image_diff_human_y = math.fabs(tmp_ave_y - human_obj.last_human_y)
            
            #求めた差分から人の位置を予測し、次の検出範囲の中心とする
            human_obj.image_expect_human_x = (tmp_ave_x + image_diff_human_x)
            human_obj.image_expect_human_y = (tmp_ave_y + image_diff_human_y)

            #矩形
            tmp_radius = math.sqrt((tmp_ave_x - human_obj.image_expect_human_x)**2 + (tmp_ave_y - human_obj.image_expect_human_y)**2)

            #脚候補の中心が検出範囲尾の中にあるか
            human_pos_judge = ((math.fabs(tmp_ave_x - human_obj.image_expect_human_x) < g_find_leg_radius) and (math.fabs(tmp_ave_y - human_obj.image_expect_human_y) < g_find_leg_radius))

            #片足間の重心の距離886                             self.comeback_count = 0

            d = math.sqrt((leg1_point.x - leg2_point.x)**2 + (leg1_point.y - leg2_point.y)**2) /float(kMToPixel)

            #脚の位置からの矩形を描写
            cv2.rectangle(lidar_image,(int(human_obj.image_expect_human_x - g_find_leg_radius),int(human_obj.image_expect_human_y -  g_find_leg_radius)),(int(human_obj.image_expect_human_x + g_find_leg_radius),int(human_obj.image_expect_human_y + g_find_leg_radius)),(0,255,0),2)         
            
            #脚と判断したときの処理
            if (d < kLegBetweenDistance) and (human_pos_judge):
                #画像中心から脚までの距離[m]
                human_obj.distance = (math.sqrt(leg1_distance) + math.sqrt(leg2_distance))/(2*kMToPixel)
                human_obj.angle = (math.atan2(leg1_point.x - kImageWidth/2,kImageHeight/2 - leg1_point.y) + (math.atan2(leg2_point.x - kImageWidth/2,kImageHeight/2 - leg2_point.y)))/2.0

                self.human_approach_distance_sin = human_obj.distance*math.sin(0.785398 + human_obj.angle)
                self.human_approach_distance_cos = human_obj.distance*math.cos(0.785398 + human_obj.angle)
                
                #現在の脚の中心座標
                ave_x =(leg1_point.x + leg2_point.x)/2.0
                ave_y =(leg1_point.y + leg2_point.y)/2.0

                #脚を見失っていないときは一時刻前の座標に現在の座標を代入
                if (not ave_x == 999.0) and (not ave_y == 999.0):
                    human_obj.last_human_x = ave_x
                    human_obj.last_human_y = ave_y

                #人の位置のx座標、見失った場合は1時刻前の座標を使用
                if human_obj.image_expect_human_x == 0.0:
                    human_obj.image_pos.x = human_obj.last_human_x
                else:
                    human_obj.image_pos.x = human_obj.image_expect_human_x
                #人の位置のy座標、見失った場合は1時刻前の座標を使用
                if human_obj.image_expect_human_y == 0.0:
                    human_obj.image_pos.y = human_obj.last_human_y
                else:
                    human_obj.image_pos.y = human_obj.image_expect_human_y

            else:
                human_obj.distance = 999.0
                human_obj.angle = 999.0
                human_obj.local.x = 999.0
                human_obj.local.y = 999.0
                human_obj.setX(999.0)
                human_obj.setY(999.0)
                human_obj.setTheta(999.0)
                human_obj.image_pos.x = 999.0
                human_obj.image_pos.y = 999.0 

    '''
    脚の位置から人を追跡する関数
    input_image : LiDAR画像
    人を見失いかけたときは検出範囲を拡大する
    '''

    def trackingHuman(self,input_image):
        #グローバル使用宣言
        global lidar_image,g_find_leg_radius
        global human

        #脚候補
        obj = [Object() for i in range(0,100)]
        #オブジェクト数
        obj_num = 0

        #脚候補を見つけるために元画像で連続領域を探索
        #脚候補を見つけてその数を返す
        #長ズボンの際は0次モーメント(輪郭面積)を40,半ズボンなら34
        obj_num = self.findLegs(input_image,obj, lidar_image, red,10,28, 10, 18, 0.2, 1.5, 40, 110, 3400, 25000, 1.5, 0.5)

        #人間の位置と方向を計算(ローカル座標系)
        self.calcHumanPoseVer2(obj_num,obj,human)

        #画像表示のために1ミリ秒待つ
        cv2.waitKey(1)

        #見つけた人の位置に丸を描写
        cv2.circle(lidar_image,(int(human.image_pos.x),int(human.image_pos.y)),5,green,2)

        #この回数だけ連続して失敗すると検出範囲を拡大
        lost_count_max = 1

        #人を見失ったとき999
        if human.distance == 999.0:
            self.lost_count += 1
        else: 
            #見失っていない
            self.human_lost = False
            self.lost_count = 0

            #検出範囲をもとの大きさにする
            g_find_leg_radius = kOrignalRadius

        #lost_count_max回連続で発見できなかった際、検出範囲を拡大する
        if self.lost_count >= lost_count_max:
            self.human_lost = True
            g_find_leg_radius = kExtendRadius

        #人を見失ったとき、一時刻前の値を使う
        if self.human_lost == True:
            human.distance = human.last_distance
            human.angle = human.last_angle

        #追跡中の並進の制御
        if human.distance == 999.0:#人を完全に見失ったとき
            #静止物体の位置を初期化
            self.defaultPos(human)

        if (human.distance >= kTrackingMinDistance) and (human.distance <= kTrackingMaxDistance): #人が追跡範囲内にいる場合
            #人を完全に見失ったら初期化する
            if self.lost_count >= kLostTime:
                self.defaultPos(human)
                human.distance = 999.0
                human.angle = 999.0
                rospy.loginfo("I lost the human")

        else: #人が追跡対象外に出たら
            #静止物体の初期化
            self.defaultPos(human)

        
        human.last_distance = human.distance
        human.last_angle = human.angle

    '''
    LiDAR画像の処理をする関数
    lIDAR画像に縮小処理をし、ノイズを除去する
    '''

    def prepWindow(self):
        #グローバル関数
        global lidar_gray_image
        global lidar_image,lidar_erode_image
        #グレースケールに変換する
        ret,lidar_bin_image = cv2.threshold(lidar_gray_image,0,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU)
        #反転
        lidar_bin_image = ~(lidar_bin_image)
        #縮小、ノイズ除去
        lidar_erode_image = cv2.erode(lidar_bin_image,np.ones((3,3),np.uint8),anchor = (-1,-1),iterations = 1)
        #カラー画像に変換
        lidar_color_image = cv2.cvtColor(lidar_gray_image,cv2.COLOR_GRAY2BGR)
        lidar_image = lidar_color_image

    '''
    画像を表示する関数
    '''

    def showWindow(self):
        #グローバル宣言
        global lidar_image

        cv2.namedWindow("Map2",cv2.WINDOW_AUTOSIZE)
        dst_img = ~(lidar_image)

        #画像表示
        cv2.imshow("Map2",dst_img)

    '''
    sawyer_emergency_controllに人の位置情報を送る関数
    '''
        
    def humanDetector(self):
        Human_detector = rospy.Publisher('human_detector_command', Emergency, queue_size=1)
        emergency = Emergency()
        if self.step < 0 or self.step >= 1 and self.step < 80:
            if self.detector_count > 20:
                if 0 < self.approach_count < 5:
                    if self.human_lost == False and self.human_approach_distance_sin <= 1.1 and self.human_approach_distance_cos <= 1.85:
                        emergency.approach = True
                        emergency.sawyerstop = True
                        print("Human approaching. I need stop")
                        self.approach_count += 1
                        Human_detector.publish(emergency)
                    else:
                        emergency.approach = False
                        emergency.sawyerstop = False
                        emergency.restart = True
                        print("Human no approach. I restart work")
                        self.approach_count = 0
                        Human_detector.publish(emergency)
                elif self.approach_count == 5:
                    if self.human_lost == False and self.human_approach_distance_sin <= 1.1 and self.human_approach_distance_cos <= 1.85:
                        emergency.approach = False
                        emergency.evacuation = True
                        emergency.sawyerstop = True
                        print("Human approaching. I need evacuation")
                        self.approach_count += 1
                        Human_detector.publish(emergency)
                        self.restart == True
                    else:
                        emergency.approach = False
                        emergency.sawyerstop = False
                        emergency.restart = True
                        print("Human no approach. I restart work") 
                        self.approach_count = 0
                        Human_detector.publish(emergency)
                elif 5 < self.approach_count:
                    if self.sleep_time > 5:
                        if self.human_lost == False and self.human_approach_distance_sin <= 1.1 and self.human_approach_distance_cos <= 1.85:
                            emergency.evacuation = True
                            emergency.sawyerstop = True
                            print("Human approaching. I need evacuation")
                            self.approach_count += 1
                            Human_detector.publish(emergency)
                        else:
                            emergency.evacuation = False
                            emergency.sawyerstop = False
                            emergency.comeback = True
                            print("Human no approach. I comeback work")
                            self.approach_count = 0
                            Human_detector.publish(emergency)
                            self.comeback = True
                            self.sleep_time = 0
                    else:
                        self.sleep_time += 1
                else:
                    if self.human_lost == False and self.human_approach_distance_sin <= 1.1 and self.human_approach_distance_cos <= 1.85:
                        emergency.approach = True
                        print("Human approach")
                        self.approach_count += 1
                        Human_detector.publish(emergency)
                    elif self.restart == True:
                        if self.restart_count > 4:
                            self.restart = False
                            self.restart_count = 0
                            self.comeback = False
                            self.comeback_count = 0
                            emergency.approach = False
                            emergency.sawyerstop = False
                            emergency.restart = False
                            emergency.comeback = False
                            print("Human no approach")
                            self.approach_count = 0
                            Human_detector.publish(emergency)
                        else:
                            emergency.approach = False
                            emergency.sawyerstop = False
                            emergency.restart = True
                            emergency.comeback = False
                            print("Human no approach. I restart work") 
                            Human_detector.publish(emergency)
                            self.restart_count += 1
                    elif self.comeback == True:
                        if self.comeback_count > 4:
                            self.restart = False
                            self.restart_count = 0
                            self.comeback = False
                            self.comeback_count = 0
                            emergency.approach = False
                            emergency.sawyerstop = False
                            emergency.restart = False
                            emergency.comeback = False
                            print("Human no approach")
                            self.approach_count = 0
                            Human_detector.publish(emergency)
                        else:
                            emergency.approach = False
                            emergency.sawyerstop = False
                            emergency.restart = False
                            emergency.comeback = True
                            print("Human no approach. I comeback work")
                            Human_detector.publish(emergency)
                            self.comeback_count += 1
                    else:
                        emergency.approach = False
                        emergency.sawyerstop = False
                        emergency.restart = False
                        emergency.comeback = False
                        print("Human no approach")
                        self.approach_count = 0
                        Human_detector.publish(emergency)
                self.detector_count = 0
            else:
                self.detector_count += 1

    def robotMecanumMoveJudge(self,msg):
        self.step = msg.mecanum

if __name__ == "__main__":
    #ROSの初期化
    rospy.init_node("sawyer_human_detector_2")

    robot = Robot()

    #パラメータ表示
    robot.welcomeMessage()

    #33Hzのタイマー
    loop_rate = rospy.Rate(33)
    sum_time = 0.0
    cut = 0

    loop = 0

    time_counter = time.time()

    while not rospy.is_shutdown():
        robot.laser_cycle()
        if loop < 10:
            loop_rate.sleep()
            cut+=1
            loop+=1
            rospy.loginfo("time = %f",time.time() - time_counter)
            continue
        
        robot.trackingHuman(lidar_erode_image)
        robot.humanDetector()
        robot.showWindow()
        loop_rate.sleep()
        cut+=1
