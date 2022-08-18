#!/usr/bin/env python3

import time
import copy
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Point32
from std_msgs.msg import String, UInt8
import numpy as np
import math
import cv2
from itertools import cycle
import matplotlib.pyplot as plt
from statistics import mean
import pandas as pd
import pcl
import serial
import collections
from functools import reduce
import operator

from sensor_msgs.msg import Imu, MagneticField, NavSatFix, NavSatStatus, PointCloud, LaserScan
from mecha_autoship_interfaces.srv import Battery, Actuator, Color
from adafruit_servokit import ServoKit
import board
import busio
import adafruit_bno055

def map(x, input_min, input_max, output_min, output_max):
    res = (x - input_min) * (output_max - output_min) / (
        input_max - input_min
    ) + output_min
    return int(output_min if res < output_min else res)
 
ser = serial.Serial(port = "/dev/ttyACM0", baudrate = 57600, timeout = 0.1)
i2c = board.I2C()
i2c_bus0 = (busio.I2C(board.SCL_1, board.SDA_1))
kit = ServoKit(channels=16,i2c=i2c_bus0)
sensor = adafruit_bno055.BNO055_I2C(i2c)     


kit.servo[0].set_pulse_width_range(1100, 1900) # servo 0 - bldc motor
kit.servo[1].set_pulse_width_range(1100, 1900) # servo 1 - bldc motor
time.sleep(1)
kit.servo[0].angle = 100 # 
kit.servo[1].angle = 100 #  
time.sleep(1)
kit.servo[0].angle = 135
kit.servo[1].angle = 65
time.sleep(1)

                
class MechaAutoshipExampleNode(Node):
    def __init__(self):
        super().__init__("mecha_autoship_example_node")
        self.get_logger().info("mecha_autoship_example_node Start")

        self.data = {
            "LiDAR": PointCloud(),
        }

        self.lidar_sub_handler = self.create_subscription(
            PointCloud, "scan_points", self.lidar_sub_callback, 10
        )

        # 특정 토픽으로부터 데이터를 가져오는 예시입니다. 연결되는 콜백 함수를 참고하세요.
        self.risk_calculator_lidar_data = self.create_timer(
            1.0, self.risk_calculator_lidar_data_callback
        )
    def GPSparser(self,data):
        gps_data = data.split(b",")
        idx_rmc = data.find(b'GNGGA')
        if data[idx_rmc:idx_rmc+5] == b"GNGGA":
            data = data[idx_rmc:]    
            if self.checksum(data):
                parsed_data = data.split(b",")
                print(parsed_data)
                return parsed_data
            else :
                print ("checksum error")
            
    def checksum(self,sentence):
        sentence = sentence.strip(b'\n')
        nmeadata, cksum = sentence.split(b'*',1)
        calc_cksum = reduce(operator.xor, (ord(chr(s)) for s in nmeadata), 0)
        print(int(cksum,16), calc_cksum)
        if int(cksum,16) == calc_cksum:
            return True 
        else:
            return False  
    
    def get_yaw(self):  #check heading
        yaw = ((math.atan2(sensor.magnetic[1], sensor.magnetic[0]) * 180/math.pi)+7) % 360
        return yaw          

    def Optimal(self,lat_1,lon_1,lat_2,lon_2): #check optimal angle (lat-long/yaw minus)
        optimal = 0
        φ1 = lat_1 * math.pi / 180
        φ2 = lat_2 * math.pi / 180
        λ1 = lon_1 * math.pi / 180
        λ2 = lon_2 * math.pi / 180
        y = math.sin(λ2 - λ1) * math.cos(φ2)
        x = math.cos(φ1) * math.sin(φ2) - math.sin(φ1) * math.cos(φ2) * math.cos(λ2 - λ1)
        θ = math.atan2(y, x)
        optimals = (θ * 180 / math.pi + 360) % 360
        self.get_logger().info("optimals:{}".format(optimals))
        yaw = self.get_yaw()
        self.get_logger().info("yaw: {}".format(yaw))
        optimal = optimals- yaw
        print("final_optiamls: %f" %(optimal))
        if abs(optimal)>180:
            if optimals > yaw:
                optimal = optimal - 360 # angle = -180 down => left  
            elif optimals < yaw:
                optimal = abs(abs(optimal) - 360) # angle = 180 up => right
        if optimal > 90:
            optimal = 90
        elif optimal < -90:
            optimal = -90   
        return optimal
        
    def locationForAngle(self): #receive gps to change variable number
        cnt = 0
        while True:
            data = ser.readline()
            result = collections.defaultdict()
            res = self.GPSparser(data)
            if res == None:
                continue
            else:
                print(res)
                lat = str (res[2])
                lon = str (res[4])
                if (res == "checksum error"):
                    print("")
                    print(lat)
                else:
                    if cnt%2 == 0:
                        return gpsangle
                    lat_h = float(lat[2:4])
                    lon_h = float(lon[2:5])
                    lat_m = float(lat[4:12])
                    lon_m = float(lon[5:13])
                    print('lat_h: %f lon_h: %f lat_m: %f lon_m: %f' %(lat_h, lon_h, lat_m, lon_m))
                    latitude = lat_h + (lat_m/60)
                    longitude = lon_h + (lon_m/60)
                    print('latitude: %f longitude: %f' %(latitude,longitude))
                    gpsangle = self.Optimal(latitude,longitude,goal_lat,goal_long)
                    cnt +=1
    def lidar_sub_callback(self, data):
        self.data["LiDAR"] = data
    
    def risk_calculator_lidar_data_callback(self):
        angle = self.locationForAngle()
        self.Servo(angle)
        # self.get_logger().info("######################################################")
       # # self.get_logger().info("{}".format(yaw))
        # yaw = self.get_yaw()
        # xs = []
        # ys = []
        # warning_r = False
        # warning_l = False
        # left_len = 100
        # right_len = 100
        # warning = False
        # for data_single in self.data["LiDAR"].points :
            # if (0.3 < data_single.y < 0.5 and -2< data_single.x < 2):
                # if  -1.5 < data_single.x < 0:
                    # warning_l = True
                    # left_len = min(left_len,math.sqrt(data_single.x**2+data_single.y**2))
                # if 0 < data_single.x < 1.5:
                    # warning_r = True
                    # right_len = min(right_len,math.sqrt(data_single.x**2+data_single.y**2))
                # if data_single.y < 0.8 and -0.25 < data_single.x < 0.25:
                    # warning = True 
            # if(data_single.y > 0 and -1 < data_single.x < 1 and math.sqrt(data_single.x**2+data_single.y**2) < 2):
                # xs.append(data_single.x)
                # ys.append(data_single.y)
        # if warning == True and 270 < yaw < 360 or 0 < yaw < 5:
            # kit.servo[0].angle = 45 
            # kit.servo[1].angle = 135
            # time.sleep(0.3)
        # elif warning == True and 5 < yaw <100:
            # kit.servo[0].angle = 65
            # kit.servo[1].angle = 155
            # time.sleep(0.5)
        # kit.servo[0].angle = 130
        # kit.servo[1].angle = 70           
        # if(len(xs) != 0):
            # point_groups = []
            # points_tmp = []
            # # self.get_logger().info(
                    # # "x_group: {0}\n".format(
                        # # x_group,
                        # # ))                
            # point_last_x = xs[0]
            # point_last_y = ys[0]
            # i_end = len(xs)
            # is_end_append = False    
        
            
            # for i in range(1,i_end):
                # point_now_x = xs[i] 
                # point_now_y = ys[i] 
                # length_a = point_last_x - point_now_x
                # length_b = point_last_y - point_now_y
                # length_c = math.sqrt((length_a * length_a) + (length_b * length_b))
            
                # if length_c < 0.1 : # 이전 점과 거리가 짧으면 임시 그룹에 넣음
                    # points_tmp.append(copy.deepcopy([point_now_x,point_now_y]))
                # else : # 이전 점에서 멀리 떨어져 있으면 다음 그룹의 점으로 간주하고 기존 그룹은 저장
                    # if len(points_tmp) > 10 : # 구성이 10개보다 큰 경우에만 그룹으로 인정
                        # if i == i_end - 1 :
                            # is_end_append = True
                        # point_groups.append(copy.deepcopy(points_tmp))
                    # points_tmp = []
                # point_last_x = copy.deepcopy(point_now_x)
                # point_last_y = copy.deepcopy(point_now_y)
            # if len(points_tmp) > 10 and is_end_append == False : # 마지막 그룹까지 계산
                # point_groups.append(copy.deepcopy(points_tmp)) 
            # mid_group = []
            # if len(point_groups) != 0:
                # for i in range(len(point_groups)):
                    # mid_group.append([point_groups[i][int(len(point_groups[i])/2)][0],point_groups[i][int(len(point_groups[i])/2)][1]])
               # # self.get_logger().info("{}".format(mid_group))
            
            # # print(len(point_groups))
            # # for group in point_groups :
                # # list_x = []
                # # list_y = []
                # # for point in group :
                    # # plt.scatter(point.x, point.y)
                    # # list_x.append(copy.deepcopy(point.x))
                    # # list_y.append(copy.deepcopy(point.y))
                # # plt.scatter(list_x, list_y)
            # # plt.show()
            # #self.get_logger().info(
             # #       "lens_pointgroup: {0}\n".format(
              # #          len(point_groups),
               # #         ))
            # # length = []
            # # if len(point_groups) != 0:
                # # if len(point_groups) == 1:
                    # # if point_groups[0][0][0] > 0:
                        # # length.append(point_groups[0][0][1]**2)
                        # # length.append(point_groups[0][-1][1]**2 + (point_groups[0][-1][0] + 1.5)**2)
                    # # elif point_groups[0][-1][0] < 0:
                        # # length.append(point_groups[0][-1][1]**2)
                        # # length.append(point_groups[0][0][1]**2 + (point_groups[0][0][0] - 1.5)**2)
                # # else:
                    # # for i in range(0,len(point_groups)):
                       # # # self.get_logger().info("i : {}".format(i))
                        # # if i == 0:
                            # # if point_groups[i][0][0] > 0:
                                # # length.append(point_groups[0][0][1]) 
                                # # length.append(math.pow((point_groups[i+1][0][1]-point_groups[i][-1][1])**2 + (point_groups[i+1][0][0]-point_groups[i][-1][0])**2))
                            # # elif point_groups[i][0][0] < 0:
                                # # length.append(math.pow(((point_groups[0][0][1])**2) + (point_groups[0][0][0] - 1.5)**2))
                                # # length.append(math.pow((point_groups[i+1][0][1]-point_groups[i][-1][1])**2 + (point_groups[i+1][0][0]-point_groups[i][-1][0])**2))
                        # # elif i == (len(point_groups) - 1):
                            # # if point_groups[i][-1][0] < 0:
                                # # length.append(point_groups[0][-1][1])
                            # # elif point_groups[i][-1][0] > 0:
                                # # length.append(math.pow(point_groups[0][-1][1]**2 + (point_groups[0][0][0] + 1.5)**2))
                        # # else:
                            # # length.append(math.pow((point_groups[i+1][0][1]-point_groups[i][-1][1])**2 + (point_groups[i+1][0][0]-point_groups[i][-1][0])**2))
            # # self.get_logger().info(
                    # # "length: {0}\n".format(
                        # # length,
                        # # ))                
            # length1 = []
            # Angles = []
            # # can pass over two way
            # if len(point_groups) < 3:
                # if len(point_groups) == 1:
                    # if point_groups[0][0][0] > 0: 
                        # x1 = point_groups[0][0][0]
                        # y1 = 0
                        # x2 = point_groups[0][0][0]
                        # y2 = point_groups[0][0][1]
                        # if point_groups[0][-1][0] > 0:
                            # x11 = point_groups[0][-1][0]
                            # y11 = point_groups[0][-1][1]
                            # x22 = -1.0
                            # y22 = 0
                        # else:
                            # x11 = point_groups[0][-1][0] 
                            # y11 = point_groups[0][-1][1]
                            # x22 = point_groups[0][-1][0]
                            # y22 = 0
                    # else:
                        # x1 = 1.0
                        # y1 = 0
                        # x2 = point_groups[0][0][0]
                        # y2 = point_groups[0][0][1]
                        # x11 = point_groups[0][-1][0]
                        # y11 = point_groups[0][-1][1]
                        # x22 = point_groups[0][-1][0]
                        # y22 = 0
                    # length1.append(math.sqrt((x1-x2)**2 + (y1-y2)**2))
                    # length1.append(math.sqrt((x11-x22)**2 + (y11-y22)**2))    
                    # Angles.append([math.atan2(y1,x1)*(180/math.pi),math.atan2(y2,x2)*(180/math.pi)])        
                    # Angles.append([math.atan2(y11,x11)*(180/math.pi),math.atan2(y22,x22)*(180/math.pi)])         
                # elif len(point_groups) == 2:
                    # x2 = point_groups[0][0][0]
                    # y2 = point_groups[0][0][1]
                    # x11 = point_groups[1][-1][0]
                    # y11 = point_groups[1][-1][1]
                    # x1m = point_groups[0][-1][0]
                    # y1m = point_groups[0][-1][1]
                    # x2m = point_groups[1][0][0]
                    # y2m = point_groups[1][0][1]
                    # if(point_groups[0][0][0] > 0 and point_groups[1][-1][0] > 0):
                        # x1 = point_groups[0][0][0]
                        # y1 = 0
                        # x22 = -1.0
                        # y22 = 0
                    # elif(point_groups[0][0][0] < 0 and point_groups[1][-1][0] < 0):
                        # x1 = 1.0
                        # y1 = 0
                        # x22 = point_groups[1][-1][0]
                        # y22 = 0
                    # else:             
                        # x1 = point_groups[0][0][0]
                        # y1 = 0
                        # x22 = point_groups[1][-1][0]
                        # y22 = 0
                    # length1.append(math.sqrt((x1-x2)**2 + (y1-y2)**2))
                    # length1.append(math.sqrt((x1m-x2m)**2 + (y1m-y2m)**2))
                    # length1.append(math.sqrt((x11-x22)**2 + (y11-y22)**2))    
                    # Angles.append([math.atan2(y1,x1)*(180/math.pi),math.atan2(y2,x2)*(180/math.pi)])
                    # Angles.append([math.atan2(y1m,x1m)*(180/math.pi),math.atan2(y2m,x2m)*(180/math.pi)])        
                    # Angles.append([math.atan2(y11,x11)*(180/math.pi),math.atan2(y22,x22)*(180/math.pi)])
            # else:
                # for i in range(len(point_groups)):
                    # if(i == 0):
                        # x11 = point_groups[0][-1][0]
                        # y11 = point_groups[0][-1][1]
                        # x22 = point_groups[1][0][0]
                        # y22 = point_groups[1][0][1]
                        # if point_groups[0][0][0] > 0: 
                            # x1 = point_groups[0][0][0]
                            # y1 = 0
                            # x2 = point_groups[0][0][0]
                            # y2 = point_groups[0][0][1]                        
                        # else:
                            # x1 = 1.0
                            # y1 = 0
                            # x2 = point_groups[0][0][0]
                            # y2 = point_groups[0][0][1]
                        # length1.append(math.sqrt((x1-x2)**2 + (y1-y2)**2))
                        # length1.append(math.sqrt((x11-x22)**2 + (y11-y22)**2))
                        # Angles.append([math.atan2(y1,x1)*(180/math.pi),math.atan2(y2,x2)*(180/math.pi)])    
                        # Angles.append([math.atan2(y11,x11)*(180/math.pi),math.atan2(y22,x22)*(180/math.pi)])
                    # elif(i == len(point_groups)-1):
                        # if point_groups[i][-1][0] < 0:
                            # x11 = point_groups[i][-1][0]
                            # y11 = point_groups[i][-1][1]
                            # x22 = point_groups[i][-1][0]
                            # y22 = 0
                        # else:
                            # x11 = point_groups[i][-1][0]
                            # y11 = point_groups[i][-1][1]
                            # x22 = -1.0
                            # y22 = 0
                        # length1.append(math.sqrt((x11-x22)**2 + (y11-y22)**2))    
                        # Angles.append([math.atan2(y11,x11)*(180/math.pi),math.atan2(y22,x22)*(180/math.pi)])        
                    # else:
                        # x1m = point_groups[i][-1][0]
                        # y1m = point_groups[i][-1][1]
                        # x2m = point_groups[i+1][0][0]
                        # y2m = point_groups[i+1][0][1]
                        # length1.append(math.sqrt((x1m-x2m)**2 + (y1m-y2m)**2)) 
                        # Angles.append([math.atan2(y1m,x1m)*(180/math.pi),math.atan2(y2m,x2m)*(180/math.pi)])
            # self.get_logger().info(
                    # "length1: {0}\n".format(
                        # length1,
                        # ))                                   
            # if(len(Angles) != 0):
                # self.get_logger().info("\nangle:{}\n".format(Angles))
                # can_angle = []
                # for i in range(len(length1)):
                    # if length1[i] > 0.6:
                        # can_angle.append(Angles[i])           # group index 
                # if(len(can_angle) != 0):
                    # self.get_logger().info("\nCangle:{}\n".format(can_angle))
                gps_angle = self.locationForAngle()
                charge_angle = 90
                if -90<=gps_angle<=0:
                    charge_angle = abs(gps_angle)+90
                elif 0<gps_angle<=90:
                    charge_angle = 90 - gps_angle
                self.get_logger().info("charge_angle:{}\n".format(charge_angle))
                ######## warning_l, warning_r,right_len,left_len
                # if len(can_angle) > 0:
                    # check = False
                    # for i in range(len(can_angle)):
                        # if can_angle[i][0] < charge_angle and charge_angle < can_angle[i][1] :
                            # check = True
                            # head_angle = (can_angle[i][0] + can_angle[i][1] ) / 2
                            # self.get_logger().info("1111111111111111111111111111111\n")
                            # self.Servo(head_angle)
                            # break
                    # Min = 500
                    # idx = 0
                    # if check == False:
                        # self.get_logger().info("222222222222222222222222222222\n")
                        # for i in range(len(can_angle)-1):
                            # if warning_l == True and warning_r == False:
                                # self.get_logger().info("warning_l is True\n")
                                # if ((can_angle[i][0] + can_angle[i][1]) / 2) > 90:
                                    # del can_angle[i]
                            # elif warning_r == True and warning_l == False:
                                # self.get_logger().info("warning_r is True\n")
                                # if ((can_angle[i][0] + can_angle[i][1]) / 2) < 90:
                                    # del can_angle[i]
                            # elif warning_l == True and warning_r == True:
                                # self.get_logger().info("warning_r and warning_l are True\n")
                                # if right_len > left_len:
                                    # if ((can_angle[i][0] + can_angle[i][1]) / 2) > 90:
                                         # del can_angle[i]                
                                # else:
                                    # if ((can_angle[i][0] + can_angle[i][1]) / 2) < 90:
                                        # del can_angle[i]
                        # if len(can_angle) == 1:
                            # self.Servo((can_angle[0][0]+can_angle[0][1])/2)
                        # else:                            
                            # for i in range(len(can_angle)-1):
                                # if (abs(can_angle[i][1] - charge_angle) < abs(can_angle[i+1][0] - charge_angle)):
                                    # if (Min > abs(can_angle[i][1] - charge_angle)):
                                        # Min = abs(can_angle[i][1] - charge_angle)
                                        # idx = i 
                                # else:
                                    # if (Min > abs(can_angle[i+1][0] - charge_angle)):
                                        # Min = abs(can_angle[i+1][0] - charge_angle)
                                        # idx = i+1 
                            # head_angle = (can_angle[idx][0] + can_angle[idx][1]) / 2
                            # self.Servo(head_angle)   
                    
                      
    def Servo(self,angle): 
        self.get_logger().info('anglereal: %f' %(angle))
        if angle < 90:
            kit.servo[0].angle = 20
            kit.servo[1].angle = 20
            if 0<abs(angle)<=3:
                time.sleep(0.72)
                kit.servo[0].angle = 180
                kit.servo[1].angle = 180
                time.sleep(0.42)
            elif 3<abs(angle)<=6:
                time.sleep(0.66)
                kit.servo[0].angle = 180
                kit.servo[1].angle = 180
                time.sleep(0.36)
            elif 6<abs(angle)<=12:
                time.sleep(0.63)
                kit.servo[0].angle = 180
                kit.servo[1].angle = 180
                time.sleep(0.33)
            elif 12<abs(angle)<=18:
                time.sleep(0.57)
                kit.servo[0].angle = 180
                kit.servo[1].angle = 180
                time.sleep(0.27)
            elif 18<abs(angle)<=20:
                time.sleep(0.52)
                kit.servo[0].angle = 180
                kit.servo[1].angle = 180
                time.sleep(0.22)
            elif 20<abs(angle)<=26:
                time.sleep(0.48)
                kit.servo[0].angle = 180
                kit.servo[1].angle = 180
                time.sleep(0.18)
            elif 26<abs(angle)<=32:
                time.sleep(0.43)
                kit.servo[0].angle = 180
                kit.servo[1].angle = 180
                time.sleep(0.13)
            elif 32<abs(angle)<=38:
                time.sleep(0.38)
            elif 38<abs(angle)<=40:
                time.sleep(0.36)
            elif 40<abs(angle)<=46:
                time.sleep(0.34)
            elif 46<abs(angle)<=52:
                time.sleep(0.3)
            elif 52<abs(angle)<=56:
                time.sleep(0.26)
            elif 56<abs(angle)<=60:
                time.sleep(0.22)
            elif 60<abs(angle)<=66:
                time.sleep(0.18)
            elif 66<abs(angle)<=70:
                time.sleep(0.14)
            elif 70<abs(angle)<=76:
                time.sleep(0.1)
            elif 76<abs(angle)<=82:
                time.sleep(0.06)
            elif 82<abs(angle)<=84:
                time.sleep(0.02)
            elif 84<abs(angle)<=90:
                kit.servo[0].angle = 140
                kit.servo[1].angle = 60
        if angle > 90:
            kit.servo[0].angle = 180
            kit.servo[1].angle = 180
            if 90<abs(angle)<=93:
                kit.servo[0].angle = 140
                kit.servo[1].angle = 60
            elif 93<abs(angle)<=96:
                time.sleep(0.02)
            elif 96<abs(angle)<=102:
                time.sleep(0.06)
            elif 102<abs(angle)<=108:
                time.sleep(0.1)
            elif 108<abs(angle)<=110:
                time.sleep(0.14)
            elif 110<abs(angle)<=116:
                time.sleep(0.18)
            elif 116<abs(angle)<=122:
                time.sleep(0.22)
            elif 122<abs(angle)<=128:
                time.sleep(0.26)
            elif 128<abs(angle)<=130:
                time.sleep(0.3)
            elif 130<abs(angle)<=136:
                time.sleep(0.34)
            elif 136<abs(angle)<=142:
                time.sleep(0.36)
            elif 142<abs(angle)<=146:
                time.sleep(0.38)
            elif 146<abs(angle)<=150:
                time.sleep(0.43)
                kit.servo[0].angle = 20
                kit.servo[1].angle = 20
                time.sleep(0.13)
            elif 150<abs(angle)<=156:
                time.sleep(0.48)
                kit.servo[0].angle = 20
                kit.servo[1].angle = 20
                time.sleep(0.18)
            elif 156<abs(angle)<=160:
                time.sleep(0.52)
                kit.servo[0].angle = 20
                kit.servo[1].angle = 20
                time.sleep(0.22)
            elif 160<abs(angle)<=166:
                time.sleep(0.57)
                kit.servo[0].angle = 20
                kit.servo[1].angle = 20
                time.sleep(0.27)
            elif 166<abs(angle)<=172:
                time.sleep(0.63)
                kit.servo[0].angle = 20
                kit.servo[1].angle = 20
                time.sleep(0.33)
            elif 172<abs(angle)<=174:
                time.sleep(0.66)
                kit.servo[0].angle = 20
                kit.servo[1].angle = 20
                time.sleep(0.36)
            elif 174<abs(angle)<=180:
                time.sleep(0.72)
                kit.servo[0].angle = 20
                kit.servo[1].angle = 20
                time.sleep(0.42)
        kit.servo[0].angle = 140
        kit.servo[1].angle = 60

def main(args=None):
    rclpy.init(args=args)
    mecha_autoship_example_node = MechaAutoshipExampleNode()
    rclpy.spin(mecha_autoship_example_node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
