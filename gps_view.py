#!/usr/bin/env python3
import sys
import rospy
from sensor_msgs.msg import NavSatFix
import matplotlib.pyplot as plt
import matplotlib
matplotlib.use('TkAgg')
import numpy as np
import matplotlib.animation
import time
from threading import Thread
import threading
# from matplotlib.colors import LinearSegmentedColormap
import matplotlib.cm as cm
# gps_latitude = 0
# gps_longitude = 0
# rtk_latitude = 0
# rtk_longitude = 0

# points = [0,0,10,10]
# fig, ax = plt.subplots()
# x, y = [],[]
# sc = ax.scatter(x,y)
# plt.xlim(points[0],points[2])
# plt.ylim(points[1],points[3])

class StoppableThread(Thread):
    """Thread class with a stop() method. The thread itself has to check
    regularly for the stopped() condition."""

    def __init__(self,  *args, **kwargs):
        super(StoppableThread, self).__init__(*args, **kwargs)
        self._stop_event = threading.Event()

    def stop(self):
        self._stop_event.set()
    def stopped(self):
        return self._stop_event.is_set()

def GpsPositionSubCallback(data):
    global gps_latitude
    global gps_longitude
    #print(" Gps 위도 : ", data.latitude, " Gps 경도 : ", data.longitude  )
    gps_latitude =  data.latitude
    gps_longitude = data.longitude
    # return gps_longitude, gps_latitude

def RtkPositionSubCallback(data):
    global rtk_latitude
    global rtk_longitude
    #print(" Rtk 위도 : ", data.latitude, " Rtk 경도 : ", data.longitude  )
    rtk_latitude = data.latitude
    rtk_longitude = data.longitude
    # return rtk_latitude, rtk_longitude

def get_drone_yaw_deg_Callback(data):
    global yaw
    yaw = data.drone_yaw_deg_

# points: Upper-left, and lower-right GPS points of the map (lat1, lon1, lat2, lon2).
def listener():
    global sc
    global gps_latitude
    global gps_longitude
    global rtk_latitude
    global rtk_longitude
    global points
    global x1, y1
    global d
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber('dji_osdk_ros/gps_position', NavSatFix, GpsPositionSubCallback)
    rospy.Subscriber('dji_osdk_ros/rtk_position', NavSatFix, RtkPositionSubCallback)
    # rospy.Subscriber('dji_osdk_ros/drone_yaw_deg_', yaw , get_drone_yaw_deg_Callback)
    list_latitude = []
    list_longitude = []
    img_points_x = []
    img_points_y = []
    img_points = []
    rospy.sleep(0.5)
    max_lat =  0
    max_lon = 0
    min_lat = 100
    min_lon = 200 
    count = 0
    t = StoppableThread(target=function)
    t.start()
    while True:
        if max_lat < gps_latitude:
            max_lat = gps_latitude + 0.00005
        if min_lat > gps_latitude:
            min_lat = gps_latitude - 0.00005
        
        if max_lon < gps_longitude:
            max_lon = gps_longitude + 0.00005
        if min_lon > gps_longitude:
            min_lon = gps_longitude - 0.00005
        
        # if before_offset_latitude == offset_latitude and before_offset_longitude == offset_longitude:
        #     count += 1
        #     if count > 6:
        #         sys.exit()
        # else:
        #     count = 0
        
        # before_gps_latitude = gps_latitude
        # before_gps_longitude = gps_longitude

        # before_rtk_latitude = rtk_latitude
        # before_rtk_longitude = rtk_longitude
        # print("out",offset_latitude, offset_longitude)
        gps_d = [gps_latitude, gps_longitude]
        rtk_d = [rtk_latitude, rtk_longitude]
        points = [min_lat, min_lon, max_lat, max_lon]
        d = [gps_d, rtk_d]
        # print("최소 위도 : ",min_lat, "최소 경도 : ",min_lon, "최대 위도 : ",max_lat, "최대 경도 : ",max_lon)
        # print(d)
        # result_image = Image.open("map.png", 'r')
        
        # print("main : ", d)
        # result_image = Image.open('map.png', 'r')
        # x1, y1 = scale_to_img(d, (1080, 1080), points)
        # img_points_x.append(x1)
        # img_points_y.append(y1)
        # img_points.append(x1,y1)
        # plt.xlim([min_lat, max_lat])      # X축의 범위: [xmin, xmax]
        # plt.ylim([min_lon, max_lon])     # Y축의 범위: [ymin, ymax]

        time.sleep(0.01)    
    rospy.spin()


def function():
    time.sleep(0.01)
    global x1, y1 , x2, y2
    global d
    global points
    global intensity1, intensity2, intensity3 ,intensity4

    gps_d = [0, 0]
    rtk_d = [0, 0]
    fig, ax = plt.subplots()
    x1,y1 = [], []
    x2,y2 = [], []
    intensity1 = []
    intensity2 = []
    intensity3 = []
    intensity4 = []
    iterations = 100
    t_vals = np.linspace(0,1, iterations)
    cmaps=[cm.get_cmap('Reds'),cm.get_cmap('Blues'),cm.get_cmap('Oranges'),cm.get_cmap('Greens')] #declaring colormaps
    sc1 = ax.scatter(x1,y1,c=[],cmap=cmaps[0],vmin=0,vmax=1) #initializing the 3 scatter plots
    sc2 = ax.scatter(x2,y2,c=[],cmap=cmaps[1],vmin=0,vmax=1) #initializing the 3 scatter plots
    sc3 = ax.scatter(d[0][1],d[0][0],c=[],cmap=cmaps[2],vmin=0,vmax=1) #initializing the 3 scatter plots
    sc4 = ax.scatter(d[1][1],d[1][0],c=[],cmap=cmaps[3],vmin=0,vmax=1) #initializing the 3 scatter plots
    plt.title('grid length : 1m')
    plt.xlabel('longitude')
    plt.ylabel('latitude')
    plt.xlim(points[1],points[3]) # 경도
    plt.ylim(points[0],points[2]) # 위도
    plt.xticks([])
    plt.yticks([])
    plt.xticks(np.arange(points[1], points[3], 0.00001),color='w'    )    
    plt.yticks(np.arange(points[0], points[2], 0.00001),color='w'    ) # ,color='w'    
    plt.grid(True)                       
    def animate(i):
        global intensity1, intensity2,intensity3 ,intensity4
        # x.append(x1)
        # y.append(y1)
        # print("thread : ", d)
        x1.append(d[0][1])
        y1.append(d[0][0])
        x2.append(d[1][1])
        y2.append(d[1][0])
        ax.set_xlim(points[1],points[3])#축
        ax.set_ylim(points[0],points[2])
        ax.set_xticks(np.arange(points[1], points[3], 0.00001)) #1m 좌표
        ax.set_yticks(np.arange(points[0], points[2], 0.00001))    
        sc1.set_offsets(np.c_[x1,y1])
        sc2.set_offsets(np.c_[x2,y2])
        sc3.set_offsets(np.c_[d[0][1],d[0][0]])
        sc4.set_offsets(np.c_[d[1][1],d[1][0]])
    #calculate new color values
        intensity1 = np.concatenate((np.array(intensity1)*0.995, np.ones(1)))
        intensity2 = np.concatenate((np.array(intensity2)*0.995, np.ones(1)))
        # intensity3 = np.concatenate((np.array(intensity3)*0.5, np.ones(1)))
        # intensity4 = np.concatenate((np.array(intensity4)*0.5, np.ones(1)))
        sc1.set_array(intensity1)
        sc2.set_array(intensity2)
        sc3.set_array([d[0][1],d[0][0]])
        sc4.set_array([d[1][1],d[1][0]])



    
    ani = matplotlib.animation.FuncAnimation(fig, animate, 
                    frames=t_vals, interval=50) 
                    
    plt.show()

if __name__ == '__main__':
    listener()
    