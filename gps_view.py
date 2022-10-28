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
import threading
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

def scale_to_img(lat_lon, h_w, points):
        """
        Conversion from latitude and longitude to the image pixels.
        It is used for drawing the GPS records on the map image.
        :param lat_lon: GPS record to draw (lat1, lon1).
        :param h_w: Size of the map image (w, h).
        :return: Tuple containing x and y coordinates to draw on map image.
        """
        old = (points[2], points[0])
        new = (0, h_w[1])
        y = ((lat_lon[0] - old[0]) * (new[1] - new[0]) / (old[1] - old[0])) + new[0]
        old = (points[1], points[3])
        new = (0, h_w[0])
        x = ((lat_lon[1] - old[0]) * (new[1] - new[0]) / (old[1] - old[0])) + new[0]
        # y must be reversed because the orientation of the image in the matplotlib.
        # image - (0, 0) in upper left corner; coordinate system - (0, 0) in lower left corner
        return int(x), h_w[1] - int(y)


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
    list_latitude = []
    list_longitude = []
    img_points_x = []
    img_points_y = []
    img_points = []
    rospy.sleep(2)
    max_lat =  0
    max_lon = 0
    min_lat = 100
    min_lon = 200 
    count = 0
    t = threading.Thread(target=function)
    t.start()
    while True:
        if rtk_latitude == 0 and rtk_longitude == 0:
            offset_latitude = gps_latitude
            offset_longitude = gps_longitude
        else:
            offset_latitude = rtk_latitude
            offset_longitude = rtk_longitude
        
        if max_lat < offset_latitude:
            max_lat = offset_latitude + 0.00005
        if min_lat > offset_latitude:
            min_lat = offset_latitude - 0.00005
        
        if max_lon < offset_longitude:
            max_lon = offset_longitude + 0.00005
        if min_lon > offset_longitude:
            min_lon = offset_longitude - 0.00005
        
        # if before_offset_latitude == offset_latitude and before_offset_longitude == offset_longitude:
        #     count += 1
        #     if count > 6:
        #         sys.exit()
        # else:
        #     count = 0
        
        before_offset_latitude = offset_latitude
        before_offset_longitude = offset_longitude
        # print("out",offset_latitude, offset_longitude)
        d = [offset_latitude, offset_longitude]
        points = [min_lat, min_lon, max_lat, max_lon]
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
    global x1, y1
    global d
    global points
    d = [0, 0]
    fig, ax = plt.subplots()
    x,y = [], []
    sc = ax.scatter(x,y)
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
        # x.append(x1)
        # y.append(y1)
        # print("thread : ", d)
        x.append(d[1])
        y.append(d[0])
        ax.set_xlim(points[1],points[3])
        ax.set_ylim(points[0],points[2])
        ax.set_xticks(np.arange(points[1], points[3], 0.00001)) #1m
        ax.set_yticks(np.arange(points[0], points[2], 0.00001))    
        sc.set_offsets(np.c_[x,y])

    ani = matplotlib.animation.FuncAnimation(fig, animate, 
                    frames=10, interval=100, repeat=True) 
                    
    plt.show()

if __name__ == '__main__':
    listener()
        