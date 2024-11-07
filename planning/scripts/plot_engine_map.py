#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
import numpy as np
from scipy.interpolate import griddata
import matplotlib

matplotlib.use('TkAgg')

from matplotlib import pyplot as plt
import matplotlib.pyplot as plt
import pandas as pd
import os
import sys
import threading
import time
import csv

path = os.path.abspath('..') + '/devel/lib/python3/dist-packages'
sys.path.insert(0, path)
from planning.msg import plan_debug

current_directory = os.path.dirname(os.path.abspath(__file__))

# 读取数据
data = pd.read_csv(current_directory + '/engine_map.csv')
x = data['rpm']
y = data['torque']
z = data['fuel_consumption']

X, Y = np.meshgrid(np.linspace(min(x), max(x), 200), np.linspace(min(y), max(y), 200))
Z = griddata((x, y), z, (X, Y), method = 'cubic')

req_engine_speed = 0
req_engine_torque = 0

req_engine_speed_vector = []
req_engine_torque_vector = []
frist_open = True

def dateupgrade(msg):
    print("Received \n")
    global req_engine_speed , req_engine_torque
    global req_engine_speed_vector , req_engine_torque_vector
    req_engine_speed = msg.pacc.output.tar_engine_ne
    req_engine_torque = msg.pacc.output.tar_engine_torque

    req_engine_speed_vector.append(msg.pacc.output.tar_engine_ne)
    req_engine_torque_vector.append(msg.pacc.output.tar_engine_torque)

    rospy.loginfo("req_engine_speed:%d, req_engine_torque:%d", req_engine_speed , req_engine_torque)


def plot(msg):
    global req_engine_speed , req_engine_torque
    global req_engine_speed_vector , req_engine_torque_vector
    global X, Y, Z

    plt.clf()
    cs = plt.contour(X,Y,Z,[184,185,187,189,191,194,199,204,214,224,244,294,344,394],linewidths=1)
    plt.clabel(cs, fontsize = 12, inline = True)
    plt.legend(loc = 0)
    plt.xlim(700, 2200)
    plt.ylim(0, 2600)
    plt.title('engine_map',fontdict = {'family' : 'Times New Roman', 'size'   : 10})
    plt.xlabel('engine_speed($ rmp $)', fontdict = {'family' : 'Times New Roman', 'size'   : 10})
    plt.ylabel('engine_toque($ N.m $)', fontdict = {'family' : 'Times New Roman', 'size'   : 10})
    plt.plot(req_engine_speed_vector, req_engine_speed_vector, 'om') 
    plt.pause(0.0000001)
    plt.ioff()

def writecsv(msg):
    global frist_open
    path = current_directory + '/engine_map_real.csv'

    if (frist_open) :
        with open(path,'w') as f:
            csv_write = csv.writer(f)
            csv_head = ["rpm_a","torque_a"]
            csv_write.writerow(csv_head)
            frist_open = False
    else :
        with open(path,'a+') as f:
            csv_write = csv.writer(f)
            data_row = [msg.age, msg.height]
            csv_write.writerow(data_row)


def Data_Callback(msg):
    date_update_thread = threading.Thread(target = dateupgrade(msg))
    date_update_thread.setDaemon(True)

    plot_thread = threading.Thread(target = plot(msg))
    plot_thread.setDaemon(True)
    
    date_csv_thread = threading.Thread(target = writecsv(msg))
    date_csv_thread.setDaemon(True)

    date_csv_thread.start()
    date_update_thread.start()
    plot_thread.start()
    

def listener():

    rospy.init_node('listener', anonymous= True)
    rospy.Subscriber("planning/debug", plan_debug, Data_Callback)
    rospy.spin()


def plotend():
    global X, Y, Z
    
    real_data = pd.read_csv(current_directory + '/engine_real.csv')

    x_real= real_data['rpm_a']
    y_real = real_data['torque_a']

    bins = len(x_real)

    x_bins = np.linspace(700, 2200, bins + 1)
    y_bins = np.linspace(0, 2600, bins + 1)

    x_idx = np.digitize(x_real, x_bins)
    y_idx = np.digitize(y_real, y_bins)

    # 统计每个区间中点的出现次数
    counts = np.zeros((bins + 1, bins + 1))
    for i in range(len(x_real)):
        counts[x_idx[i]-1, y_idx[i]-1] += 1

    max_count = len(x_real)
    colors = counts / max_count
    c = colors[x_idx-1, y_idx-1]

    sc = plt.scatter(x_real, y_real, c = c, marker = '+', cmap='rainbow', s = 100)
    plt.colorbar(sc)

    # 极坐标设置
    cs = plt.contour(X, Y, Z, [184,185,187,189,191,194,199,204,214,224,244,294,344,394], colors = "R", linewidths = 1)

    plt.clabel(cs, fontsize = 12, inline = True)
    plt.legend(loc = 0)
    plt.xlim(600, 2200)
    plt.show()

if __name__ == '__main__':
    Online = False

    if (Online) :
        listener()
    else :
        plotend()




