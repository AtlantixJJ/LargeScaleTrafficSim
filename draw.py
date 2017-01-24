"""
This will take log.txt and produce several figures naming from 1 to lane_num
You should place them in the same directory
"""


import matplotlib.pyplot as plt
import numpy as np

def parselog():
    log = open("log.txt")
    T = 0
    carID = set()
    # logging is composed of 6 columns: ID lane position velocity  and acc front car dist used to compute acc
    keys = log.readline().split(' ')
    segs = log.readline().split(' ')
    # the first line is :  total_time,tot_lane
    lane_num = int(keys[1])
    arr = np.zeros((1,6,int(keys[0])+3,10000))

    for i,line in enumerate(log.readlines()):
        keys = line.split(' ')
        if keys[0].find('R') > -1:
            T = T +1
            continue
        carID.add(int(keys[0]))
        if len(keys) < 2:
            break
        if int(keys[0])>9990:
            break
        # position
        for i in range(1):
            arr[i,int(keys[1]),T,int(keys[0])] = float(keys[i+2])
    
    return arr,segs,T,len(carID)

arr,segs,T,num_car = parselog()

def draw_lane(arr,name):
    # 2D arr
    for i in arr.transpose():
        plt.plot(i)
    plt.savefig(name+'.png')
    plt.clf()

lane_num = 4

namelist = ['pos','vec','acc']
for j in range(1):
    for i in range(lane_num):
        draw_lane(arr[j,i,:T-2,:num_car]+1,namelist[j]+str(i+1))


def extract_lane(arr,lanenum):
    return arr[:,lanenum,:T,:num_car]

def print_arr(arr):
    # 3D : arr(5,time,car)
    for i in range(arr.shape[1]):
        print("T:%d"%i)
        for j in range(arr.shape[2]):
            if arr[:,i,j].max() < 0.1:
                continue
            print("%.2f %.2f %.2f %.2f" % (arr[0,i,j],arr[1,i,j],arr[2,i,j],arr[3,i,j]) )

#cet = extract_lane(arr,1)
#print_arr(cet[:,400:500,:])