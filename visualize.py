import numpy as np
import matplotlib.pyplot as plt
import scipy.integrate as integrate
import matplotlib.animation as animation

fig = plt.figure(figsize=(7,7))
ax = fig.add_axes([0, 0, 1, 1], frameon=False)
ax.set_xlim(0,1), ax.set_xticks([])
ax.set_ylim(0,1), ax.set_yticks([])

def parselog():
    log = open("log.txt")
    T = 0
    carID = set()
    # logging is composed of 6 columns: ID lane position velocity  and acc front car dist used to compute acc
    keys = log.readline().split(' ')
    segs = log.readline().split(' ')
    # the first line is :  total_time,tot_lane
    lane_num = int(keys[1])
    arr = np.zeros((2,6,int(keys[0])+10,10000))

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
        for i in range(2):
            arr[i,int(keys[1]),T,int(keys[0])] = float(keys[i+2])
    
    return arr,segs,T,len(carID)

arr,segs,T,num_car = parselog()
#segs = [float(i) for i in segs]

def take(data,t):
    res = []
    cs = float(data.shape[2]-1)
    print(data.shape)
    for j in range(data.shape[1]):
        # lane coordinate
        """
        cx_list = []
        x_list = []
        for l in range(data.shape[3]):
            if data[0,j,t,l] > 0.9 and data[1,j,t,l]>1: #convoy
                cx_list.append(data[1,j,t,l])
            if data[0,j,t,l] <0.1 and data[1,j,t,l]>1:
                x_list.append(data[1,j,t,l])
        x_arr = np.array(x_list)/10000.
        cx_arr = np.array(cx_list)/10000.
        """
        
        y_set = (j+1) /cs
        res.append([x_arr,cx_arr,y_set])
    return res

def display_data(data):
    count = 0
    l = []
    for t in range(data.shape[0]):
        now = take(data,t)
        if now.shape[1] < 2:
            continue
        l.append(now)
    return l

def init():
    scat = ax.scatter(disp[0][0,:],disp[0][1,:], animated=True)
    
    return scat,

delta = 50
def animate(i):
    print("Processing : %d" % i)
    cet = take(arr,i+delta)
    ax.clear()
    color1 = np.ones(len(cet[0][0]))*0.8
    color2 = np.ones(len(cet[0][1]))/4.
    #color1 = ['b' for i in range(len(cet[0][0]))]
    #color2 = ['r' for i in range(len(cet[0][1]))]
    for lane in cet:
        #y_arr = np.ones_like(lane[0])/lane[2]
        #scat = ax.scatter(lane[0],y_arr,c=color1,animated=True)
        y_arr = np.ones_like(lane[1])/lane[2]
        scat = ax.scatter(lane[1],y_arr,animated=True)
    ax.axis([0,1,0,1])
    return scat,
    

ani = animation.FuncAnimation(fig, animate, frames=100,interval=5,
blit=True)#, init_func=init)
ani.save('siv.mp4', fps=10)