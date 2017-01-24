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
    arr = np.zeros((1,5,int(keys[0])+3,10000))

    for i,line in enumerate(log.readlines()):
        keys = line.split(' ')
        if keys[0].find('R') > -1:
            T = T +1
            continue
        carID.add(int(keys[0]))
        if len(keys) < 5:
            break
        if int(keys[0])>9990:
            break
        # position
        for i in range(1):
            arr[i,int(keys[1]),T,int(keys[0])] = float(keys[i+2])
    
    return arr,segs,T,len(carID)

arr,segs,T,num_car = parselog()
#segs = [float(i) for i in segs]

def take(data,t):
    res = []
    cs = float(data.shape[0])
    for j in range(data.shape[0]):
        # lane coordinate
        x_arr = data[j,t,:][data[j,t,:]>1]/10000.
        y_arr = (j+1) * np.ones_like(x_arr)/cs
        res.append(np.array([x_arr,y_arr]))
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
    cet = take(arr[0,:],i+delta)
    ax.clear()
    for lane in cet:
        scat = ax.scatter(lane[0,:],lane[1,:],animated=True)
    ax.axis([0,1,0,1])
    return scat,
    

ani = animation.FuncAnimation(fig, animate, frames=T-1-delta,interval=5,
blit=True)#, init_func=init)
ani.save('siv.mp4', fps=10)