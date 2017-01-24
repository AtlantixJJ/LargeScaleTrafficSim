import matplotlib.pyplot as plt
import numpy as np

def parselog(path="res50-1.txt"):
    log = open(path)
    v_tot = []
    v_sd = []
    v_hm = []
    tot_dist = 0
    hm_dist = 0
    sd_dist = 0
    tot_t = 0
    hm_t = 0
    sd_t = 0
    for line in log.readlines():
        keys = line.split(' ')
        if keys[0] == '0':
            v_hm.append(float(keys[3]))
            hm_dist += float(keys[3])*float(keys[4])
            hm_t += float(keys[4])
        if keys[1] == '1':
            v_sd.append(float(keys[3]))
            sd_dist += float(keys[3])*float(keys[4])
            sd_t += float(keys[4])
    
    return np.array(v_tot),np.array(v_sd),np.array(v_hm)

vt,vs,vh = parselog()
plt.hist(vs)
plt.show()