#import example_websocket
import warper
import Angle2Distance
import requesttest
import time
import numpy as np
import math
from matplotlib import pyplot as plt

def getLoc(a,b,time):
    rl,rr,yl,yr,bl,br,_,_ = requesttest.fetch_data(time)
    print(rl,rr,yl,yr,bl,br)
    xm = []
    for i in range(min(len(bl),len(br))):
        btemp = round((bl[i] + br[i])/2)
        xm.append(btemp)
    print(xm)
    xl = []
    xr = []
    for j in range(len(xm)):
        #xl, assume yellow place on left
        if(yl[i] < xm[i] and yl[i] < rl[i]):
            xl.append(yl[i])
        elif(rl[i] < xm[i] and rl[i] < yl[i]):
            xl.append(rl[i])
        else:
            xl.append(0)

        #xr, assume yellow place on left
        if(yr[i] > xm[i] and yr[i] > rr[i]):
            xr.append(yr[i])
        elif(rr[i] > xm[i] and rr[i] > yr[i]):
            xr.append(rr[i])
        else:
            xr.append(0)
    rx = []
    ry = []
    for i in range(len(xl)):
        if(xl[i] ==0 or xm[i]==0 or xr[i]==0):
            continue  
        rxx,ryy = Angle2Distance.locationRoutine(a,b,xl[i],xm[i],xr[i])
        rx.append(rxx)
        ry.append(ryy)

    return rx,ry

#locating main routine
def locating(a,b):
    deg = 30
    #TODO: TURN 45 deg LEFT
    time.sleep(5)
    rx,ry = getLoc(a,b,5)
    x_avg = sum(rx)/len(rx)
    y_avg = sum(ry)/len(ry)
    return x_avg, y_avg

a = 18
b = 15
print(locating(a,b))