import requests
from datetime import datetime, timedelta
import operator
import Angle2Distance
import math

def fetch_data():
    try:
        response = requests.get('http://18.168.204.27:3001/pointQuery')
        response_data = response.json()

 

        # Sort the data by timestamp in ascending order
        sorted_data = sorted(response_data, key=lambda k: datetime.strptime(k['timestamp'], "%Y-%m-%dT%H:%M:%S.%fZ"), reverse=True)

 

        start_point = datetime.now()- timedelta(hours=1, minutes=0)

 

        batch_data = []  # Reset batchData for each batch

 

        for point in sorted_data:
            point_time = datetime.strptime(point['timestamp'], "%Y-%m-%dT%H:%M:%S.%fZ")

 

            #If this point is within 10 seconds of the start point, add it to the batch

            if 0 <= (start_point - point_time).total_seconds() <= 10:
                batch_data.append(point)
            elif (start_point - point_time).total_seconds() > 10:
                # If this point is more than 10 seconds from the start point, stop processing
                break

 

            #batch_data.append(point) # use this instead for getting all the points

 

        filtered_data = [point for point in batch_data if point['BeaconType'] == 'RedTL' and point['xcordinate'] <= 640 and point['ycordinate'] <= 480]
        red_points_left = [point['xcordinate'] for point in filtered_data]


        filtered_data = [point for point in batch_data if point['BeaconType'] == 'RedBR' and point['xcordinate'] <= 640 and point['ycordinate'] <= 480]
        red_points_right = [point['xcordinate'] for point in filtered_data]


        filtered_data = [point for point in batch_data if point['BeaconType'] == 'YELLOWTL' and point['xcordinate'] <= 640 and point['ycordinate'] <= 480]
        yellow_points_left = [point['xcordinate'] for point in filtered_data]


        filtered_data = [point for point in batch_data if point['BeaconType'] == 'YELLOWBR' and point['xcordinate'] <= 640 and point['ycordinate'] <= 480]
        yellow_points_right = [point['xcordinate'] for point in filtered_data]


        filtered_data = [point for point in batch_data if point['BeaconType'] == 'BlueTL' and point['xcordinate'] <= 640 and point['ycordinate'] <= 480]
        blue_points_left = [point['xcordinate'] for point in filtered_data]


        filtered_data = [point for point in batch_data if point['BeaconType'] == 'BlueBR' and point['xcordinate'] <= 640 and point['ycordinate'] <= 480]
        blue_points_right = [point['xcordinate'] for point in filtered_data]
 
        filtered_data = [point for point in batch_data if point['BeaconType'] == 'WL' and point['xcordinate'] <= 640 and point['ycordinate'] <= 480]
        WLx = [point['xcordinate'] for point in filtered_data]

        filtered_data = [point for point in batch_data if point['BeaconType'] == 'WL' and point['xcordinate'] <= 640 and point['ycordinate'] <= 480]
        WLy = [point['ycordinate'] for point in filtered_data]

        return red_points_left, red_points_right, yellow_points_left, yellow_points_right, blue_points_left, blue_points_right, WLx, WLy

 

    except Exception as e:
        print(e)
        return []

def postproc():
    rl,rr,yl,yr,bl,br,_,_ = fetch_data()
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
    print(xl)
    print(xr)
    return xl,xm,xr


a = 18
b = 26
xl,xm,xr = postproc()
rx = []
ry = []
for i in range(len(xl)):
    if(xl[i] ==0 or xm[i]==0 or xr[i]==0):
        continue  
    rxx,ryy = Angle2Distance.locationRoutine(a,b,xl[i],xm[i],xr[i])
    rx.append(rxx)
    ry.append(ryy)
print(rx)
print(ry)
