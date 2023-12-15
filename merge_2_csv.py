import os
import numpy as np
import csv
import math

file_in2 = open('19.csv','r') #other
file_in1 = open('12.csv','r') #self
file = open('out.csv','w')
reader1 = csv.reader(file_in1)
reader2 = csv.reader(file_in2)
writer = csv.writer(file)
# flag = -1
rrow1 = next(reader1)
rrow2 = next(reader2)

def getDis(lat1, lng1, yaw1, lat2, lng2):
    x = abs(lng1-lng2)*111700*np.cos(lat1/180*np.pi)
    y = abs(lat1-lat2)*111700
    dis = np.sqrt(x*x +y*y)
    angle = math.atan2(x,y)/np.pi*180.0
    if angle<0:
        angle += 360
    angle -= yaw1
    if angle>=180:
        angle-=360
    if angle<=-180:
        angle+=360
    dis_x = dis*np.sin(angle*np.pi/180)
    dis_y = dis*np.cos(angle*np.pi/180)
    return dis_x, dis_y


while 1:
    # print(rrow1[0],rrow2[0])
    if rrow1[0]>rrow2[0]:
        print(rrow2[0]+'****************************')
        # data= [float(d) for d in rrow2]
        # writer.writerow(data)
        try:
            rrow2=next(reader2)
        except StopIteration:
            break
    elif rrow1[0] == rrow2[0]:
        data= [float(d) for d in rrow2]
        dis_x, dis_y = getDis(float(rrow1[1]),float(rrow1[2]),float(rrow1[3]),float(rrow2[1]),float(rrow2[2]))
        data.append(rrow1[1])
        data.append(rrow1[2])
        data.append(rrow1[3])
        data.append(rrow1[0])
        data.append(str(dis_x))
        data.append(str(dis_y))
        writer.writerow(data)
        try:
            rrow2=next(reader2)
        except StopIteration:
            break
        try:
            rrow1=next(reader1)
        except StopIteration:
            break
    elif rrow1[0]<rrow2[0]:
        print('****************************'+rrow1[0])
        # data = [0,0,0,0]
        # data.append(rrow1[1])
        # data.append(rrow1[2])
        # data.append(rrow1[0])
        # writer.writerow(data)
        try:
            rrow1=next(reader1)
        except StopIteration:
            # flag = 1
            break
# if flag==1:
#     while 1:
#         data= [float(d) for d in rrow2]
#         writer.writerow(data)
#         try:
#             rrow2=next(reader2)
#         except StopIteration:
#             break
# elif flag==0:
#     while 1:
#         data = [0,0,0,0]
#         data.append(rrow1[1])
#         data.append(rrow1[2])
#         data.append(rrow1[0])
#         writer.writerow(data)
#         try:
#             rrow1=next(reader1)
#         except StopIteration:
#             break





