# -*- coding: utf-8 -*-
"""
Spyder Editor

bag包抽帧清洗，根据情况选取topic进行处理
"""

import os
import rosbag
import numpy as np
import sensor_msgs.point_cloud2 as pc2
import pandas as pd
import cv2
from cv_bridge import CvBridge
#import open3d as p3d



bag_path = '/home/wxw/wangxuewen/bag/'
bag_name = '1.bag'
bag = rosbag.Bag(bag_path+bag_name,'r')

#获取bag包中的topics
info = bag.get_type_and_topic_info()
print('bag info ',info)
topics_in_bag = []

for key in info.topics:
    topics_in_bag.append(key)

print(topics_in_bag)

topic = {}
image_topic=[]
if '/lidar_left/raw_cloud' in topics_in_bag:
    topic[1] = '/lidar_left/raw_cloud'
if '/lidar_right/raw_cloud' in topics_in_bag:
    topic[2] = '/lidar_right/raw_cloud'
if '/front_raw_image' in topics_in_bag:
    image_topic = '/front_raw_image'
'''
if '/front_lidar/mid_calibrate_cloud' in topics_in_bag:
    topic[3] = '/front_lidar/mid_calibrate_cloud'


if '/front_left_lidar/raw_cloud' in topics_in_bag:
    topic[1] = '/front_left_lidar/raw_cloud'
if '/front_right_lidar/raw_cloud' in topics_in_bag:
    topic[2] = '/front_right_lidar/raw_cloud'
if '/front_mid_lidar/raw_cloud' in topics_in_bag:
    topic[3] = '/front_mid_lidar/raw_cloud'
'''
#该bag包均为calibrate点云，只需将其叠加，抽帧即可
#先读取所需topic的sensor_msgs,存至map
pc_points_map = {}
time_map = {}
topic_map = {}
for key in topic:
    topic_to_read = topic[key]
    t_time = []
    points = []
    topic_array = []
    for topics, msg, t in bag.read_messages(topics=topic_to_read):
        t = t.to_sec()   #转化为sec
        data = pc2.read_points(msg)
        #print(topics,'', t)
        points.append(np.array(list(data)))
        t_time.append(t)
        topic_array.append(topics)
    pc_points_map[key] = points
    time_map[key] = t_time
    topic_map[key] = topic_array
#print("left, ", time_map[1])
#print("---------------------------")
#print("right, ", time_map[2])
#print("---------------------------")
#print("image, ", time_map[2])

if len(image_topic) > 0:
    image_data=[]
    topic_to_read = image_topic
    t_time = []
    topic_array=[]
    bridge = CvBridge()
    for topice,msg,t in bag.read_messages(topics=topic_to_read):
        t = t.to_sec()
        # tmp = bridge.compressed_imgmsg_to_cv2(msg,"bgr8")
        tmp = bridge.imgmsg_to_cv2(msg,"bgr8")
        image_data.append(tmp)
        t_time.append(t)
        # topic_array.append(topics)
        topic_array.append(topice)
        # import ipdb;ipdb.set_trace()
    image_time_array = t_time
index = np.arange(np.shape(pc_points_map[1])[0])
#print("wxw")
# 1s取1个点，找参考时间点，选择左雷达时间点为参考
freq = 10  #10Hz
refer_time_list = time_map[1]
selected_index = np.arange(0,index.shape[0],freq)
selected_index = selected_index.tolist()

def find_min_value(list_1):
    min_value = min(list_1)
    min_index = list_1.index(min_value)
    
    return min_value, min_index

#左右雷达时间对齐

right_index = []
middle_index = []
image_index = []
t_diff_th = 0.06
t_diff_th = 0.1
start_t1 = 0
end_t1 = 0
start_t2 = 0
end_t2 = 0
start_t3 = 0
end_t3 = 0
for i in selected_index:
    right_t = []
    middle_t = []
    image_t = []
    t = refer_time_list[i]
    while(start_t1 < len(time_map[2]) and abs(time_map[2][start_t1] - t) > 1):
        start_t1 += 1
    end_t1 = min(len(time_map[2]),start_t1+20)
    right_t = list(abs(np.array(time_map[2][start_t1:end_t1]) - t))
    '''
    while(start_t2 < len(time_map[3]) and abs(time_map[3][start_t2] - t) > 1):
        start_t2 += 1
    end_t2 = min(len(time_map[3]),start_t2+20)
    middle_t = list(abs(np.array(time_map[3][start_t2:end_t2]) - t))
    '''
    while(start_t3 < len(image_data) and abs(image_time_array[start_t3] - t) > 1):
        start_t3 += 1
    end_t3 = min(len(image_time_array),start_t3+20)
    image_t = list(abs(np.array(image_time_array[start_t3:end_t3]) - t))
    
    if(len(right_t) == 0 or len(image_t) == 0 ):
    #or len(middle_t) == 0):
        selected_index.remove(i)
        continue
    right_diff, right_diff_index = find_min_value(right_t)
#    middle_diff, middle_diff_index = find_min_value(middle_t)
    image_diff, image_diff_index = find_min_value(image_t)
    #判断左雷达该帧0.06s内有无右雷达及中间雷达的帧
    if right_diff < t_diff_th and image_diff < t_diff_th :
    #and middle_diff < t_diff_th:
        #存在所需帧，记下该帧index
        right_index.append(start_t1+right_diff_index)
#        middle_index.append(start_t2+middle_diff_index)
        image_index.append(start_t3+image_diff_index)
    else:
        selected_index.remove(i)
    #print(i,t,right_diff,middle_diff,image_diff)

#print(selected_index,right_index)

#是否能逐帧查看，按键选择是否保存
#Open3D不支持xyzi格式点云，pclpy不支持python2.7，rosbag对python3支持较差
#对比此处读取的点云xyz和i坐标值和转成csv后用matlab处理
#存储格式：

def save_cloud_to_csv(cloud_list, index,save_path,prefix_name):
    cloud = []
    count = 0
    if not os.path.exists(save_path):
        os.makedirs(save_path)
    for i in index:
        count += 1
        cloud = cloud_list[i]
        save_name = prefix_name + '-' + str(count) + '.csv'
        data = pd.DataFrame(data=cloud)
        data.to_csv(save_path+'/'+save_name)
#有camera的时候用------------------------------------------------
def save_image_to_png(image_data, index,save_path,prefix_name):
    image = []
    count = 0
    if not os.path.exists(save_path):
        os.makedirs(save_path)
    for i in index:
        count += 1
        image = image_data[i]
        save_name = prefix_name + '-' + str(count) + '.jpg'
        # import ipdb;ipdb.set_trace()
        cv2.imwrite(save_path+'/'+save_name, image)
#-----------------------------------------------------------------
selected_index = list(selected_index)
save_root_path = bag_path.replace('/bag/','/bagag/')
save_path_1 = save_root_path + 'left'
save_path_2 = save_root_path + 'right'
#save_path_3 = save_root_path + 'middle'
save_path_4 = save_root_path + 'image'
prefix_name = bag_name.split('.bag')[0]
save_cloud_to_csv(pc_points_map[1],selected_index,save_path_1,prefix_name)
# import ipdb;ipdb.set_trace()
save_cloud_to_csv(pc_points_map[2],right_index,save_path_2,prefix_name)
# import ipdb;ipdb.set_trace()
#save_cloud_to_csv(pc_points_map[3],middle_index,save_path_3,prefix_name)
save_image_to_png(image_data, image_index, save_path_4,prefix_name)


