import rospy
import rosbag
import time
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Header
import os
import roslib
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2
import numpy as np
import pcl #sudo apt install python3-pcl
import pcl_msgs.msg
import csv

class extraGpsData():
    def __init__(self, topic, bag_file):
        self.bag_file = bag_file
        self.gps_topic = topic
        self.outfile = open('test/12.csv', 'w')
        self.writer = csv.writer(self.outfile)
        for file in bag_file:
            print("now converting ...............................  ",file)
            try:
                self.bag = rosbag.Bag(file)
                self.extract_gps_from_bag()
                self.bag.close()
            except:
                print("file bag error ###############################  ",file)
                continue
        self.outfile.close()

    def extract_gps_from_bag(self):
        bag_data = self.bag.read_messages(self.gps_topic)
        for topic, msg, t, in bag_data:
            timestr = '%.4f' % t.to_sec()
            tt = timestr.split('.')
            tf = int(tt[0])%1000*10000+int(tt[1])
            data=[tf, msg.latitude, msg.longitude, msg.dip]
            self.writer.writerow(data)



import os
# import zipfile
file_list = []
def recursive_listdir(path):
    files = os.listdir(path)
    for file in files:
        file_path = os.path.join(path, file)
        # print(path)
        # if file_path.endswith('zip'):
        #     zip_files = zipfile.ZipFile(file_path,'r')
        #     for filename in zip_files.namelist():
        #         #filename = os.path.join(path, filename)
        #         data = zip_files.read(filename)
        #         file = open(filename,'w+b')
        #         file.write(data)
        #         file.close()
        if os.path.isfile(file_path) and file_path.endswith('.bag'):
            file_list.append(file_path)
        elif os.path.isdir(file_path):
            recursive_listdir(file_path)

    return file_list


if __name__ == '__main__':
    gps_topic = '/rtk_test'
    # file_list = recursive_listdir('./')
    # print(file_list)
    # file_list = ['./19/2023-11-22_17-04-11_all.bag','./19/2023-11-22_17-09-55_all.bag']
    file_list = ['./12/2023_11_22_17_01_12_850_all.bag','./12/2023_11_22_17_04_11_524_all.bag','./12/2023_11_22_17_04_16_356_all.bag','./12/2023_11_22_17_09_57_234_all.bag']
    extraGpsData(gps_topic, file_list)
