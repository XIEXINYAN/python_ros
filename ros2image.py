# coding:utf-8

import rospy
import time
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import roslib  # roslib.load_manifest(PKG)
import numpy as np
from PIL import Image
import os
class DirMake():

    def __init__(self):
        self.SAVE_PATH = './'
        self.SAVE_CLASS = '/calib/'
        self.big_image_path = self.SAVE_PATH + self.SAVE_CLASS+'big_image/'
        self.mkDir(self.big_image_path)
        self.small_image_path = self.SAVE_PATH + self.SAVE_CLASS+'small+image/'
        self.mkDir(self.small_image_path)


    def mkDir(self, path):
        # path = path.strip()# 去除尾部 \ 符号
        # path = path.rstrip("\\")
        # 判断路径是否存在
        # 存在     True
        # 不存在   False
        isExists = os.path.exists(path)
        # 判断结果
        if not isExists:
            # 如果不存在则创建目录
            # 创建目录操作函数
            os.makedirs(path)
            print(path + ' 创建成功')
            return True
        else:
            # 如果目录存在则不创建，并提示目录已存在
            print(path + ' 目录已存在')
            return False

    @property
    def getBigImagePath(self):
        return self.big_image_path

    @property
    def getSamllImagePath(self):
        return self.small_image_path


class ImageSave():

    def __init__(self, image_path, topic):
        self.sub = rospy.Subscriber(topic, Image, self.callback_image)
        self.save_image_file = image_path
        # self.save_image_timestamp_file = os.path.abspath(os.path.join(self.save_image_file, ".."))+'/timestamp.txt'
        self.count = 0
    def callback_image(self, data):
            bridge = CvBridge()
            cv_image=bridge.imgmsg_to_cv2(data, "bgr8")
            # cv_image = bridge.compressed_imgmsg_to_cv2(data, "bgr8")
            # timestr = "%.6f" % data.header.stamp.to_sec()
            # print("cam list is : ",self.count)
            # with open(self.save_image_timestamp_file, mode="a") as f:
            #     f.writelines(timestr + "\n")
            # f.close()
            image_name = self.save_image_file+str(self.count).zfill(10)+".png"
            self.count = self.count +1
            cv2.imwrite(image_name, cv_image)
            cv2.waitKey(3)

def main(big_path, small_path):
    rospy.init_node('listener', anonymous=True)
    #save image class
    ImageSave(big_path,"/image_front_raw2")

    #save point depth class
    ImageSave(small_path,"/image_front_raw")
    rospy.spin()

if __name__ == "__main__":
    d = DirMake()
    main(d.getBigImagePath,d.getSamllImagePath)
