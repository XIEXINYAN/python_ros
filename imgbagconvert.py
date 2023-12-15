import rospy
import rosbag
import time
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage
import os
import roslib
import numpy as np



class extractImgData():
    def __init__(self, img_topic, bag_file):
        self.img_file = './row_test/image/'
        self.mkDir(self.img_file)
        self.img_topic = img_topic
        self.count = 0
        if isinstance(bag_file, list):
            for file in bag_file:
                print("now converting ...............................  ",file)
                try:
                    self.bag = rosbag.Bag(file,'r')
                    # info = self.bag.get_type_and_topic_info()
                    # print(info)
                    self.extract_img_from_bag()
                except:
                    print("file bag error ###############################  ",file)
                    continue
                print('##################################count#########################',self.count)
        elif bag_file.endswith('bag'):
            print("now converting   ",file)
            self.bag = rosbag.Bag(bag_file,'r')
            info = self.bag.get_type_and_topic_info()
            print(info)
            self.extract_img_from_bag()
        else:
            self.sub1 = rospy.Subscriber(self.img_topic, CompressedImage, self.callback_img)
            self.image = np.array([1080, 1920, 3])

    def callback_img(self, data):
        bridge = CvBridge()
        self.image = bridge.imgmsg_to_cv2(data, 'bgr8')



    def extract_img_from_bag(self):
        bag_data = self.bag.read_messages(self.img_topic)
        bridge = CvBridge()
        for topic, msg, t, in bag_data:
            try:
                self.image = bridge.imgmsg_to_cv2(msg, 'bgr8')
                # cv2.imshow('img',self.image)
                # cv2.waitKey(1)
                # timestr = '%.4f' % t.to_sec()
                # tt = timestr.split('.')
                # tf = int(tt[0])%1000*10000+int(tt[1])
                # print(tf)
                img_name = self.img_file+str(int(self.count/10))+'.jpg'
                if self.count%10 == 0:
                    # print(img_name)
                    cv2.imwrite(img_name,self.image)
                self.count=self.count+1
            except CvBridgeError as e:
                print(e)



    def mkDir(self, path):
        is_exists = os.path.exists(path)
        if not is_exists:
            os.makedirs(path)
            print('successful create '+ path)
            return True
        else:
            print(path + 'already exists !!')
            return False

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
        if os.path.isfile(file_path) and file_path.endswith('bag'):
            file_list.append(file_path)
        elif os.path.isdir(file_path):
            recursive_listdir(file_path)

    return file_list


def main(rosbag=False, rosonline=False):
    img_topic = '/image_front_raw2'
    if rosbag:
        file_list = recursive_listdir('./')
        print(file_list)
        extractImgData(img_topic, file_list)
    if rosonline:
        rospy.init_node('listener',anonymous=True)
        extractImgAndLidarData(img_topic)
        rospy.spin()

if __name__ == '__main__':
    main(True, False)
