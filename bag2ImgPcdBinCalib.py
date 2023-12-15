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




class extractImgAndLidarData():
    def __init__(self, img_topic, lidar_topic, bag_file):
        self.img_file = './row_test/image/'
        self.lidar_pcd = './row_test/lidar_pcd/'
        self.mkDir(self.img_file)
        self.mkDir(self.lidar_pcd)

        self.img_topic = img_topic
        self.lidar_topic = lidar_topic
        self.image_lasttimestamp=0
        self.image_timestamp=0

        self.count = 0
        self.calib = self.readCameraCfg('cam.yaml')
        self.image = np.array([1080, 1920, 3])
        self.remap_image = np.array([1080, 1920, 3])
        # print(self.calib)
        self.mapx, self.mapy = cv2.initUndistortRectifyMap(self.calib['CameraMat'],self.calib['DisCoffes'],None,self.calib['CameraMat'],(1920,1080),5)
        # print(self.mapy, self.mapx)
        if isinstance(bag_file, list):
            for file in bag_file:
                print("now converting ...............................  ",file)
                try:
                    self.bag = rosbag.Bag(file)
                    # info = self.bag.get_type_and_topic_info()
                    # print(info)
                    # self.extract_img_from_bag()
                    # self.extract_pcd_from_bag()
                    self.extract_imgpcd_from_bag()
                    self.bag.close()
                except:
                    print("file bag error ###############################  ",file)
                    continue
                print('##################################count#########################',self.count)
        elif bag_file.endswith('.bag'):
            print("now converting   ",file)
            self.bag = rosbag.Bag(bag_file,'r')
            # info = self.bag.get_type_and_topic_info()
            # print(info)
            self.extract_imgpcd_from_bag()
        else:
            print("###########################################online###########################")
            self.sub1 = rospy.Subscriber(self.img_topic, Image, self.callback_img)
            self.sub2 = rospy.Subscriber(self.lidar_topic, PointCloud2, self.callbak_lidar)

    def callback_img(self, data):
        t =data.header.stamp
        timestr = '%.4f' % t.to_sec()
        tt = timestr.split('.')
        tf = int(tt[0])%1000*10000+int(tt[1])
        # print(tf)
        self.image_timestamp = tf
        bridge = CvBridge()
        self.image = bridge.imgmsg_to_cv2(data, 'bgr8')
        # cv2.imshow("img",self.image)
        # cv2.waitKey(1)



    def callbak_lidar(self, data):
        assert isinstance(data, PointCloud2)
        cloud = pcl.PointCloud()
        points = []
        gen_points = point_cloud2.read_points(data, field_names=('x','y','z','intensity'), skip_nans = True)
        for p in gen_points:
            points.append([p[0],p[1],p[2]])
        cloud.from_list(points)
        if self.image_timestamp!=self.image_lasttimestamp:
            cnt = str(self.count).zfill(6)
            image_name = self.img_file + cnt +'.jpg'
            remap_image = cv2.remap(self.image, self.mapx, self.mapy, cv2.INTER_LINEAR)
            cv2.imwrite(image_name, remap_image)
            pcl.save(cloud,self.lidar_pcd+ cnt + '.pcd')
            self.image_lasttimestamp = self.image_timestamp
            self.count = self.count + 1
            print(self.count)
            points = np.array(points)
            # print(self.calib['ExtrinsicRVec'], self.calib['ExtrinsicTVec'], self.calib['CameraMat'], self.calib['DisCoffes'])
            pts2d, _ = cv2.projectPoints(points, self.calib['ExtrinsicRVec'], self.calib['ExtrinsicTVec'], self.calib['CameraMat'], self.calib['DisCoffes'])
            # 在图像上绘制点
            for p in pts2d:
                center = (int(p[0][0]), int(p[0][1]))
                # print(center)
                if center[0]<0 or center[0]>1919 or center[1]<0 or center[1]>1079:
                    continue
                cv2.circle(remap_image, center, 3, (0, 0, 255), -1)
            cv2.imshow('image',remap_image)
            cv2.waitKey(1)
            # print(self.calib)


    def readCameraCfg(self, yamlpath):
        cv_file = cv2.FileStorage(yamlpath, cv2.FILE_STORAGE_READ)

        res = {
            "CameraMat": cv_file.getNode("CameraMat").mat(),
            "DisCoffes": cv_file.getNode("DisCoffes").mat(),
            "ExtrinsicRVec": cv_file.getNode("ExtrinsicRVec").mat(),
            "ExtrinsicTVec": cv_file.getNode("ExtrinsicTVec").mat(),
            "LidarDisCoffes":np.array([[0,0,0,0,0]])
        }
        # print(res)
        return res

    def extract_img_from_bag(self):
        bag_data = self.bag.read_messages(self.img_topic)
        bridge = CvBridge()
        for topic, msg, t, in bag_data:
            self.image = bridge.imgmsg_to_cv2(msg, 'bgr8')
            timestr = '%.4f' % t.to_sec()
            tt = timestr.split('.')
            tf = int(tt[0])%1000*10000+int(tt[1])
            # print(tf)
            self.image_timestamp=tf
            # cv2.imshow('image_raw',self.image)
            # cv2.waitKey(1)
        self.bag.close()

    def extract_pcd_from_bag(self):
        bag_data = self.bag.read_messages(self.lidar_topic)
        for topic, msg, t, in bag_data:
            cloud = pcl.PointCloud()
            points = []
            gen_points = point_cloud2.read_points(msg, field_names=('x','y','z','intensity'), skip_nans = True)
            for p in gen_points:
                points.append([p[0],p[1],p[2]])
            cloud.from_list(points)
            pcl.save(cloud,self.lidar_pcd+ cnt + '.pcd')
        self.bag.close()

    def extract_imgpcd_from_bag(self):
        bag_data = self.bag.read_messages()
        bridge = CvBridge()
        for topic, msg, t, in bag_data:
            # print('#########',topic==self.lidar_topic, topic, self.lidar_topic)
            if topic==self.img_topic:
                # print(msg)
                self.image = bridge.imgmsg_to_cv2(msg, 'bgr8')
                # print("################11111111111111111111111####################################")
                timestr = '%.4f' % t.to_sec()
                tt = timestr.split('.')
                tf = int(tt[0])%1000*10000+int(tt[1])
                # print('################2222222222222222222222#############',tf)
                self.image_timestamp=tf
                # cv2.imshow('image_raw',self.image)
                # # cv2.waitKey(1)
            elif topic==self.lidar_topic:
                # print(msg)
                cloud = pcl.PointCloud()
                points = []
                gen_points = point_cloud2.read_points(msg, field_names=('x','y','z','intensity'), skip_nans = True)
                for p in gen_points:
                    points.append([p[0],p[1],p[2]])
                cloud.from_list(points)
                # print(self.image_timestamp,self.image_lasttimestamp)
                if self.image_timestamp!=self.image_lasttimestamp:
                    # print("###################￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥#################################")
                    cnt = str(self.count).zfill(6)
                    image_name = self.img_file + cnt +'.jpg'
                    self.remap_image = cv2.remap(self.image, self.mapx, self.mapy, cv2.INTER_LINEAR)
                # #     remap_image = self.image
                    cv2.imwrite(image_name, self.remap_image)
                    pcl.save(cloud,self.lidar_pcd+ cnt + '.pcd')
                    self.image_lasttimestamp = self.image_timestamp
                    self.count = self.count + 1
                    print(self.count)
                    points = np.array(points)
                    # print(self.calib['ExtrinsicRVec'], self.calib['ExtrinsicTVec'], self.calib['CameraMat'], self.calib['DisCoffes'])
                    d=self.calib['DisCoffes']
                    for i in np.arange(8):
                        d[0][i]=0
                    pts2d, _ = cv2.projectPoints(points, self.calib['ExtrinsicRVec'], self.calib['ExtrinsicTVec'], self.calib['CameraMat'], d)
                    # 在图像上绘制点
                    for p in pts2d:
                        center = (int(p[0][0]), int(p[0][1]))
                        # print(center)
                        if center[0]<0 or center[0]>1919 or center[1]<0 or center[1]>1079:
                            continue
                        cv2.circle(self.remap_image, center, 3, (0, 0, 255), -1)
                    cv2.imshow('image',self.remap_image)
                    cv2.waitKey(1)
        self.bag.close()

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
    img_topic = "/image_front_raw2"
    # img_header = '/image_front_raw2/header'
    lidar_topic = "lidar_left/raw_cloud"
    if rosbag:
        file_list = recursive_listdir('./')
        # print(file_list)
        extractImgAndLidarData(img_topic, lidar_topic, file_list)
    if rosonline:
        # print("###########################################")
        rospy.init_node('listener',anonymous=True)
        extractImgAndLidarData(img_topic, lidar_topic)
        rospy.spin()

if __name__ == '__main__':
    main(True, False)
