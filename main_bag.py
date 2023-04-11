# coding:utf-8
from radar.radar_bag import RadarObjectAnalysis
# import rospy
def main():
    topic = "/front_radar/mid_objects"
    path = "/media/xxy/US100 1TB/radar/2022-01-06-14-26-39行人纵向.bag"
    RadarObjectAnalysis(topic, path)
if __name__ == "__main__":
    main()

