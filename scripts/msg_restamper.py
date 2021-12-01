#!/usr/bin/env python
import rospy

from sensor_msgs.msg import Imu
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Image


class MsgRestamper:
    def __init__(self):

        self.msg_pub = rospy.Publisher("/synced_msg", SyncData, queue_size=10)

        # ==========================================#
        self.imu_in = False
        self.lidar_in = False
        self.rgb_image_in = False

        self.t_imu = 0.0
        self.t_lidar = 0.0
        self.t_rgb_image = 0.0

        self.imu_data = Imu()
        self.lidar_data = LaserScan()
        self.rgb_image_data = Image()
        # ==========================================#

        self.tolerance = 1e-1  # second

        self.is_data_synchronized = False

    def toSec(self, stamp):
        return stamp.secs + stamp.nsecs * (1e-9)

    def publish(self):


    def update_data(
        self,
    ):
        # ==========================================#
        all_data_received = self.lidar_in and self.imu_in and self.rgb_image_in  #!
        # print('all_data_received : ',all_data_received)

        if all_data_received:
            # print('t_rgb_image, t_imu, t_lidar : ',self.t_rgb_image,self.t_imu,self.t_lidar)
            t_all_data = [self.t_rgb_image, self.t_imu, self.t_lidar]  #!

            t_oldest = min(t_all_data)  # old
            t_youngest = max(t_all_data)  # young
            print("t_youngest-t_oldest :", t_youngest - t_oldest)
            if t_youngest - t_oldest < self.tolerance:
                self.is_data_synchronized = True
                self.publish_synced_msg()

                self.imu_in = False
                self.lidar_in = False
                self.rgb_image_in = False
        # ==========================================#

    def publish_synced_msg(self):

        sync_data = SyncData()
        # print('is_data_synchronized : ',self.is_data_synchronized)

        if self.is_data_synchronized:

            # ==========================================#
            sync_data.imu = self.imu_data
            sync_data.lidar = self.lidar_data
            sync_data.rgb_image = self.rgb_image_data
            # ==========================================#

            self.sync_msg_pub.publish(sync_data)

            self.is_data_synchronized = False

    # =====================================================================#
    def imu_sub_callback(self, data):
        # print('imu received')
        self.t_imu = self.toSec(data.header.stamp)
        self.imu_data = data
        self.imu_in = True
        self.update_data()

    def lidar_sub_callback(self, data):
        # print('lidar received')
        self.t_lidar = self.toSec(data.header.stamp)
        self.lidar_data = data
        self.lidar_in = True
        self.update_data()

    def rgb_image_sub_callback(self, data):
        # print('rgb_image received')
        # self.t_rgb_image=self.toSec(data.header.stamp)
        self.t_rgb_image = rospy.get_time()
        self.rgb_image_data = data
        self.rgb_image_in = True
        self.update_data()
    # =====================================================================#

    def start(self):
        rospy.init_node("msg_restamper", anonymous=False)

        # ====================================================================#
        rospy.Subscriber("/scan", LaserScan, self.lidar_sub_callback)
        rospy.Subscriber("/imu/data", Imu, self.imu_sub_callback)
        rospy.Subscriber("/camera/color/image_raw", Image, self.rgb_image_sub_callback)
        # ====================================================================#

        rospy.spin()


if __name__ == "__main__":
    MsgRestamper().start()

