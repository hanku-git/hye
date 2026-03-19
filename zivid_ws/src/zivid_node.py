#!/usr/bin/env python3

import os
import rclpy
import zivid
import numpy as np
from rclpy.node import Node

BASE_DIR = os.environ.get("ROBOT_BASE_DIR", os.path.expanduser("~"))
from cv_bridge import CvBridge

from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud2, PointField
from std_srvs.srv import Empty

class ZividCapture(Node):

    def __init__(self):
        super().__init__('zivid_node')

        app = zivid.Application()
        self.cv_bridge = CvBridge()
        print("Connecting to camera")
        self.camera = app.connect_camera()

        print("Configuring 3D settings")
        self.settings = zivid.Settings.load(BASE_DIR + "/zivid_ws/config/Zivid2+_Settings_Zivid_Two_Plus_M130_ManufacturingSpecular_60Hz.yml")

        self.srv = self.create_service(Empty, 'zivid/capture', self.capture_callback)
        self.img_pub = self.create_publisher(Image, 'zivid/color/image_color', 1)
        self.cloud_pub = self.create_publisher(PointCloud2, '/zivid/points/xyzrgba', 1)
        print("ZIVID Ready")

    def build_cloud_msg(self, point_cloud, xyz, rgba):
        msg = PointCloud2()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.height = point_cloud.height
        msg.width = point_cloud.width
        # msg.is_bigendian 
        # msg.fields.reverse(4)
        msg.fields.append(PointField(
                            name="x",
                            offset=0,
                            datatype=PointField.FLOAT32, count=1))
        msg.fields.append(PointField(
                            name="y",
                            offset=4,
                            datatype=PointField.FLOAT32, count=1))
        msg.fields.append(PointField(
                            name="z",
                            offset=8,
                            datatype=PointField.FLOAT32, count=1))
        msg.fields.append(PointField(
                            name="rgba",
                            offset=12,
                            datatype=PointField.UINT32, count=1))
        
        msg.is_dense = False
        msg.point_step = 28
        msg.row_step = (msg.point_step * msg.width)
        points = np.block([xyz,rgba])
        msg.data = np.asarray(points, np.float32).tobytes()

        return msg

    def capture_callback(self, req, res):
    
        print("Capturing frame")
        with self.camera.capture(self.settings) as frame:
            point_cloud = frame.point_cloud()
            xyz = point_cloud.copy_data("xyz")
            rgba = point_cloud.copy_data("rgba")

        cloud_msg = self.build_cloud_msg(point_cloud, xyz,rgba)
        img_msg = self.cv_bridge.cv2_to_imgmsg(rgba, 'rgba8')

        self.img_pub.publish(img_msg)
        self.cloud_pub.publish(cloud_msg)

        return res

def _main() -> None:    
    rclpy.init()

    zivid_capture = ZividCapture()

    rclpy.spin(zivid_capture)

    rclpy.shutdown()

if __name__ == "__main__":
    _main()