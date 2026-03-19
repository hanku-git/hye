#!/usr/bin/env python3

import threading
import sys
import time
import os
import rclpy
import zivid
import numpy as np
from rclpy.node import Node
from cv_bridge import CvBridge

from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud2, PointField
from std_srvs.srv import Empty
from bin_picking_msgs.msg import ScanCommandMsg

# https://github.com/Box-Robotics/ros2_numpy
import ros2_numpy

BASE_DIR = os.environ.get("ROBOT_BASE_DIR", os.path.expanduser("~"))
SCANNER_MODEL = "zivid2_mr"
class ZividCaptureNode(Node):

    def __init__(self):
        super().__init__('zivid_node') # inheritance from ZIVID SDK
        self.node = rclpy.create_node('zivid_node_sub')
        self.init_set()

        app = zivid.Application()
        self.cv_bridge = CvBridge()
        print("Connecting to camera")
        self.camera = app.connect_camera()
        print("Configuring 3D settings")
        self.settings = zivid.Settings.load(BASE_DIR + "/zivid_ws/config/" + SCANNER_MODEL + "/scan_settings_default.yml")
        self.img_pub = self.create_publisher(Image, '/zivid/color/image_color', 1)
        self.cloud_pub = self.create_publisher(PointCloud2, '/zivid/points/xyzrgba', 1)
        print("...")
        print("SCANNER_MODEL: ")
        print(SCANNER_MODEL)
        print("...")
        print("ZIVID Ready")

        self._last_msg = None
        self._msg_lock = threading.Lock()
        self._last_msg_2 = None
        self._msg_lock_2 = threading.Lock()

    def init_set(self):
        self._publish_rate = self.node.declare_parameter('~publish_rate', 100).value

    def setScanSetting(self):
        print("Configuring 3D settings")
        filename = BASE_DIR + "/zivid_ws/config/" + SCANNER_MODEL  + "/scan_settings_" + self._target_name + ".yml"
        if os.path.exists(filename):
            self.settings = zivid.Settings.load(filename)
            print(filename)
            print('Scan Setting loaded! (object: ' + self._target_name + ', Scanner: '+ SCANNER_MODEL + ')')
        else:
            self.settings = zivid.Settings.load(BASE_DIR + "/zivid_ws/config/" + SCANNER_MODEL + "/scan_settings_default.yml")
            print(filename)
            print('Scan Setting loaded! (object: unknown' + ', Scanner: '+ SCANNER_MODEL + ')')

    def run(self):
        sub = self.node.create_subscription(ScanCommandMsg, '/zivid/capture2',
                                    self._scan_callback, 1)

        rate = self.node.create_rate(self._publish_rate)
        while rclpy.ok():            
            rclpy.spin_once(self.node)
            if self._msg_lock.acquire(False):
                msg = self._last_msg
                self._last_msg = None
                self._msg_lock.release()
            else:
                rate.sleep()
                continue
            
            if msg is not None:
                print('3D Scanning Start!')
                # self.init_set()

                # Load scan setting
                self._class_names = ['BG', msg.target_name]
                self._target_name = msg.target_name
                print('Target object: ' + msg.target_name)
                self.setScanSetting()

                # Scan
                start_scan = time.time()                
                self.capture() # zivid2 scan (image & point cloud)
                end_scan = time.time()      
                print(f"scan runtime: {end_scan - start_scan:.5f} sec")    
                print('3D Scanning Finished!')
            else:
                rate.sleep()
                continue
            print('--')

    def build_cloud_msg(self, point_cloud, xyz, rgba):


        #########################################################################
        #### 기존 - msg.data = np_tmp.tobytes()에서 4초 소요됨
        # s_t = time.time()      
        # msg = PointCloud2()
        # msg.header.stamp = self.get_clock().now().to_msg()
        # e_t0 = time.time()      
        # print(f"build_cloud_msg (1-1): {e_t0 - s_t:.5f} sec")    
        # msg.header.frame_id = "map"
        # msg.height = point_cloud.height
        # msg.width = point_cloud.width
        # # msg.is_bigendian 
        # # msg.fields.reverse(4)
        # e_t1 = time.time()      
        # print(f"build_cloud_msg (1-2): {e_t1 - e_t0:.5f} sec")    
        # msg.fields.append(PointField(
        #                     name="x",
        #                     offset=0,
        #                     datatype=PointField.FLOAT32, count=1))
        # msg.fields.append(PointField(
        #                     name="y",
        #                     offset=4,
        #                     datatype=PointField.FLOAT32, count=1))
        # msg.fields.append(PointField(
        #                     name="z",
        #                     offset=8,
        #                     datatype=PointField.FLOAT32, count=1))
        # msg.fields.append(PointField(
        #                     name="rgba",
        #                     offset=12,
        #                     datatype=PointField.UINT32, count=1))
        # e_t2 = time.time()      
        # print(f"build_cloud_msg (1-3): {e_t2 - e_t1:.5f} sec")    
        # msg.is_dense = False
        # msg.point_step = 28
        # msg.row_step = (msg.point_step * msg.width)
        # e_t3 = time.time()      
        # print(f"build_cloud_msg (1-4): {e_t3 - e_t2:.5f} sec")    
        # points = np.block([xyz,rgba])
        # e_t4 = time.time()      
        # print(f"build_cloud_msg (1-5): {e_t4 - e_t3:.5f} sec")   
        # # msg.data = np.asarray(points, np.float32).tobytes()
        # np_tmp = np.asarray(points, np.float32)
        # e_t5 = time.time()      
        # print(f"build_cloud_msg (1-6): {e_t5 - e_t4:.5f} sec")  
        # msg.data = np_tmp.tobytes()
        # e_t6 = time.time()      
        # print(f"build_cloud_msg (1-7): {e_t6 - e_t5:.5f} sec")  

        #########################################################################
        #### 변경 - 스캔 시간 외의 소요 시간 대폭 줄임

        # ZIVID1
        # xyz = xyz.reshape([1200*1920, 3])
        # rgba = rgba.reshape([1200*1920, 4])

        # ZIVID2
        xyz = xyz.reshape([1024*1224, 3])
        rgba = rgba.reshape([1024*1224, 4])

        
        x = xyz[:,0]
        y = xyz[:,1]
        z = xyz[:,2]
        r = rgba[:,0]
        g = rgba[:,1]
        b = rgba[:,2]
        npoints = len(x)

        points_arr = np.zeros((npoints,), dtype=[
            ('x', np.float32),
            ('y', np.float32),
            ('z', np.float32),
            ('r', np.uint8),
            ('g', np.uint8),
            ('b', np.uint8)])
        points_arr['x'] = x
        points_arr['y'] = y
        points_arr['z'] = z
        points_arr['r'] = r
        points_arr['g'] = g
        points_arr['b'] = b
   
        msg = ros2_numpy.msgify(PointCloud2, points_arr)

        return msg

    def capture(self):
        print("Capturing frame")
        s_t = time.time()                
        with self.camera.capture(self.settings) as frame:
            point_cloud = frame.point_cloud()
            xyz = point_cloud.copy_data("xyz")
            rgba = point_cloud.copy_data("rgba")
        e_t0 = time.time()      
        print(f"camera.capture: {e_t0 - s_t:.5f} sec")    
        
        cloud_msg = self.build_cloud_msg(point_cloud, xyz,rgba)
        e_t1 = time.time()      
        print(f"build_cloud_msg: {e_t1 - e_t0:.5f} sec")    
        
        img_msg = self.cv_bridge.cv2_to_imgmsg(rgba, 'rgba8')
        e_t2 = time.time()      
        print(f"cv2_to_imgmsg: {e_t2 - e_t1:.5f} sec")    

        self.img_pub.publish(img_msg)
        self.cloud_pub.publish(cloud_msg)


    def _scan_callback(self, msg):
        self.node.get_logger().debug("Scanning command!")
        if self._msg_lock.acquire(False):
            self._last_msg = msg
            self._msg_lock.release()

def main():
    rclpy.init(args=sys.argv)

    node = ZividCaptureNode()
    node.run()

if __name__ == '__main__':
    main()
