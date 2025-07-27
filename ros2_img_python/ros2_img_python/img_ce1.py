#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
 
@说明: ROS2话题示例-发布图像话题
"""

import rclpy                        # ROS2 Python接口库
from rclpy.node import Node         # ROS2 节点类
from sensor_msgs.msg import Image   # 图像消息类型
from cv_bridge import CvBridge      # ROS与OpenCV图像转换类
import cv2                          # Opencv图像处理库

"""
创建一个发布者节点
ros2 run ce_python ce_img1
"""
class ImagePublisher(Node):

    def __init__(self, name):
        super().__init__(name)                                           # ROS2节点父类初始化
        self.publisher_ = self.create_publisher(Image, 'image_raw', 10)  # 创建发布者对象（消息类型、话题名、队列长度）
        self.timer = self.create_timer(0.01, self.timer_callback)         # 创建一个定时器（单位为秒的周期，定时执行的回调函数）
        self.path_ce1 = "/home/lbw/ros2_all/ros2_img_python/ros2_img_python/4.mp4"
        self.num = 0
        self.cap = cv2.VideoCapture(self.path_ce1)                             # 创建一个视频采集对象，驱动相机采集图像（相机设备号）
        self.cv_bridge = CvBridge()                                      # 创建一个图像转换对象，用于稍后将OpenCV的图像转换成ROS的图像消息

    def timer_callback(self):
        ret, frame = self.cap.read()                                     # 一帧一帧读取图像

        
        if ret == True:                                                  # 如果图像读取成功
            # 图像缩放
            frame = cv2.resize(frame, (640, 640))
            self.publisher_.publish(
                self.cv_bridge.cv2_to_imgmsg(frame, 'bgr8'))             # 发布图像消息
            self.num += 1
        else:
            self.cap = cv2.VideoCapture(self.path_ce1) 

        if self.num >= 5000:
            exit()
        self.get_logger().info(f'Publishing video frame {self.num}')                 # 输出日志信息，提示已经完成图像话题发布
        


def main(args=None):                                 # ROS2节点主入口main函数
    rclpy.init(args=args)                            # ROS2 Python接口初始化
    node = ImagePublisher("topic_webcam_pub")        # 创建ROS2节点对象并进行初始化
    rclpy.spin(node)                                 # 循环等待ROS2退出
    node.destroy_node()                              # 销毁节点对象
    rclpy.shutdown()                                 # 关闭ROS2 Python接口

