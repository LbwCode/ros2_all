#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
@说明: ROS2服务示例-提供目标识别服务
"""

import rclpy                                           # ROS2 Python接口库
from rclpy.node import Node                            # ROS2 节点类
from sensor_msgs.msg import Image                      # 图像消息类型
import numpy as np                                     # Python数值计算库
from cv_bridge import CvBridge                         # ROS与OpenCV图像转换类
import cv2                                             # Opencv图像处理库
from interface_all.srv import GetObjectPosition   # 自定义的服务接口

class ImageSubscriber(Node):
    def __init__(self, name):
        # 调用父类Node的构造函数，初始化节点名称
        super().__init__(name)                                          
        # 创建图像订阅者，订阅名为'image_raw'的话题
        self.sub = self.create_subscription(
            Image, 'image_raw', self.listener_callback, 10)             
        # 创建CVBridge对象，用于ROS Image消息与OpenCV图像的转换
        self.cv_bridge = CvBridge()                                     
        
        # 创建服务端，提供名为'get_target_position'的服务
        self.srv = self.create_service(GetObjectPosition,               
                                       'get_target_position',
                                       self.object_position_callback)    
        # 初始化目标位置变量
        self.objectX = 0
        self.objectY = 0                              

    def object_detect(self, image):
        # 简单示例：每次调用时递增目标位置坐标
        # 实际应用中应替换为真实的目标检测算法
        self.objectX += 1
        self.objectY += 1

        # 显示处理后的图像（调试用）
        cv2.imshow("object", image)                                  
        cv2.waitKey(1)

    def listener_callback(self, data):
        # 图像订阅回调函数，每次收到图像消息时被调用
        self.get_logger().info('Receiving video frame')               
        # 将ROS Image消息转换为OpenCV BGR格式图像
        image = self.cv_bridge.imgmsg_to_cv2(data, 'bgr8')            
        # 调用目标检测函数处理图像
        self.object_detect(image)                                      

    def object_position_callback(self, request, response):
        # 服务请求回调函数，处理客户端的服务请求
        if request.get == True:
            # 如果请求有效，返回当前目标位置
            response.x = self.objectX                                
            response.y = self.objectY
            self.get_logger().info('Object position\nx: %d y: %d' %
                                   (response.x, response.y))          
        else:
            # 如果请求无效，返回默认值
            response.x = 0
            response.y = 0
            self.get_logger().info('Invalid command')                 
        return response

def main(args=None):                                
    # 初始化ROS2 Python客户端库
    rclpy.init(args=args)                            
    # 创建服务端节点实例
    node = ImageSubscriber("service_object_server")  
    # 进入循环，保持节点运行并处理ROS2事件
    rclpy.spin(node)                                
    # 销毁节点并释放资源
    node.destroy_node()                             
    # 关闭ROS2 Python客户端库
    rclpy.shutdown()                                