#!/usr/bin/env python3
# coding=utf-8

import rospy
from std_msgs.msg import String
import serial
import threading
import subprocess

class USBDataNode:
    def __init__(self):
        rospy.init_node('usb_data_node', anonymous=True)
        self.publisher_ = rospy.Publisher('usb_data', String, queue_size=10)
        self.serial_port = '/dev/ttyUSB0'  # 替换为你的USB串口设备
        self.serial_baud = 9600  # 替换为你的串口波特率
        self.serial_timeout = 1.0  # 超时时间，单位秒

        try:
            self.ser = serial.Serial(self.serial_port, self.serial_baud, timeout=self.serial_timeout)
            rospy.loginfo("Serial port %s opened successfully" % self.serial_port)
        except serial.SerialException:
            rospy.logerr("Failed to open serial port %s" % self.serial_port)
            raise

        # 尝试更改串口权限
        try:
            subprocess.run(['sudo', 'chmod', 'a+rw', self.serial_port], check=True)
            rospy.loginfo(f"Changed permissions for {self.serial_port} successfully.")
        except subprocess.CalledProcessError as e:
            rospy.logerr(f"Failed to change permissions for {self.serial_port}: {e}")
            raise

        # 使用线程安全的方式读取串口数据
        self.data_thread = threading.Thread(target=self.read_and_publish_data)
        self.data_thread.daemon = True
        self.data_thread.start()

    def read_and_publish_data(self):
        rate = rospy.Rate(125)  # 约8ms的频率，根据你的需要调整
        while not rospy.is_shutdown():
            if self.ser.isOpen():
                try:
                    line = self.ser.readline().decode('utf-8').strip()
                    rospy.loginfo("Received data: %s" % line)
                    self.publish_data(line)
                except serial.SerialException:
                    rospy.logerr("Error reading from serial port")
            else:
                rospy.logwarn("Serial port is not open")
            rate.sleep()

    def publish_data(self, data):
        msg = String()
        msg.data = data
        self.publisher_.publish(msg)
        rospy.loginfo("Published data: %s" % msg.data)

    def __del__(self):
        if self.ser.isOpen():
            self.ser.close()
            rospy.loginfo("Serial port closed")

if __name__ == '__main__':
    try:
        usb_data_node = USBDataNode()
        rospy.spin()
    except KeyboardInterrupt:
        pass
    finally:
        del usb_data_node
