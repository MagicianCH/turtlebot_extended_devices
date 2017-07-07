#!/usr/bin/env python
'''extended_imu ROS Node'''
# license removed for brevity
import rospy
import serial
import struct
import math
import numpy as np
from sensor_msgs.msg import Imu

def crc16_update(curcrc, bytess, lenth):
    ''' CRC '''
    crc = curcrc
    for j in range(lenth):
        byte = bytess[j]
        byte = struct.unpack('B', byte)
        d = byte[0]
        d = d << 8
        crc ^= d 
        crc &= 65535
        for i in range(8):
            temp = crc << 1
            if crc & 32768:
                temp ^= 4129
            crc = temp 
    crc = crc & 65535
    return crc

class IMU(object):
    ''' IMU class '''
    def __init__(self, device_info='HiPNUC HI219M'):
        ''' imu class init '''
        rospy.loginfo("IMU Device attached: %s", device_info)
        self.imu_msg = Imu()
        self.imu_msg.header.frame_id = "imu_link"
        self.pub = rospy.Publisher('imu_data', Imu, queue_size=10)

    def openSerial(self, port, baud, timeout=0):
        ''' open imu serial port '''
        try:
            self.device= serial.Serial(port, baud)
        except serial.SerialException as e:
            rospy.logerr("Error: cannot open serial port: %s" % e)
            rospy.signal_shutdown('Serial thread shutdown now')
            raise SystemExit
        rospy.loginfo('Serial opened successfully from %s at %d' % (port, baud))
    

    def readOnePackage(self):
        ''' imu read one package '''
        crc = 0
        while True: 
            prebyte = self.device.read(1)
            if prebyte[0] == b'\x5A':
                crc = crc16_update(crc, prebyte, 1)
                break
            else:
                pass

        while True:
            typebyte = self.device.read(1)
            if typebyte[0] == b'\xA5':
                crc = crc16_update(crc, typebyte, 1)
                break
            else:
                pass

        framelen = self.device.read(2)
        crc = crc16_update(crc, framelen, 2)
        framelen = struct.unpack('<H', framelen)
        datalen = framelen[0]
        #framecrc = self.device.read(2)
        framecrc = struct.unpack('<H', self.device.read(2))
        data = self.device.read(datalen)
        crc = crc16_update(crc, data, datalen)
        if crc == framecrc[0]:
            return datalen, data
        else:
            rospy.logerr("Imu package CRC error")
            return (0, None)

    def parseOnePackage(self, lenth, package):
        ''' imu parse one package '''
        offset = 0
        while lenth > 0:
            id = package[offset]
            if id == b'\x90':
                data = package[offset+1:offset+2]
                data = struct.unpack('<B', data)
                lenth -= 2
                offset += 2
                rospy.logdebug("ID: %x" % data)
            elif id == b'\xA0':
                data = package[offset+1:offset+7]
                data = struct.unpack('<3h', data)
                lenth -= 7
                offset += 7
                self.imu_msg.linear_acceleration.x = float(data[0] * 0.001 * 9.8)
                self.imu_msg.linear_acceleration.y = float(data[1] * 0.001 * 9.8)
                self.imu_msg.linear_acceleration.z = float(data[2] * 0.001 * 9.8)
                rospy.logdebug("acc: %d %d %d" % (data[0], data[1], data[2]))
            elif id == b'\xB0':
                data = package[offset+1:offset+7]
                data = struct.unpack('<3h', data)
                lenth -= 7
                offset += 7
                self.imu_msg.angular_velocity.x = math.radians(data[0] * 0.1)
                self.imu_msg.angular_velocity.y = math.radians(data[1] * 0.1)
                self.imu_msg.angular_velocity.z = math.radians(data[2] * 0.1)
                rospy.logdebug("ang: %d %d %d" % (data[0], data[1], data[2]))
            elif id == b'\xC0':
                lenth -= 7
                offset += 7
            elif id == b'\xD0': 
                lenth -= 7
                offset += 7
            elif id == b'\xD9':
                lenth -= 13
                offset += 13
            elif id == b'\xD1':
                data = package[offset+1:offset+17]
                data = struct.unpack('<4f', data)
                lenth -= 17
                offset += 17
                self.imu_msg.orientation.w = data[0]
                self.imu_msg.orientation.x = data[1]
                self.imu_msg.orientation.y = data[2]
                self.imu_msg.orientation.z = data[3]
                rospy.logdebug("ori %f %f %f %f" % data)
            else:
                rospy.logerr("something is wrong when parse a imu package")
                break
        
    def publishOnce(self):
        x = self.imu_msg.linear_acceleration.x
        y = self.imu_msg.linear_acceleration.y
        z = self.imu_msg.linear_acceleration.z
        q0 = self.imu_msg.orientation.w
        q1 = self.imu_msg.orientation.x
        q2 = self.imu_msg.orientation.y
        q3 = self.imu_msg.orientation.z
        r00 = 1-2*q2**2-2*q3**2 
        r01 = 2*q1*q2-2*q0*q3   
        r02 = 2*q1*q3+2*q0*q2

        r10 = 2*q1*q2+2*q0*q3   
        r11 = 1-2*q1**2-2*q3**2 
        r12 = 2*q2*q3-2*q0*q1

        r20 = 2*q1*q3-2*q0*q2   
        r21 = 2*q2*q3+2*q0*q1   
        r22 = 1-2*q1**2-2*q2**2
        
        R = np.mat([[r00, r01, r02], [r10, r11, r12], [r20, r21, r22]])
        RT = R.T
        g = np.mat([0, 0, 9.8]).T
        x1, y1, z1 = RT * g
        self.imu_msg.linear_acceleration.x = x - x1
        self.imu_msg.linear_acceleration.y = y - y1
        self.imu_msg.linear_acceleration.z = z - z1
        
        self.imu_msg.header.stamp = rospy.Time.now()
        self.pub.publish(self.imu_msg)

if __name__ == '__main__':
    rospy.init_node('extended_imu', anonymous=True)
    device_info = rospy.get_param('~device_info', 'HiPNUC HI219M')
    port = rospy.get_param('~port', '/dev/ttyUSB0')
    baud = int(rospy.get_param('~baud', '115200'))
    rate = rospy.Rate(100)
    imu = IMU(device_info)
    imu.openSerial(port, baud)
    
    while not rospy.is_shutdown():
        size, data = imu.readOnePackage()
        if size is not 0:
            imu.parseOnePackage(size, data)
            imu.publishOnce()
        else:
            pass
        rate.sleep()
