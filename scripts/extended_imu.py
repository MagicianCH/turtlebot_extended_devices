#!/usr/bin/env python
'''extended_imu ROS Node'''
# license removed for brevity
import rospy
import serial
import struct
import math
import time
import numpy as np
from sensor_msgs.msg import Imu

class IMU(object):
    ''' IMU class '''
    def __init__(self):
        ''' imu class init '''
        self.port = rospy.get_param('~port', '/dev/ttyUSB0')
        self.baud = int(rospy.get_param('~baud', '115200'))
        self.frame_id = rospy.get_param('~frame_id', 'imu_link')
        self.use_linear = bool(rospy.get_param('~use_linear', 'true'))
        self.device_info = rospy.get_param('~device_info', 'HiPNUC HI219M')
        self.imu_msg = Imu()
        self.imu_msg.header.frame_id = self.frame_id
        self.euler = [0.0, 0.0, 0.0]
        self.pub = rospy.Publisher('imu_data', Imu, queue_size=10)

    def openSerial(self, timeout=200):
        ''' open imu serial port '''
        try:
            self.device= serial.Serial(self.port, self.baud, timeout=0.5)
        except serial.SerialException as e:
            rospy.logerr("Error: cannot open serial port: %s" % e)
            rospy.signal_shutdown('Serial thread shutdown now')
            raise SystemExit
        rospy.loginfo("IMU Device attached: %s", self.device_info)
        rospy.loginfo('Serial opened successfully from %s at %d' % (self.port, self.baud))
    

    def readOnePackage(self):
        ''' imu read one package '''
        crc = 0
        while True: 
            prebyte = self.device.read(1)
            if prebyte[0] == b'\x5A':
                crc = self.crc16_update(crc, prebyte, 1)
                break
            else:
                pass

        while True:
            typebyte = self.device.read(1)
            if typebyte[0] == b'\xA5':
                crc = self.crc16_update(crc, typebyte, 1)
                break
            else:
                pass

        framelen = self.device.read(2)
        crc = self.crc16_update(crc, framelen, 2)
        framelen = struct.unpack('<H', framelen)
        datalen = framelen[0]
        #framecrc = self.device.read(2)
        framecrc = struct.unpack('<H', self.device.read(2))
        data = self.device.read(datalen)
        crc = self.crc16_update(crc, data, datalen)
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
                rospy.logdebug("acc: %d %d %d" % (data[0], data[1], data[2]))
            elif id == b'\xA1':
                data = package[offset+1:offset+7]
                data = struct.unpack('<3h', data)
                lenth -= 7
                offset += 7
                if self.use_linear is False:
                    self.imu_msg.linear_acceleration.x = float(data[0] * 0.001 * 9.8)
                    self.imu_msg.linear_acceleration.y = float(data[1] * 0.001 * 9.8)
                    self.imu_msg.linear_acceleration.z = float(data[2] * 0.001 * 9.8)
                rospy.logdebug("accfilter: %d %d %d" % (data[0], data[1], data[2]))
            elif id == b'\xA5':
                data = package[offset+1:offset+7]
                data = struct.unpack('<3h', data)
                lenth -= 7
                offset += 7
                if self.use_linear is True:
                    self.imu_msg.linear_acceleration.x = float(data[0] * 0.001 * 9.8)
                    self.imu_msg.linear_acceleration.y = float(data[1] * 0.001 * 9.8)
                    self.imu_msg.linear_acceleration.z = float(data[2] * 0.001 * 9.8)
                rospy.logdebug("linear acc: %d %d %d" % (data[0], data[1], data[2]))
            elif id == b'\xA6':
                data = package[offset+1:offset+7]
                data = struct.unpack('<3h', data)
                lenth -= 7
                offset += 7
            elif id == b'\xB0':
                data = package[offset+1:offset+7]
                data = struct.unpack('<3h', data)
                lenth -= 7
                offset += 7
                rospy.logdebug("ang: %d %d %d" % (data[0], data[1], data[2]))
            elif id == b'\xB1':
                data = package[offset+1:offset+7]
                data = struct.unpack('<3h', data)
                lenth -= 7
                offset += 7
                self.imu_msg.angular_velocity.x = math.radians(data[0] * 0.1)
                self.imu_msg.angular_velocity.y = math.radians(data[1] * 0.1)
                self.imu_msg.angular_velocity.z = math.radians(data[2] * 0.1)
                rospy.logdebug("angfilter: %d %d %d" % (data[0], data[1], data[2]))
            elif id == b'\xC0':
                data = package[offset+1:offset+7]
                data = struct.unpack('<3h', data)
                lenth -= 7
                offset += 7
            elif id == b'\xC1':
                data = package[offset+1:offset+7]
                data = struct.unpack('<3h', data)
                lenth -= 7
                offset += 7
            elif id == b'\xD0': 
                data = package[offset+1:offset+7]
                data = struct.unpack('<3h', data)
                lenth -= 7
                offset += 7
                self.euler[0] = data[0]/100.0
                self.euler[1] = data[1]/100.0
                self.euler[2] = data[2]/10.0
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
                rospy.logdebug("ori %.3f %.3f %.3f %.3f" % data)
            else:
                rospy.logerr("something is wrong when parse a imu package")
                break
        
    def publishOnce(self):
        self.imu_msg.header.stamp = rospy.Time.now()
        self.pub.publish(self.imu_msg)

    def crc16_update(self, curcrc, bytess, lenth):
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

    def deviceConfigure(self):
        configure = bool(rospy.get_param('~configure', 'false'))
        if configure is False:
            return
        odr = rospy.get_param('~imu_odr', '100')
        baud = rospy.get_param('~imu_baud', '115200')
        mode = rospy.get_param('~imu_mode', '0')
        uid = rospy.get_param('~imu_uid', '0')
        dtype = rospy.get_param('~imu_data_type', '90,A1,A5,B1,D1')
        self.device.write('AT+EOUT=0\r\n')
        time.sleep(0.2)
        self.device.flushInput()
        
        rospy.loginfo('IMU: Setting data output frequency...')
        self.device.write('AT+ODR=' + str(odr) +'\r\n')
        ack = self.device.readlines()
        for x in ack:
            rospy.loginfo(x.strip())
        time.sleep(0.1)

        rospy.loginfo('IMU: Setting user ID...')
        self.device.write('AT+ID=' + str(uid) + '\r\n')
        ack = self.device.readlines()
        for x in ack:
            rospy.loginfo(x.strip())
        time.sleep(0.1)
        
        rospy.loginfo('IMU: Setting mode...')
        self.device.write('AT+MODE=' + str(mode) + '\r\n')
        ack = self.device.readlines()
        for x in ack:
            rospy.loginfo(x.strip())
        time.sleep(0.1)

        rospy.loginfo('IMU: Setting output data type...')
        self.device.write('AT+SETPTL=' + str(dtype) + '\r\n')
        ack = self.device.readlines()
        for x in ack:
            rospy.loginfo(x.strip())
        time.sleep(0.1)

        self.device.write('AT+BAUD='+str(baud)+'\r\n')
        rospy.loginfo('IMU: Device configure done!')

if __name__ == '__main__':
    rospy.init_node('extended_imu', anonymous=True)
    rate = rospy.Rate(300)
    imu = IMU()
    imu.openSerial()
    imu.deviceConfigure()
    
    while not rospy.is_shutdown():
        size, data = imu.readOnePackage()
        if size is not 0:
            imu.parseOnePackage(size, data)
            imu.publishOnce()
        else:
            pass
        rate.sleep()
