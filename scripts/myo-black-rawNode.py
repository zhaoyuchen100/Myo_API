#!/usr/bin/env python

from __future__ import print_function

import enum
import re
import struct
import sys
import threading
import time
import math
import serial
import numpy
from serial.tools.list_ports import comports
from common import *
import rospy
from std_msgs.msg import String, UInt8, Header, MultiArrayLayout, MultiArrayDimension, Float64MultiArray
from geometry_msgs.msg import Quaternion, Vector3
from sensor_msgs.msg import Imu
from ros_myo_rv.msg import MyoArm, EmgPoints, EmgStamped

def multichr(ords):
    if sys.version_info[0] >= 3:
        return bytes(ords)
    else:
        return ''.join(map(chr, ords))

def multiord(b):
    if sys.version_info[0] >= 3:
        return list(b)
    else:
        return map(ord, b)

class Arm(enum.Enum):
    UNKNOWN = 0
    RIGHT = 1
    LEFT = 2

class XDirection(enum.Enum):
    UNKNOWN = 0
    X_TOWARD_WRIST = 1
    X_TOWARD_ELBOW = 2

class Pose(enum.Enum):
    REST = 0
    FIST = 1
    WAVE_IN = 2
    WAVE_OUT = 3
    FINGERS_SPREAD = 4
    THUMB_TO_PINKY = 5
    UNKNOWN = 255


class Packet(object):
    def __init__(self, ords):
        self.typ = ords[0]
        self.cls = ords[2]
        self.cmd = ords[3]
        self.payload = multichr(ords[4:])

    def __repr__(self):
        return 'Packet(%02X, %02X, %02X, [%s])' % \
            (self.typ, self.cls, self.cmd,
             ' '.join('%02X' % b for b in multiord(self.payload)))


class BT(object):
    '''Implements the non-Myo-specific details of the Bluetooth protocol.'''
    def __init__(self, tty):
        self.ser = serial.Serial(port=tty, baudrate=9600, dsrdtr=1)
        self.buf = []
        self.lock = threading.Lock()
        self.handlers = []

    ## internal data-handling methods
    def recv_packet(self, timeout=None):
        t0 = time.time()
        self.ser.timeout = None
        while timeout is None or time.time() < t0 + timeout:
            if timeout is not None: self.ser.timeout = t0 + timeout - time.time()
            c = self.ser.read()  #### c comes in from seriel port one by one  and in normal ASCII value !!!
            if not c:
                return None
            ret = self.proc_byte(ord(c)) # ord('a') = 97 in byte. ord(c) return unicode value
	    #print(ret) # here 'ret' is already a well converted packet. i.e. Packet(00,06,03,[00,00,00])
            if ret:
                if ret.typ == 0x80:
                    self.handle_event(ret)  # 0x80 is a event              #1
		    #print(ret)
                return ret

    def recv_packets(self, timeout=.5):## a stack of packets.....
        res = []
        t0 = time.time()
        while time.time() < t0 + timeout:
            p = self.recv_packet(t0 + timeout - time.time())
            if not p:
                return res
            res.append(p)
        return res

    def proc_byte(self, c): ## convert what we got in terms of unicode to byte i.e. 0x80
	
        if not self.buf:
            if c in [0x00, 0x80, 0x08, 0x88]:
                self.buf.append(c)  # buf is where we store each packet!!! stack one by one
            return None
        elif len(self.buf) == 1:
            self.buf.append(c)
            self.packet_len = 4 + (self.buf[0] & 0x07) + self.buf[1]
            return None
        else:
            self.buf.append(c)

        if self.packet_len and len(self.buf) == self.packet_len:
            p = Packet(self.buf)
            self.buf = []
            return p
        return None

    def handle_event(self, p):   #2
        for h in self.handlers:
            h(p)

    def add_handler(self, h):
        self.handlers.append(h)

    def remove_handler(self, h):
        try: self.handlers.remove(h)
        except ValueError: pass

    def wait_event(self, cls, cmd):
        res = [None]
        def h(p):
            if p.cls == cls and p.cmd == cmd:
                res[0] = p
        self.add_handler(h)
        while res[0] is None:
            self.recv_packet()
        self.remove_handler(h)
        return res[0]

    ## specific BLE commands
    def connect(self, addr):
        return self.send_command(6, 3, pack('6sBHHHH', multichr(addr), 0, 6, 6, 64, 0))

    def get_connections(self):
        return self.send_command(0, 6)

    def discover(self):
        return self.send_command(6, 2, b'\x01')

    def end_scan(self):
        return self.send_command(6, 4)

    def disconnect(self, h):
        return self.send_command(3, 0, pack('B', h))

    def read_attr(self, con, attr):
        self.send_command(4, 4, pack('BH', con, attr))
        return self.wait_event(4, 5)

    def write_attr(self, con, attr, val):
        self.send_command(4, 5, pack('BHB', con, attr, len(val)) + val)
        return self.wait_event(4, 1)  ## Ack message

    def send_command(self, cls, cmd, payload=b'', wait_resp=True):
        s = pack('4B', 0, len(payload), cls, cmd) + payload
        self.ser.write(s)

        while True:
            p = self.recv_packet()

            ## no timeout, so p won't be None
            if p.typ == 0:
                return p

            ## not a response: must be an event
            self.handle_event(p)


class MyoRaw(object):
    '''Implements the Myo-specific communication protocol.'''
	## on_pose is on fly whenever 

    def __init__(self, tty=None):
	#self.white_myo_addr = [150, 49, 222, 148, 48, 192]
	self.black_myo_addr = [102, 221, 12, 68, 251, 209]
        if tty is None:
            tty = self.detect_tty()
        if tty is None:
            raise ValueError('Myo dongle not found!')

        self.bt = BT(tty)
        self.conn = None
        self.emg_handlers = []
        self.imu_handlers = []
        self.arm_handlers = []
        self.pose_handlers = []

    def detect_tty(self):
        for p in comports():
            if re.search(r'PID=2458:0*1', p[2]):
                print('using device:', p[0])
                return p[0]

        return None

    def run(self, timeout=None):
        self.bt.recv_packet(timeout)

    def connect(self):
        ## stop everything from before
        self.bt.end_scan()
        self.bt.disconnect(0)
        self.bt.disconnect(1)
        self.bt.disconnect(2)

        ## start scanning
        print('scanning...')
        self.bt.discover()
        while True:
            p = self.bt.recv_packet()
            print('scan response:', p)

            if p.payload.endswith(b'\x06\x42\x48\x12\x4A\x7F\x2C\x48\x47\xB9\xDE\x04\xA9\x01\x00\x06\xD5'):
                addr = list(multiord(p.payload[2:8]))
		if addr == self.black_myo_addr:
                	break
        self.bt.end_scan()

        ## connect and wait for status event
        conn_pkt = self.bt.connect(addr)
        self.conn = multiord(conn_pkt.payload)[-1]
        self.bt.wait_event(3, 0)

        ## get firmware version
        fw = self.read_attr(0x17)
        _, _, _, _, v0, v1, v2, v3 = unpack('BHBBHHHH', fw.payload)
        print('firmware version: %d.%d.%d.%d' % (v0, v1, v2, v3))

        #self.write_attr(0x19, pack('BBBBHBBBBB', 2, 9, 2, 1, C, emg_smooth, C // emg_hz, imu_hz, 0, 0))
        name = self.read_attr(0x03)
        print('device name: %s' % name.payload)
	
        # self.write_attr(0x19, b'\x01\x03\x00\x01\x01')
        self.start_raw()

        ## add data handlers
        def handle_data(p):
            if (p.cls, p.cmd) != (4, 5): return

	    #print(p)
            c, attr, typ = unpack('BHB', p.payload[:4])
            pay = p.payload[5:]
            if attr == 0x2b:
                vals = unpack('16b', pay)
                ## each int8_t byte is one emg channel, intotal 2 arrays of emg signals stored in one packet! 
                emg1 = vals[:8]
                self.on_emg(emg1)
		rospy.sleep(0.001)
		emg2 = vals[8:]
		self.on_emg(emg2)
	    elif attr == 0x2e:
		vals = unpack('16b', pay)
                ## each int8_t byte is one emg channel, intotal 2 arrays of emg signals stored in one packet! 
                emg1 = vals[:8]
                self.on_emg(emg1)
		rospy.sleep(0.001)
		emg2 = vals[8:]
		self.on_emg(emg2)
	    elif attr == 0x31:
		vals = unpack('16b', pay)
                ## each int8_t byte is one emg channel, intotal 2 arrays of emg signals stored in one packet! 
                emg1 = vals[:8]
                self.on_emg(emg1)
		rospy.sleep(0.001)
		emg2 = vals[8:]
		self.on_emg(emg2)
	    elif attr == 0x34:
		vals = unpack('16b', pay)
                ## each int8_t byte is one emg channel, intotal 2 arrays of emg signals stored in one packet! 
                emg1 = vals[:8]
                self.on_emg(emg1)
		rospy.sleep(0.001)
		emg2 = vals[8:]
		self.on_emg(emg2)
            elif attr == 0x1c:
                vals = unpack('10h', pay)
                quat = vals[:4]
                acc = vals[4:7]
                gyro = vals[7:10]
                self.on_imu(quat, acc, gyro)
            elif attr == 0x23: ## classifier event data recieved in a myohw_att_handle_classifier event attribut 0x23
                typ, val, xdir = unpack('3B', pay[:3])

                if typ == 1: # on arm (arm synced)
		    print("arm synced!")
                    self.on_arm(Arm(val), XDirection(xdir))
                elif typ == 2: # removed from arm (arm unsynced)
                    self.on_arm(Arm.UNKNOWN, XDirection.UNKNOWN)
                elif typ == 3: # pose
                    self.on_pose(Pose(val))
		elif typ == 4: # event_unlocked
	 	    print("i am unlocked")
		elif typ == 5:
		    print("i am locked")
		elif typ == 6:
		    print("sync failed")

            else:
                print('data with unknown attr: %02X %s' % (attr, p))

        self.bt.add_handler(handle_data) #1


    def write_attr(self, attr, val):
        if self.conn is not None:
            self.bt.write_attr(self.conn, attr, val)

    def read_attr(self, attr):
        if self.conn is not None:
            return self.bt.read_attr(self.conn, attr)
        return None

    def disconnect(self):
        if self.conn is not None:
            self.bt.disconnect(self.conn)

    def start_raw(self):
        '''Sending this sequence for v1.0 firmware seems to enable both raw data and
        pose notifications.
        '''
	self.write_attr(0x12,b'\x01\x00')
	self.write_attr(0x2c,b'\x01\x00')
	self.write_attr(0x2f,b'\x01\x00')
	self.write_attr(0x32,b'\x01\x00')
	self.write_attr(0x35,b'\x01\x00')
        ## enable IMU data
        self.write_attr(0x1d, b'\x01\x00')
        ## enable on/off arm notifications
        self.write_attr(0x24, b'\x02\x00')
        #self.write_attr(0x28, b'\x01\x00')
        #self.write_attr(0x19, b'\x01\x03\x00\x01\x01')
        self.write_attr(0x19, b'\x01\x03\x02\x01\x01') 
					 #this# enable filtered emg!!#  '\x02' 
	#self.write_attr(0x19, pack('BBBBHBBBBB', 10, 2, 1, 1, 200, 100 ,20, 50, 0, 0))
        self.write_attr(0x19, b'\x0a\x02\x01\x01\xc8\x00\x64\x14\x32\x00\x00')
	#set sleep mode
        self.write_attr(0x19, pack('3B', 9, 1, 1))

    def vibrate(self, length): ## 0x00 19   00   03      03       01     03
			       ##     att      length   vib_mode length long_vib
        if length in xrange(1, 4):
            ## first byte tells it to vibrate; purpose of second byte is unknown
	    
            self.write_attr(0x19, pack('3B', 3, 1, length))
	    #print("hello")

    def add_emg_handler(self, h):
	
        self.emg_handlers.append(h)

    def add_imu_handler(self, h):
        self.imu_handlers.append(h)

    def add_pose_handler(self, h):
        self.pose_handlers.append(h)

    def add_arm_handler(self, h):
        self.arm_handlers.append(h)


    def on_emg(self, emg):  #emg is data h is the handler that handle that data.  here handler is the publisher.
        for h in self.emg_handlers:
            h(emg)

    def on_imu(self, quat, acc, gyro):
        for h in self.imu_handlers:
            h(quat, acc, gyro)

    def on_pose(self, p):
        for h in self.pose_handlers:
            h(p)

    def on_arm(self, arm, xdir):
        for h in self.arm_handlers:
            h(arm, xdir)
    #def unlock(self):
	#print("unlocking myo")
	#self.write_attr(0x19, b'\x0a\x01\x00')
class upsmplThread (threading.Thread):
	def __init__(self,threadID,name,counter):
		threading.Thread.__init__(self)
		self.threadID = threadID
		self.name = name
		self.counter = counter
		self.emg_up = EmgPoints()
		imu_data = Imu()
		self.imu_up = [imu_data.orientation,imu_data.orientation_covariance,imu_data.angular_velocity,imu_data.angular_velocity_covariance,imu_data.linear_acceleration,imu_data.linear_acceleration_covariance]
		self.emgPub = rospy.Publisher('myo_black_emg_upsmpl', EmgStamped, queue_size=10)
		self.imuPub = rospy.Publisher('myo_black_imu_upsmpl', Imu, queue_size=10)
	def run(self):
		print ("Myo_black_upsample thread starting "+self.name)
		rate = rospy.Rate(1000)
		times = []
		try:
			while not rospy.is_shutdown():
				h = Header()
       				h.stamp = rospy.Time.now()
        			h.frame_id = 'myo_black'
				emg_ = EmgStamped(h,self.emg_up)
				imu_ = Imu(h,self.imu_up[0],self.imu_up[1],self.imu_up[2],self.imu_up[3],self.imu_up[4],self.imu_up[5])
		    		self.emgPub.publish(emg_)
				self.imuPub.publish(imu_)
				times.append(time.time())
				#if len(times) > 1000: ##20
				    #print(len(times))
				    #print((len(times) - 1) / (times[-1] - times[0]))
				    #times[:] = []
				rate.sleep()

    		except (rospy.ROSInterruptException, serial.serialutil.SerialException) as e:
        		pass
		
if __name__ == '__main__':
    # Start by initializing the Myo and attempting to connect. 
    # If no Myo is found, we attempt to reconnect every 0.5 seconds
    connected = 0;
    print("Initializing...")
    while(connected == 0):
        try:
            m = MyoRaw(sys.argv[1] if len(sys.argv) >= 2 else None)
            connected = 1;
        except (ValueError, KeyboardInterrupt) as e:
            print("Myo Armband not found. Attempting to connect...")
            rospy.sleep(0.5)
            pass

    # Define Publishers
    imuPub = rospy.Publisher('myo_black_imu', Imu, queue_size=10)
    emgPub = rospy.Publisher('myo_black_emg', EmgStamped, queue_size=10)
    armPub = rospy.Publisher('myo_black_arm', MyoArm, queue_size=10)
    gestPub = rospy.Publisher('myo_black_gest', UInt8, queue_size=10)


    rospy.init_node('myo_black_raw', anonymous=True)
    #thread1= upsmplThread(1,"upsmpl thread",1)
    h_g = Header()
    flag = True
    # Package the EMG data into an EmgArray
    def proc_emg(emg,times=[]): # here emg is still an array
	global flag 
	global h_g
	#print(flag)
	if flag:
		h = Header()
        	h.stamp = rospy.Time.now()
        	h.frame_id = 'black_myo'
		h_g = h
		flag = False
        ## create an array of ints for emg data
        ##### have to initiaze and allocate value to EmgPoints then give these to EmgStamped
	emg_p = EmgPoints()
	emg_p.data1 = emg[0]
	emg_p.data2 = emg[1]
	emg_p.data3 = emg[2]
	emg_p.data4 = emg[3]
	emg_p.data5 = emg[4]
	emg_p.data6 = emg[5]
	emg_p.data7 = emg[6]
	emg_p.data8 = emg[7]

	#emg_p.data1 = abs(emg[0])-1
	#emg_p.data2 = abs(emg[1])-1
	#emg_p.data3 = abs(emg[2])-1
	#emg_p.data4 = abs(emg[3])-1
	#emg_p.data5 = abs(emg[4])-1
	#emg_p.data6 = abs(emg[5])-1
	#emg_p.data7 = abs(emg[6])-1
	#emg_p.data8 = abs(emg[7])-1
	emg_ = EmgStamped(h_g,emg_p)
	#print(emg_)
	#thread1.emg_up = emg_p
        emgPub.publish(emg_)
	h_g.stamp = h_g.stamp + rospy.Duration(0.005) 
        ## print framerate of received data
        times.append(time.time())
	#print (len(times))
        if len(times) > 500:
            print((len(times) - 1) / (times[-1] - times[0]))
	    #print (rospy.Time.now()-h_g.stamp)
	    #h_g.stamp = rospy.Time.now() 
	    flag = True
            del times[:]
    # Package the IMU data into an Imu message
    def proc_imu(quat1, acc, gyro):
        # New info: https://github.com/thalmiclabs/myo-bluetooth/blob/master/myohw.h#L292-L295
        # Scale values for unpacking IMU data
        # define MYOHW_ORIENTATION_SCALE   16384.0f ///< See myohw_imu_data_t::orientation
        # define MYOHW_ACCELEROMETER_SCALE 2048.0f  ///< See myohw_imu_data_t::accelerometer
        # define MYOHW_GYROSCOPE_SCALE     16.0f    ///< See myohw_imu_data_t::gyroscope
        h = Header()
        h.stamp = rospy.Time.now()
        h.frame_id = 'black_myo'
        # We currently do not know the covariance of the sensors with each other
        cov = [0, 0, 0, 0, 0, 0, 0, 0, 0]
        quat = Quaternion(quat1[1]/16384.0, quat1[2]/16384.0, quat1[3]/16384.0, quat1[0]/16384.0)
        ## Normalize the quaternion and accelerometer values
        quatNorm = math.sqrt(quat.x*quat.x+quat.y*quat.y+quat.z*quat.z+quat.w*quat.w)
        normQuat = Quaternion(quat.x/quatNorm, quat.y/quatNorm, quat.z/quatNorm, quat.w/quatNorm)
        normAcc = Vector3(acc[0]/2048.0, acc[1]/2048.0, acc[2]/2048.0)
        normGyro = Vector3(gyro[0]/16.0, gyro[1]/16.0, gyro[2]/16.0)
        imu = Imu(h, normQuat, cov, normGyro, cov, normAcc, cov)
        imuPub.publish(imu)

    # Package the arm and x-axis direction into an Arm message	
    def proc_arm(arm, xdir):
        #When the arm state changes, publish the new arm and orientation
        calibArm=MyoArm(arm.value, xdir.value)
        armPub.publish(calibArm)

    # Publish the value of an enumerated gesture
    def proc_pose(p):
        gestPub.publish(p.value)

    m.add_emg_handler(proc_emg)#register publishing method here
    m.add_imu_handler(proc_imu)
    m.add_arm_handler(proc_arm)
    m.add_pose_handler(proc_pose)

    m.connect()
    m.vibrate(1)
    #thread1.start()
    #m.unlock()
    try:

        while not rospy.is_shutdown():
            m.run(1)
	    
	    #m.vibrate(1)

    except (rospy.ROSInterruptException, serial.serialutil.SerialException) as e:
        pass
    finally:
        print()
        print("Disconnecting...")
        m.disconnect()
        print()
