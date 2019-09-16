#!/usr/bin/env python
import rospy
import serial
from gps.msg import gpsMessage
import sys
import pynmea2

gpsmessage = gpsMessage()

def talker():
    pub = rospy.Publisher('gps_nmea', gpsMessage, queue_size = 10)
    rospy.init_node('gps', anonymous=False)
    serial_port = rospy.get_param('~port', '/dev/ttyUSB0')
    serial_baud = rospy.get_param('~baud', 115200)
    try:
	GPS = serial.Serial(port = serial_port, baudrate=serial_baud, timeout = 1)
	while not rospy.is_shutdown():
	    nmea = GPS.readline()
	    gpsmessage.raw=str(nmea.strip())
	    if nmea.startswith("$"):
		msg = pynmea2.parse(nmea)
		if hasattr(msg, 'latitude') and hasattr(msg, 'longitude'):
		    gpsmessage.lat = msg.latitude
		    gpsmessage.lon = msg.longitude
		if hasattr(msg, 'datetime'):
		    gpsmessage.timestamp = str(msg.datetime)

	    pub.publish(gpsmessage)    
    except serial.SerialException as ex:
        rospy.logfatal("Could not open serial port")
	    

if __name__ == '__main__':
    try:
	talker()
    except rospy.ROSInterruptException:
	pass


