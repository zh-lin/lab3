#!/usr/bin/python

# 2.12 Lab 3 me212bot_node: ROS driver running on the pc side to read and send messages to Arduino
# Peter Yu Sept 2016
# modified by Daniel
DEMO=0
import rospy
import threading
import serial
import tf.transformations as tfm
from geometry_msgs.msg import Pose, Quaternion
import math
import helper
from me212bot.msg import WheelCmdVel

if DEMO==1:
    from Adafruit_MotorHAT import Adafruit_MotorHAT


## main function (Need to modify)
def main():

    rospy.init_node('me212bot', anonymous=True)
    if DEMO==1:
        serialComm = serial.Serial('/dev/ttyACM0', 115200, timeout = 5)
        Motorhat = Adafruit_MotorHAT(addr = 0x60)
        leftMotor = Motorhat.getMotor(1)
        rightMotor = Motorhat.getMotor(2)
        odometry_thread = threading.Thread(target = read_odometry_loop)
        odometry_thread.start()
    
    ## 1. Initialize a subscriber (subscribe ROS topic)
    cmdvel_sub = rospy.Subscriber('/cmdvel', WheelCmdVel, cmdvel_callback)
    
    rospy.spin()


## msg handling function (Need to modify)
def cmdvel_callback(msg):  
    print "cmdvel callback"
    ## unpack msg and interpret it to adafruit motor commands 
    ## adafruit motor initialization: see line 15~18
    ## adafruit motor cmds: leftMotor.setSpeed(), leftMotor.run()
    
    return


# read_odometry_loop() is for reading odometry from Arduino and publish to rostopic. (No need to modify)
def read_odometry_loop():
    prevtime = rospy.Time.now()
    while not rospy.is_shutdown():
        # get a line of string that represent current odometry from serial
        serialData = serialComm.readline()
        
        # split the string e.g. "0.1,0.2,0.1" with cammas
        splitData = serialData.split(',')
        
        # parse the 3 split strings into 3 floats
        try:
            x     = float(splitData[0])
            y     = float(splitData[1])
            theta = float(splitData[2])
            
            hz    = 1.0 / (rospy.Time.now().to_sec() - prevtime.to_sec())
            prevtime = rospy.Time.now()
            
            print 'x=', x, ' y=', y, ' theta =', theta, ' hz =', hz
            
            # publish odometry as Pose msg
            odom = Pose()
            odom.position.x = x
            odom.position.y = y
            
            qtuple = tfm.quaternion_from_euler(0, 0, theta)
            odom.orientation = Quaternion(qtuple[0], qtuple[1], qtuple[2], qtuple[3])
        except:
            # print out msg if there is an error parsing a serial msg
            print 'Cannot parse', splitData
            

if __name__=='__main__':
    main()


