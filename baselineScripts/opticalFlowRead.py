#!/usr/bin/env python
import rospy
import math

from  mavros_msgs.msg import OpticalFlowRad

def callback(msg):
    x = msg.integrated_x
    y = msg.integrated_y
    z = msg.distance
    rospy.loginfo("\nOptical Flow x: {} \n Optical Flow y: {} \n Optical Flow z: {} \n ---".format(x, y, z))



def main():
    rospy.init_node('opticalFlowRead')   # make ros node

    rospy.Subscriber("/mavros/px4flow/raw/optical_flow_rad", OpticalFlowRad, callback)  #subscribe to position messages
    rospy.spin()    #keeps python from exiting until this node is stopped


if __name__ == '__main__':
    main()
