#!/usr/bin/env python

import rospy

from geographic_msgs.msg import GeoPoint
from std_msgs.msg import Float64

from smarc_msgs.msg import ThrusterFeedback

class TopicThrottler:

    def __init__(self):
        # Publishers.
        self.lat_lon_pub = rospy.Publisher('dr/lat_lon/throttled', GeoPoint, queue_size=1)
        self.roll_pub = rospy.Publisher('dr/roll/throttled', Float64, queue_size=1)
        self.pitch_pub = rospy.Publisher('dr/pitch/throttled', Float64, queue_size=1)
        self.yaw_pub = rospy.Publisher('dr/yaw/throttled', Float64, queue_size=1)
        self.thruster1_pub = rospy.Publisher('core/thruster1_feedback/throttled', ThrusterFeedback, queue_size=1)
        self.thruster2_pub = rospy.Publisher('core/thruster2_feedback/throttled', ThrusterFeedback, queue_size=1)

        # Subscribers.
        self.lat_lon_sub = rospy.Subscriber('dr/lat_lon', GeoPoint, self.lat_lon_callback)
        self.roll_sub = rospy.Subscriber('dr/roll', Float64, self.roll_callback)
        self.pitch_sub = rospy.Subscriber('dr/pitch', Float64, self.pitch_callback)
        self.yaw_sub = rospy.Subscriber('dr/yaw', Float64, self.yaw_callback)
        self.thruster1_sub = rosyp.Subscriber('core/thruster1_fb', ThrusterFeedback, self.thruster1_callback)
        self.thruster2_sub = rosyp.Subscriber('core/thruster2_fb', ThrusterFeedback, self.thruster2_callback)

    def lat_lon_callback(self, msg):
        #print("Quat callback!")
        self.quat = msg.quaternion

    def depth_callback(self, msg):
        self.altitude = -msg.data

    def lat_lon_callback(self, msg):
        odom = LatLonOdometry()
        odom.lat_lon_pose.position.latitude = 180./math.pi*msg.latitude
        odom.lat_lon_pose.position.longitude = 180./math.pi*msg.longitude
        odom.lat_lon_pose.position.altitude = self.altitude

        print "Got ", msg.latitude, msg.longitude, self.altitude

        #print "Quat in lat lon: ", self.quat
        if self.quat is not None:
            odom.lat_lon_pose.orientation = self.quat
            #print "Adding odom: ", self.quat
        else:
            odom.lat_lon_pose.orientation.w = 1.

        try:
            translate_odom = rospy.ServiceProxy('lat_lon_to_utm_odom', LatLonToUTMOdometry)
            resp = translate_odom(odom)
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)
            return
        print "Got ", resp.odom.pose.pose.position
        self.pub.publish(resp.odom)

if __name__ == '__main__':
    rospy.init_node('old_pose_converter')
    conv = OldPoseConverter()
    rospy.spin()
