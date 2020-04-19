#!/usr/bin/env python
import rospy
from math import *
from spherical_scan_msgs.msg import Scan
from sensor_msgs.msg import PointCloud2
import std_msgs.msg
import sensor_msgs.point_cloud2 as pcl2

pcl_pub = rospy.Publisher("/spherical_cloud", PointCloud2, queue_size=1);

def callback(data):
    
    cloud_points = []
    for point in data.points:
        
        # extract coordinates
        position = point.position
        radius = point.position.radius
        azimuth = point.position.azimuth
        polar = point.position.polar

        # convert to x,y,z
        x = radius * cos(azimuth) * sin(polar)
        y = radius * sin(azimuth) * sin(polar)
        z = radius * cos(polar)
        
        # store in point 
        cloud_point = [x, y, z];
        
        # append to point list
        cloud_points.append(cloud_point)
        
    # publish point cloud

    header = data.header
    scaled_polygon_pcl = pcl2.create_cloud_xyz32(header, cloud_points)
    pcl_pub.publish(scaled_polygon_pcl)
        
    
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('test_spherical', anonymous=False)

    rospy.Subscriber("/velodyne_spherical_scan", Scan, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
