#!/usr/bin/env python

from threading import local
import numpy as np
import math
from numpy.core.fromnumeric import shape, size
from numpy.lib.function_base import append
from numpy.lib.type_check import asfarray
import rospy
from sensor_msgs import msg
import tf
import matplotlib
import matplotlib.pyplot as plt
import ros_numpy as rnp
from aam_common_msgs.msg import Cone
from aam_common_msgs.msg import ConeDetections
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from sensor_msgs.msg import LaserScan
import sys
import random

class pc2cam_transform():

    def __init__(self, namespace = 'pc2cam_transform'):
        rospy.init_node("pc2cam_transform", anonymous = True)
        
        self.no_ground_pc = rospy.Subscriber("/centroid_lidar_cones_marker",MarkerArray,self.centroid_callback)
        self.trans_pc = rospy.Publisher("/pc_translation_markers",MarkerArray, queue_size = 0)

        self.x_points = []
        self.y_points = []
        self.z_points = []






    def centroid_callback(self, centroid_msg):

        self.x_points = []
        self.y_points = []
        self.z_points = []

        for marker in centroid_msg.markers:
            x = marker.pose.position.x
            y = marker.pose.position.y
            z = 0

            self.x_points.append(x)
            self.y_points.append(y)
            self.z_points.append(z)

        self.matrix()

        return

        



    def matrix(self):
        #ZED - VELODYNE
        px = 0.3-1.7
        py=-0.00204-0
        pz=0-(-0.15)

        #Transformation matrix
        t = np.array([
            [1,0,0,px],
            [0,1,0,py],
            [0,0,1,pz],
            [0,0,0,1]
            ])

        output=[]
        outx=[]
        outy=[]
        outz=[]

        if len(self.x_points) != 0:
            for idx in range(len(self.x_points)-1):
                x = self.x_points[idx]
                y = self.y_points[idx]
                z = self.z_points[idx]



                #Matrix from pointcloud points
                u = np.array([x,y,z,1])

                #output matrix
                v = np.dot(t,u)
                output.append(v)

                outx.append(v[0])
                outy.append(v[1])
                outz.append(v[2])
                


        #plt.scatter(outx,outy)
        #plt.pause(0.01)

        self.rviz_pc_trans(outx,outy,outz)
        
        return   





    def rviz_pc_trans(self, avgx,avgy,avgz):
        self.rviz_msg = Marker()
        self.rviz_msg_array = MarkerArray()

        c = 0

        while c < len(avgx):
            count = 0
            MARKERS_MAX = 100

            self.rviz_msg = Marker()
            self.rviz_msg.header.frame_id = "base_link"
            self.rviz_msg.ADD
            self.rviz_msg.SPHERE
            self.rviz_msg.pose.position.x = avgx[c]
            self.rviz_msg.pose.position.y = avgy[c]
            self.rviz_msg.pose.position.z = avgz[c]
            self.rviz_msg.lifetime = rospy.Duration(0.5)
            self.rviz_msg.pose.orientation.w = 1
            self.rviz_msg.scale.x = 1
            self.rviz_msg.scale.y = 1
            self.rviz_msg.scale.z = 1
            self.rviz_msg.color.a = 1
            self.rviz_msg.color.r = 0
            self.rviz_msg.color.g = 0
            self.rviz_msg.color.b = 0
            self.rviz_msg.mesh_resource = "package://aamfsd_description/meshes/cone_blue.dae"
            self.rviz_msg.type = Marker.MESH_RESOURCE
            self.rviz_msg.mesh_use_embedded_materials = True


            if(count > MARKERS_MAX):
                self.rviz_msg_array.markers.pop(0)
            
            self.rviz_msg_array.markers.append(self.rviz_msg)
            m = 0
            id = 0
            for m in self.rviz_msg_array.markers:
                m.id = id
                id += 1
            
            c +=1

        self.trans_pc.publish(self.rviz_msg_array)    

        return





if __name__ == '__main__':
    try:
        pc2cam_transform()
    except rospy.ROSInterruptException:
        pass
    rospy.spin()