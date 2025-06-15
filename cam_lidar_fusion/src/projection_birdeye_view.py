#!/usr/bin/env python

from ctypes import pointer
from mimetypes import init
from re import T
from threading import local
from unittest import result
import numpy as np
import math

import cv2
from cv_bridge import CvBridge, CvBridgeError
from numpy.core.fromnumeric import shape, size
from numpy.lib.function_base import append
from numpy.lib.type_check import asfarray
import rospy
from sensor_msgs import msg
import tf
import matplotlib
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import ros_numpy as rnp
from aam_common_msgs.msg import Cone
from aam_common_msgs.msg import ConeDetections
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from sensor_msgs.msg import LaserScan
import sys
import random
from darknet_ros_msgs.msg import BoundingBoxes
from darknet_ros_msgs.msg import BoundingBox
import PIL


class projection_birdeye_view():

    
    def __init__(self, namespace = 'projection_birdeye_view'):
        rospy.init_node("projection_birdeye_view", anonymous = True)
        
        self.lidar_cones_sub = rospy.Subscriber("/pc_translation_markers",MarkerArray,self.lidar_cones_callback)
        self.cam_sub = rospy.Subscriber("/zed/right/image_raw",Image,self.img_callback)
        self.box_msg = rospy.Subscriber("/darknet_ros/bounding_boxes", BoundingBoxes, self.box_pipeline)

        self.cones_in_frame_x = []
        self.cones_in_frame_y = []
        self.cones_in_frame_color = []
        self.points=[]






    def lidar_cones_callback(self, L_cones_msg):
        cone_msg = Marker()
        
        self.points=[]

        for marker in L_cones_msg.markers:
            x = marker.pose.position.x
            y = marker.pose.position.y
            z = marker.pose.position.z
            self.points.append([x,y,1])
            


        
            


    

    def img_callback(self, img):
        bridge = CvBridge()

        # Try to convert the ROS Image message to a CV2 Image
        try:
            cv_image = bridge.imgmsg_to_cv2(img, "passthrough")
        except CvBridgeError:
            rospy.logerr("CvBridge Error: error happened in conversion")

        # Show the converted image
        cv_image = cv2.cvtColor(cv_image,cv2.COLOR_RGB2BGR)
        raw_image = cv_image
        

        project_pc = projection(cv_image,raw_image,self.points,self.cones_in_frame_x,self.cones_in_frame_y,self.cones_in_frame_color)
        project_pc.projection_pipeline()

        self.points = []

        return





        
    def box_pipeline(self,box_msg):



        self.cones_in_frame_color = []
        self.cones_in_frame_x = []
        self.cones_in_frame_y = []

        for box  in box_msg.bounding_boxes:
            x_min = box.xmin
            y_min = box.ymin
            x_max = box.xmax
            y_max = box.ymax             
            cone_color = box.Class
            x_center = ((x_max-x_min)/2)+x_min
            y_center = ((y_max-y_min)/2)+y_min

            #print(x_center,y_center,cone_color)

            self.cones_in_frame_x.append(x_center)
            self.cones_in_frame_y.append(y_center)
            self.cones_in_frame_color.append(cone_color)











class projection():
    def __init__(self,cv_image,raw_image,points,cones_in_frame_x,cones_in_frame_y,cones_in_frame_color):
        self.cv_image = cv_image
        self.raw_image = raw_image
        self.points = points
        self.cones_in_frame_x = cones_in_frame_x
        self.cones_in_frame_y = cones_in_frame_y
        self.cones_in_frame_color = cones_in_frame_color

        self.cam_lidar_fusion_markers_pub = rospy.Publisher("/cam_lidar_fusion_cones",MarkerArray,queue_size=0)


    def projection_pipeline(self):
        flag = 0
        offset = 25
        height = 9.8 * 30
        width = 2.984 * 30
        img_W=30 * 30
        img_L=30 * 30
        source = [[209,300], [463,305], [364,209], [314,209]]
        destination = [[img_W/2-width/2,img_L - offset], [img_W/2+width/2,img_L - offset], [img_W/2+width/2,img_L-height - offset], [img_W/2-width/2,img_L-height - offset]]

        #matrix = cv2.getPerspectiveTransform(np.array(source,np.float32), np.array(destination,np.float32))

        
        
        matrix=np.array([[-2.29671205e-01 ,-2.41135046e+00  ,5.26975883e+02],
                            [-1.48936912e-02 ,-5.04315745e+00,  9.88282133e+02],
                            [-2.56345805e-05, -5.32605699e-03 , 1.00000000e+00]])
        
        x=[]
        y=[]
        
        xy = []

        

        if len(self.cones_in_frame_x) == 0  or len(self.points)==0 :
            return


        
        i = 0


        while i < len(self.cones_in_frame_x):

            if self.cones_in_frame_color[i] == "yellow_cone":
                r = 255
                g = 255 
                b = 0
            elif self.cones_in_frame_color[i] == "blue_cone":
                r = 0
                g = 0
                b = 255
            elif self.cones_in_frame_color[i] == "orange_cone":
                r = 255
                g = 128
                b = 0
            else:
                r = 0
                g = 0
                b = 0

            v = self.cones_in_frame_x[i]
            u = self.cones_in_frame_y[i]

            for f in range(5):
                self.cv_image[u,v] = (b,g,r)

            #print(r,g,b)

            i+=1

        output = cv2.warpPerspective(self.cv_image, matrix, (img_W, img_L),flags=cv2.INTER_LINEAR)


        outputY = output
        outputB = output
        outputO = output

        mask_yellow = cv2.inRange(outputY,(0,180,180),(0,255,255) )
        mask_rgb_yellow = cv2.cvtColor(mask_yellow,cv2.COLOR_GRAY2BGR)
        output_yellow = outputY & mask_rgb_yellow
        yellow_pix = np.where(output_yellow)

 

        mask_blue = cv2.inRange(outputB,(120,2,4),(180,8,12) )
        mask_rgb_blue = cv2.cvtColor(mask_blue,cv2.COLOR_GRAY2BGR)
        output_blue = outputB & mask_rgb_blue
        blue_pix = np.where(output_blue)
        
      


        mask_orange = cv2.inRange(outputO,(0,40,160),(20,80,255) )
        mask_rgb_orange = cv2.cvtColor(mask_orange,cv2.COLOR_GRAY2BGR)
        output_orange = outputO & mask_rgb_orange

        orange_pix = np.where(output_orange)



        

        # plt.imshow(mask_rgb_blue)
        # plt.scatter(yellow_pix[1],yellow_pix[0],color = "yellow")
        # plt.scatter(blue_pix[1],blue_pix[0],color = "blue")
        # plt.scatter(orange_pix[1],orange_pix[0],color = "orange")

        # plt.pause(0.01)



        resultU = []
        resultV = []
        resultX = []
        resultY = []
        resultColor = []
        

        
        
        for j in self.points:
            flagy = 0
            flagb = 0
            flago = 0
            px = int(img_L-offset-j[0]*30)
            py = int(img_W/2-j[1]*30)

            # print(len(yellow_pix))

            for pixU , pixV in zip(yellow_pix[0] , yellow_pix[1]):
                if abs((pixU-px) < 50) and abs((pixV-py) < 50) and flago == 0 and flagb == 0:

                    resultU.append(pixU)
                    resultV.append(pixV)
                    resultX.append(j[0]+1.4)
                    resultY.append(j[1]+0.00204)
                    resultColor.append("yellow_cone")
                    flagy = 1


            for pixU , pixV in zip(blue_pix[0] , blue_pix[1]):
                if abs((pixU-px) < 50) and abs((pixV-py) < 50) and flagy == 0 and flago == 0:
                    resultU.append(pixU)
                    resultV.append(pixV)
                    resultX.append(j[0]+1.4)
                    resultY.append(j[1]+0.00204)
                    resultColor.append("blue_cone")
                    flagb = 1
                
            
            for pixU , pixV in zip(orange_pix[0] , orange_pix[1]):
                if abs((pixU-px) < 50) and abs((pixV-py) < 50) and flagy == 0 and flagb == 0:

                    resultU.append(pixU)
                    resultV.append(pixV)
                    resultX.append(j[0]+1.4)
                    resultY.append(j[1]+0.00204)
                    resultColor.append("orange_cone")
                    flago = 1






        #print(len(resultX))
        
        #print(resultX,resultY)

        #plt.scatter(resultU,resultV)
        #plt.pause(0.01)

        self.cones_visuals(resultX,resultY,resultColor)

        resultU = []
        resultV = []
        resultX = []
        resultY = []
        resultColor = []
        

        return






    def cones_visuals(self,resultX,resultY,resultColor):
        c = 0

        self.rviz_msg = Marker()
        self.rviz_msg_array = MarkerArray()

        while c < len(resultX):
        
            
            x_cone = resultX[c]
            y_cone = resultY[c]
            c_cone = resultColor[c]

            if c_cone == "yellow_cone":
                r = 255
                g = 255 
                b = 0
            elif c_cone == "blue_cone":
                r = 0
                g = 0
                b = 255
            elif c_cone == "orange_cone":
                r = 255
                g = 0
                b = 0

            #print(x_cone,y_cone,c_cone)


            c +=1
            count = 0
            MARKERS_MAX = 100

            self.rviz_msg = Marker()
            self.rviz_msg.header.frame_id = "os1_sensor_base_link"
            self.rviz_msg.ADD
            self.rviz_msg.SPHERE
            self.rviz_msg.pose.position.x = x_cone
            self.rviz_msg.pose.position.y = y_cone
            self.rviz_msg.pose.position.z = 0
            self.rviz_msg.lifetime = rospy.Duration(0.5)
            self.rviz_msg.pose.orientation.w = 1
            self.rviz_msg.scale.x = 1
            self.rviz_msg.scale.y = 1
            self.rviz_msg.scale.z = 1
            self.rviz_msg.color.a = 1
            self.rviz_msg.color.r = r
            self.rviz_msg.color.g = g
            self.rviz_msg.color.b = b
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
        
        self.cam_lidar_fusion_markers_pub.publish(self.rviz_msg_array)

        return







if __name__ == '__main__':
    try:
        pass
        projection_birdeye_view()
    except rospy.ROSInterruptException:
        pass
    rospy.spin()
