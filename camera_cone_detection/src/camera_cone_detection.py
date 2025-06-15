#!/usr/bin/env python



import numpy as np
import math
import rospy
from sensor_msgs import msg
import tf
from aam_common_msgs.msg import Cone
from aam_common_msgs.msg import Map
from aam_common_msgs.msg import ConeDetections
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2
from darknet_ros_msgs.msg import BoundingBoxes
from darknet_ros_msgs.msg import BoundingBox
from geometry_msgs.msg import Point
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from sensor_msgs.msg import LaserScan
import sys
import ros_numpy
import struct
import time 
import pcl_ros
from array import *
import math
import matplotlib.pyplot as plt 
from nav_msgs.msg import Path
from stereo_msgs.msg import DisparityImage
from image_geometry import StereoCameraModel



def box_pipeline (box_msg):
  for box  in box_msg.bounding_boxes:
    x_min = box.xmin
    y_min = box.ymin
    x_max = box.xmax
    y_max = box.ymax             
    cone_color = box.Class
    x_center = ((x_max-x_min)/2)+x_min
    y_center = ((y_max-y_min)/2)+y_min

    #print(x_center,y_center,cone_color)

    cones_in_frame_x.append(x_center)
    cones_in_frame_y.append(y_center)
    cones_in_frame_color.append(cone_color)

    #print(cones_in_frame_x," - ",cones_in_frame_y," - ",cones_in_frame_color)


  #print(cones_in_frame_x,cones_in_frame_y,cones_in_frame_color)

  #plt.scatter(cones_in_frame_x,cones_in_frame_y)
  #plt.show()

    

def pc_pipeline(pc_msg):
  xyz_array = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(pc_msg,remove_nans=True)

  x_cloud_array = []
  y_cloud_array = []
  z_cloud_array = []

  
  p = np.array([[235.27027794031173, 0.0, 336.5, -28.232433352837408],[0.0, 235.27027794031173, 188.5, 0.0],[0.0, 0.0, 1.0, 0.0]])

  for point in xyz_array:
    x_cloud = point[0]
    y_cloud = point[1]
    z_cloud = point[2]

    x_cloud_array.append(x_cloud)
    y_cloud_array.append(y_cloud)
    z_cloud_array.append(z_cloud)
  
  i = 0 

  x_projected = []
  y_projected = []
  color_projected = []
  
  f = 0

  #print(self.cones_in_frame_color)

  yellow_cones_x = []
  yellow_cones_y = []
  blue_cones_x = []
  blue_cones_y = []
  orange_cones_x = []
  orange_cones_y = []


  cone_detection_msg = ConeDetections()



  while i < len(x_cloud_array):
    x = x_cloud_array[i]
    y = y_cloud_array[i]
    z = z_cloud_array[i]
    
    
    if z < 1.6 and len(cones_in_frame_y)>0:
      
      xyz_array_world = np.array([x,y,z,1])
      xyz_array_frame = np.dot(p,xyz_array_world)

      px = xyz_array_frame[0]/xyz_array_frame[2]
      py = xyz_array_frame[1]/xyz_array_frame[2]
      
      
      print(xyz_array_frame)

      j=0

      while j <len(cones_in_frame_x):
        
        cone_msg = Cone()
        
        if abs(cones_in_frame_x[j] - px)<90  and abs(cones_in_frame_y[j] -py)<90 :
          x_projected.append(px)
          y_projected.append(py)
          color_projected.append(cones_in_frame_color[j])
          
          

          if cones_in_frame_color[j] == 'blue_cone':
            #print(j,"b")
            blue_cones_x.append(z+2)
            blue_cones_y.append(y+2)
            cone_msg.position.x = z
            cone_msg.position.y = y
            cone_msg.color.data = "blue_cone"
            cone_detection_msg.cone_detections.append(cone_msg)

          elif cones_in_frame_color[j] == 'yellow_cone':
            #print(j,"y")
            yellow_cones_x.append(z+2)
            yellow_cones_y.append(y-4)
            cone_msg.position.x = z
            cone_msg.position.y = y
            cone_msg.color.data = "yellow_cone"
            cone_detection_msg.cone_detections.append(cone_msg)

          elif cones_in_frame_color[j] == 'orange_cone':
            #print(j,"o")
            orange_cones_x.append(z)
            orange_cones_y.append(y)
            cone_msg.position.x = z 
            cone_msg.position.y = y 
            cone_msg.color.data = "orange_cone"
            cone_detection_msg.cone_detections.append(cone_msg)


        j+=1

    i+=1
  

  #print(yellow_cones_x,yellow_cones_y)
  #print(blue_cones_x,blue_cones_y)
  #print(orange_cones_x,orange_cones_y)  

  plt.scatter(yellow_cones_x,yellow_cones_y,c='yellow')
  plt.scatter(blue_cones_x,blue_cones_y,c='blue')
  plt.scatter(orange_cones_x,orange_cones_y,c='orange')

  #plt.show()

  #print(cone_detection_msg)


  y = 0 
  b = 0
  o = 0

  rviz_msg = Marker()
  rviz_msg_array = MarkerArray()

  while y < len(yellow_cones_x):
      
    
    x_yellow = yellow_cones_x[y]
    y_yellow = yellow_cones_y[y]
    y +=1
    count = 0
    MARKERS_MAX = 10

    rviz_msg = Marker()
    rviz_msg.header.frame_id = "odom"
    rviz_msg.ADD
    rviz_msg.SPHERE
    rviz_msg.pose.position.x = x_yellow 
    rviz_msg.pose.position.y = y_yellow
    rviz_msg.pose.position.z = 0
    rviz_msg.pose.orientation.x = 0
    rviz_msg.pose.orientation.y = 0
    rviz_msg.pose.orientation.z = 0
    rviz_msg.pose.orientation.w = 0
    rviz_msg.scale.x = 1
    rviz_msg.scale.y = 1
    rviz_msg.scale.z = 1
    rviz_msg.color.a = 1
    rviz_msg.color.r = 0
    rviz_msg.color.g = 0
    rviz_msg.color.b = 0
    rviz_msg.mesh_resource = "package://aamfsd_description/meshes/cone_yellow.dae"
    rviz_msg.type = Marker.MESH_RESOURCE
    rviz_msg.mesh_use_embedded_materials = True

    
    rviz_msg_array.markers.append(rviz_msg)
    m = 0
    id = 0
    for m in rviz_msg_array.markers:
      m.id = id
      id += 1
    f += 1


  while b < len(blue_cones_x):
      
      
    x_blue = blue_cones_x[b]
    y_blue = blue_cones_y[b]
    b +=1
    count = 0
    MARKERS_MAX = 10

    rviz_msg = Marker()
    rviz_msg.header.frame_id = "odom"
    rviz_msg.ADD
    rviz_msg.SPHERE
    rviz_msg.pose.position.x = x_blue 
    rviz_msg.pose.position.y = y_blue
    rviz_msg.pose.position.z = 0
    rviz_msg.pose.orientation.x = 0
    rviz_msg.pose.orientation.y = 0
    rviz_msg.pose.orientation.z = 0
    rviz_msg.pose.orientation.w = 0
    rviz_msg.scale.x = 1
    rviz_msg.scale.y = 1
    rviz_msg.scale.z = 1
    rviz_msg.color.a = 1
    rviz_msg.color.r = 0
    rviz_msg.color.g = 0
    rviz_msg.color.b = 1
    rviz_msg.mesh_resource = "package://aamfsd_description/meshes/cone_blue.dae"
    rviz_msg.type = Marker.MESH_RESOURCE
    rviz_msg.mesh_use_embedded_materials = True




    rviz_msg_array.markers.append(rviz_msg)
    m = 0
    id = 0
    for m in rviz_msg_array.markers:
      m.id = id
      id += 1
    f += 1

  while o < len(orange_cones_y):
      
    
    x_orange = orange_cones_x[o]
    y_orange = orange_cones_y[o]
    y +=1
    count = 0
    MARKERS_MAX = 10

    rviz_msg = Marker()
    rviz_msg.header.frame_id = "odom"
    rviz_msg.ADD
    rviz_msg.SPHERE
    rviz_msg.pose.position.x = x_orange
    rviz_msg.pose.position.y = y_orange
    rviz_msg.pose.position.z = 0
    rviz_msg.pose.orientation.x = 0
    rviz_msg.pose.orientation.y = 0
    rviz_msg.pose.orientation.z = 0
    rviz_msg.pose.orientation.w = 0
    rviz_msg.scale.x = 1
    rviz_msg.scale.y = 1
    rviz_msg.scale.z = 1
    rviz_msg.color.a = 1
    rviz_msg.color.r = 0
    rviz_msg.color.g = 0
    rviz_msg.color.b = 0
    rviz_msg.mesh_resource = "package://aamfsd_description/meshes/cone_big.dae"
    rviz_msg.type = Marker.MESH_RESOURCE
    rviz_msg.mesh_use_embedded_materials = True

    
    rviz_msg_array.markers.append(rviz_msg)
    m = 0
    id = 0
    for m in rviz_msg_array.markers:
      m.id = id
      id += 1
    f += 1
    o+=1

    if(count > MARKERS_MAX):
      rviz_msg_array.markers.pop(0)

  
  rviz_pub_array.publish(rviz_msg_array) 
  #print(rviz_msg_array)

  #plt.scatter(yellow_cones_x,yellow_cones_y,c="yellow")
  #plt.scatter(blue_cones_x,blue_cones_y,c="blue")
  #plt.scatter(orange_cones_x,orange_cones_y,c="orange")

  #plt.show()
  
  #print(yellow_cones_x,yellow_cones_y)
  #print(blue_cones_x,blue_cones_y)
  #print(orange_cones_x,orange_cones_y)

  camera_cone_detection_pub.publish(cone_detection_msg)


  yellow_x_min , yellow_x_max = min(yellow_cones_x) , max(yellow_cones_x)
  new_yellow_x = np.linspace(yellow_x_min,yellow_x_max,100)
  yellow_coefficients = np.polyfit(yellow_cones_x,yellow_cones_y,3)
  new_yellow_y = np.polyval(yellow_coefficients,new_yellow_x)

  blue_x_min , blue_x_max = min(blue_cones_x) , max(blue_cones_x)
  new_blue_x = np.linspace(blue_x_min,blue_x_max,100)
  blue_coefficients = np.polyfit(blue_cones_x,blue_cones_y,3)
  new_blue_y = np.polyval(blue_coefficients,new_blue_x)

  plt.plot(new_yellow_x,new_yellow_y,c='yellow')
  plt.plot(new_blue_x,new_blue_y,c='blue')
  #plt.show()

  midx = []
  midy = []
  for i in range(100):
    midx.append(np.mean([new_yellow_x[i],new_blue_x[i]]))
    midy.append(np.mean([new_yellow_y[i],new_blue_y[i]]))



  plt.plot(midx, midy,'--', c='black')
  #plt.show()

  mid_path = Path()
  i = 0

  mid_path.header.frame_id = "base_link"
  mid_path.header.stamp = rospy.Time.now()

  while i < len(midx):  
    
    pose_msg = PoseStamped() 

    pose_msg.pose.position.x = midx[i]
    pose_msg.pose.position.y = midy[i]
    pose_msg.pose.position.z = 0
    pose_msg.pose.orientation.x = 0
    pose_msg.pose.orientation.y = 0
    pose_msg.pose.orientation.z = 0
    pose_msg.pose.orientation.w = 0

    mid_path.poses.append(pose_msg)
  
    #print(i)
    #print(pose_msg)
    #time.sleep(1)
    #print(self.mid_path)

    i+=1
  
  #print(mid_path)

  mid_path_pub.publish(mid_path)

  


















def start():
  rospy.init_node('camera_cone_detection')
  
  global rviz_pub_array
  global mid_path_pub
  global camera_cone_detection_pub

  rospy.Subscriber("/darknet_ros/bounding_boxes", BoundingBoxes, box_pipeline)
  rospy.Subscriber("/zed/points2", PointCloud2, pc_pipeline)
  rviz_pub_array = rospy.Publisher("/camera_cones_rviz",MarkerArray, queue_size = 1)
  camera_cone_detection_pub = rospy.Publisher("/camera_cone_detection_cones",ConeDetections,queue_size=1)
  mid_path_pub = rospy.Publisher("/mid_path",Path,queue_size=1)
  




  rate = rospy.Rate(10)     # 10hz
  
  while not rospy.is_shutdown():
    
    global cones_in_frame_x
    cones_in_frame_x = []
    global cones_in_frame_y
    cones_in_frame_y = []
    global cones_in_frame_color
    cones_in_frame_color = []

    global K00
    global K02
    global K11
    global K12
    global K22

    K00 = 235.27027794031173
    K02 = 336.5
    K11 = 235.27027794031173
    K12 = 188.5
    K22 = 1.000000


    rate.sleep()
    






if __name__ == '__main__':
  try:
    start()
  except rospy.ROSInterruptException:
    pass
  