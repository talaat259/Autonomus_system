#!/usr/bin/env python

# import time 
import numpy
import math
import rospy
# import copy
# from sensor_msgs import msg
# import tf
# from aam_common_msgs.msg import Cone
from aam_common_msgs.msg import ConeDetections
# from sensor_msgs.msg import PointCloud2
# from sensor_msgs import point_cloud2
from geometry_msgs.msg import Point
# from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
# from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
# import random
from tf.transformations import euler_from_quaternion
# import sys

########################################################
#Getting vehicle orientation from the odometry messages
#########################################################
# def odometryCallback(self, odom_msg):
#   # rospy.loginfo("odometryCallback")
#   orientation_q = odom_msg.pose.pose.orientation
#   orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
#   self.car_orientation_w = odom_msg.pose.pose.orientation.w
#   (roll, pitch, yaw)  = euler_from_quaternion(orientation_list)
  
#   '''
#   self.carPosX = odom_msg.pose.pose.position.x
#   self.carPosY = odom_msg.pose.pose.position.y
#   self.carPosYaw = yaw
#   '''
#   #print(self.carPosX,self.carPosY,self.carPosYaw)

class acceleration_planner:
  def __init__(self): #,namespace='rrt'
    self.c = 0
    self.flag=True
    self.firstPos = 0 
    self.points = []
    rospy.init_node("acceleration_planner", anonymous = True)
    savedPoints = rospy.Subscriber("/lidar_cone_detection_cones",ConeDetections,self.cones_callback)
    self.rviz_pub_array = rospy.Publisher("/cones_colored",MarkerArray,queue_size = 0)
    self.waypointsVisualPub = rospy.Publisher("/visual/waypoints", MarkerArray, queue_size = 1) 
  def addCone(self,x,y):
    
    center_angle = math.radians(0)
    theta = math.atan2(y,x)
    if center_angle < theta: # and theta > final
      self.left_conesX.append(x)
      self.left_conesY.append(y)
      # self.li+=1
    
    if center_angle > theta : #and theta < final
      self.right_conesX.append(x)
      self.right_conesY.append(y)
      # self.ri+=1

  def getMinimumConeX(self):
    if min(self.left_conesX) < min(self.right_conesX):
      return min(self.left_conesX)
    else:
      return min(self.right_conesX)

  def cones_callback(self,cones_msg):
    # print(cones_msg)
    cones_x = []
    cones_y  = []
    
    global map_x
    global map_y
    
    self.right_conesX = []
    self.right_conesY = []
    self.left_conesX = []
    self.left_conesY = []
    
    # final = math.radians(1)
    # negFinal = math.radians(-90)
    map_x = cones_x
    map_y = cones_y
    # self.firstPos=0
    # self.ri = 0
    # self.li = 0
    for cone in cones_msg.cone_detections:
      x = cone.position.x
      y = cone.position.y
      self.addCone(x,y)

      # if li < 4:
      #   if y < 0: # and theta > final
      #    left_conesX.append(x)
      #    left_conesY.append(y)
      #    li+=1
      # if ri < 4:
      #   if y > 0 : #and theta < final
      #     right_conesX.append(x)
      #     right_conesY.append(y)
      #     ri+=1
      # distance = math.sqrt(x**2 +  y**2)

    self.firstPos= self.getMinimumConeX()

    # if self.c == 0: 
    #   self.firstPos= self.getMinimumConeX()
    #   self.c+=1

    self.cones_visuals(self.right_conesX, self.right_conesY, self.left_conesX, self.left_conesY)
    self.acceleration_path_algorithm(self.right_conesX, self.right_conesY, self.left_conesX, self.left_conesY)



  def acceleration_path_algorithm(self,right_conesX ,right_conesY,left_conesX ,left_conesY):
        if(self.flag):
          # leftCones = self.leftCones
          # rightCones = self.rightCones
      

          # Process(target= self.loop_LeftCones(leftCones)).start()
          # Process(target= self.loop_RightCones(rightCones)).start()
          
          # print("First left Cone :")
          # print(leftCones[0][0])
          # print(leftCones[1][0])
          # print("2nd left Cone :")
          # print(leftCones[0][1])
          # print(leftCones[1][1])
          
          half = numpy.poly1d(0.5,False)

          leftPolyfit = numpy.poly1d(numpy.polyfit(left_conesX,left_conesY, 1)) #(x array ,y array, 1st degree poly) of left cones
          # print("leftpolyfit")
          # print(leftPolyfit)

          rightPolyfit = numpy.poly1d(numpy.polyfit(right_conesX, right_conesY, 1))
          # print("rightpolyfit")
          # print(rightPolyfit)

          averagepolyfitPath = numpy.polymul(numpy.polyadd(leftPolyfit,rightPolyfit),half)
          print("MIDDLEPOLYFIT!")     
            
          # print(averagepolyfitPath)
          # if min(left_conesX)<min(right_conesX):
          #   xcoeff = numpy.linspace(min(left_conesX),max(left_conesX), 150)
            
          # else:
          #   xcoeff = numpy.linspace(min(right_conesX),max(right_conesX), 150)
          (averagepolyfitPath, xcoeff)
          self.points = numpy.vs
          xcoeff = numpy.linspace(self.firstPos, 75 + self.firstPos, 500)
          # xcoeff = numpy.linspace(0, 75, 500)
          ycoeff = numpy.polyvaltack((xcoeff,ycoeff)).T
          # print(points)
      
          # left_conesX = []
          # left_conesY = []
          # right_conesX = []
          # left_conesY = []
          # self.waypointsPlot(xcoeff,ycoeff,points)
          flag = False
        self.publishWaypointsVisuals(self.points)
    


  def cones_visuals(self,xr,yr,xl,yl): #,xl,yl,xr,yr
   

    self.rviz_msg = Marker()
    self.rviz_msg_array = MarkerArray()
    
    c = 0
    while c < len(xr):
      
      
      x_cone = xr[c]
      y_cone = yr[c]


      c +=1
      count = 0
      MARKERS_MAX = 100

      self.rviz_msg = Marker()
      self.rviz_msg.header.frame_id = "map"
      self.rviz_msg.ADD
      self.rviz_msg.SPHERE
      self.rviz_msg.pose.position.x = x_cone
      self.rviz_msg.pose.position.y = y_cone
      self.rviz_msg.pose.position.z = 0

      self.rviz_msg.pose.orientation.w = 1
      self.rviz_msg.scale.x = 1
      self.rviz_msg.scale.y = 1
      self.rviz_msg.scale.z = 1
      self.rviz_msg.color.a = 1
      self.rviz_msg.color.r = 255
      self.rviz_msg.color.g = 255
      
      self.rviz_msg.color.b = 0
      self.rviz_msg.mesh_resource = "package://aamfsd_description/meshes/cone_blue.dae"
      self.rviz_msg.type = Marker.MESH_RESOURCE
      self.rviz_msg.mesh_use_embedded_materials = True
      self.rviz_msg.lifetime = rospy.Duration(0.1)


      if(count > MARKERS_MAX):
        self.rviz_msg_array.markers.pop(0)
      
      self.rviz_msg_array.markers.append(self.rviz_msg)
      m = 0
      id = 0
      for m in self.rviz_msg_array.markers:
        m.id = id
        id += 1


    c = 0
    while c < len(xl):
      
      
      x_cone = xl[c]
      y_cone = yl[c]


      c +=1
      count = 0
      MARKERS_MAX = 100

      self.rviz_msg = Marker()
      self.rviz_msg.header.frame_id = "map"
      self.rviz_msg.ADD
      self.rviz_msg.SPHERE
      self.rviz_msg.pose.position.x = x_cone
      self.rviz_msg.pose.position.y = y_cone
      self.rviz_msg.pose.position.z = 0

      self.rviz_msg.pose.orientation.w = 1
      self.rviz_msg.scale.x = 1
      self.rviz_msg.scale.y = 1
      self.rviz_msg.scale.z = 1
      self.rviz_msg.color.a = 1
      self.rviz_msg.color.r = 0
      self.rviz_msg.color.g = 0
      self.rviz_msg.color.b = 255
      self.rviz_msg.mesh_resource = "package://aamfsd_description/meshes/cone_blue.dae"
      self.rviz_msg.type = Marker.MESH_RESOURCE
      self.rviz_msg.mesh_use_embedded_materials = True
      self.rviz_msg.lifetime = rospy.Duration(0.1)


      if(count > MARKERS_MAX):
        self.rviz_msg_array.markers.pop(0)
      
      self.rviz_msg_array.markers.append(self.rviz_msg)
      m = 0
      id = 0
      for m in self.rviz_msg_array.markers:
        m.id = id
        id += 1
    
    self.rviz_pub_array.publish(self.rviz_msg_array)

  def publishWaypointsVisuals(self, points = None):
      markerArray = MarkerArray()
      savedWaypointsMarker = Marker()
      savedWaypointsMarker.header.frame_id = "map"
      savedWaypointsMarker.header.stamp = rospy.Time.now()
      savedWaypointsMarker.ns = "saved-publishWaypointsVisuals" #namespace
      savedWaypointsMarker.id = 1
      savedWaypointsMarker.type = savedWaypointsMarker.SPHERE_LIST
      savedWaypointsMarker.action = savedWaypointsMarker.ADD
      savedWaypointsMarker.scale.x = 0.3
      savedWaypointsMarker.scale.y = 0.3
      savedWaypointsMarker.scale.z = 0.3
      savedWaypointsMarker.pose.orientation.w = 1
      savedWaypointsMarker.color.a = 1.0
      savedWaypointsMarker.color.r = 0.866
      savedWaypointsMarker.color.g = 0.152
      savedWaypointsMarker.color.b = 0.670
      savedWaypointsMarker.lifetime = rospy.Duration(0.1)

    
     
      for point in points:
          p = Point(point[0], point[1], 0.0)      #(x,y,z)
          savedWaypointsMarker.points.append(p)
      markerArray.markers.append(savedWaypointsMarker)
      self.waypointsVisualPub.publish(markerArray)
      # savedWaypointsMarker.points = []

  # def waypointsPlot(self,xcoeff,ycoeff,points):
  #   plotF = open("acceleration_waypoints_plot.csv",'w')
  #   writer = csv.writer(plotF)
  #   writer.writerow(['x','y'])
  #   for w in range(len(xcoeff)):
  #     writer.writerow([xcoeff[w],ycoeff[w]])
  #   plotF.close()

    # with open('acceleration_waypoints_plot.csv','w') as plotF: #~/111/aamfsd_pp_2022/src/aam_path_planning/acceration_planner/src/
      # writer = csv.writer(plotF)
      # writer.writerow(['x','y'])
      # for w in range(len(xcoeff)):
        # writer.writerow([xcoeff[w],ycoeff[w]])
    

  
if __name__ == '__main__':
  try:
      acceleration_path = acceleration_planner()

  except rospy.ROSInterruptException:
      pass
  rospy.spin()

