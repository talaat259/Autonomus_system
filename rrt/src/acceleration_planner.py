#!/usr/bin/env python

import numpy
import math
import rospy
from aam_common_msgs.msg import Cone
from aam_common_msgs.msg import ConeDetections
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from nav_msgs.msg import Odometry , Path
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import PoseStamped , Quaternion , Pose


class acceleration_planner:
  def __init__(self): #,namespace='acceleration'
    self.c = 0
    self.flag=True
    self.carPosX= 0
    self.carPosY = 0 
    self.li=0
    self.ri=0
    self.points = []
    rospy.init_node("acceleration_planner", anonymous = True)
    rospy.Subscriber("/car_pose", Path, self.getCarPos)
    #rospy.Subscriber("/cam_lidar_fusion_cones",MarkerArray,self.cones_callback)
    rospy.Subscriber("/centroid_lidar_cones_marker",MarkerArray,self.cones_callback)
    
    self.rviz_pub_array = rospy.Publisher("/cones_colored",MarkerArray,queue_size = 0)
    self.waypointsVisualPub = rospy.Publisher("/visual/waypoints", MarkerArray, queue_size = 1) 

  def getCarPos(self,path_msg):
    self.carPosX = path_msg.poses[-1].pose.position.x
    self.carPosY = path_msg.poses[-1].pose.position.y

  # def addCone(self,cone,x,y):
  #   r=cone.color.r
  #   g=cone.color.g
  #   b=cone.color.b
  #   # color=""
  #   if r == 255 and g == 255 and b == 0 :
  #     # color="yellow_cone"
  #     self.right_conesX.append(x)
  #     self.right_conesY.append(y)
  #   elif r == 0 and g == 0 and b == 255 :
  #     # color="blue_cone"
  #     self.left_conesX.append(x)
  #     self.left_conesY.append(y)
  #   elif r == 255 and g == 0 and b == 0 :
  #     # color="orange_cone"
  #     pass
  def addCone(self,x,y):
    
    center_angle = math.radians(0)
    theta = math.atan2(y,x)

    maxCones=4
    if center_angle < theta : # and theta > final
      self.left_conesX.append(x)
      self.left_conesY.append(y)
     
      
    elif center_angle > theta : #and theta < final
      self.right_conesX.append(x)
      self.right_conesY.append(y)
      
              
  def dist(self, x1, y1, x2, y2):       
    distSq = (x1 - x2) ** 2 + (y1 - y2) ** 2
    return math.sqrt(distSq) 


  def cones_callback(self,cones_msg):
    #print(cones_msg)
    cones_x = []
    cones_y  = []
    
    global map_x
    global map_y
    
    self.right_conesX = []
    self.right_conesY = []
    self.left_conesX = []
    self.left_conesY = []
    
    map_x = cones_x
    map_y = cones_y
    i=0
    for cone in cones_msg.markers:
      x = cone.pose.position.x
      y = cone.pose.position.y
      i+=1
      #print(i)
      #self.addCone(cone,x,y)
      self.addCone(x,y)
  
    self.cones_visuals(self.right_conesX, self.right_conesY, self.left_conesX, self.left_conesY)
    self.acceleration_path_algorithm(self.right_conesX, self.right_conesY, self.left_conesX, self.left_conesY)



  def acceleration_path_algorithm(self,right_conesX ,right_conesY,left_conesX ,left_conesY):
    if(self.flag):
	    try:
	      if(len(right_conesX)>=2 and len(left_conesX)>=2):
		print("inif")
		half = numpy.poly1d(0.5,False)
		leftPolyfit = numpy.poly1d(numpy.polyfit(left_conesX,left_conesY, 1))
		rightPolyfit = numpy.poly1d(numpy.polyfit(right_conesX, right_conesY, 1))
		averagepolyfitPath = numpy.polymul(numpy.polyadd(leftPolyfit,rightPolyfit),half)
		xcoeff = numpy.linspace(0, 75, 500)
		ycoeff = numpy.polyval(averagepolyfitPath, xcoeff)
	      	self.points = numpy.vstack((xcoeff,ycoeff)).T
		self.flag=False
	      	while(True):
		 self.publishWaypointsVisuals(self.points)
		
	    except:
	      pass
    
  

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
      savedWaypointsMarker.lifetime = rospy.Duration()

    
     
      for point in points:
          p = Point(point[0], point[1], 0.0)      #(x,y,z)
          savedWaypointsMarker.points.append(p)
      markerArray.markers.append(savedWaypointsMarker)
      self.waypointsVisualPub.publish(markerArray)

  
if __name__ == '__main__':
  try:
      acceleration_path = acceleration_planner()

  except rospy.ROSInterruptException:
      pass
  rospy.spin()

