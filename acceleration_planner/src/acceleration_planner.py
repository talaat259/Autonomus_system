#!/usr/bin/env python

import numpy
import math
import rospy
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray



class acceleration_planner:
  def __init__(self): #,namespace='acceleration'
   
    self.flag=True
    self.points = []
    rospy.init_node("acceleration_planner", anonymous = True)
    rospy.Subscriber("/centroid_lidar_cones_marker",MarkerArray,self.cones_callback)
    
    self.rviz_pub_array = rospy.Publisher("/cones_colored",MarkerArray,queue_size = 0)
    self.waypointsVisualPub = rospy.Publisher("/visual/waypoints", MarkerArray, queue_size = 0) 



  def cones_callback(self,cones_msg):
    self.acceleration_path_algorithm(self)



  def acceleration_path_algorithm(self,cones):
    if(self.flag):
      try:
          xcoeff = numpy.linspace(0, 75, 150)
          ycoeff = []
          for x in xcoeff:
            ycoeff.append(0.0)
          self.points = numpy.vstack((xcoeff,ycoeff)).T
          self.flag=False
          
          while(True):
            self.publishWaypointsVisuals(self.points)
            
          
            
      except:
        pass

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

