#!/usr/bin/env python
from operator import delitem, ne
from threading import local
import time ,math
import math
import rospy
from sensor_msgs import msg
from tf.transformations import euler_from_quaternion
from aam_common_msgs.msg import Cone
from aam_common_msgs.msg import ConeDetections
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2
from geometry_msgs.msg import Point
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import csv 

class planner:
      def __init__(self):
            rospy.init_node("skidpadplanner", anonymous = True)
            rospy.Subscriber("robot_control/odom",Odometry,self.odometryCallback)
            rospy.Subscriber("/centroid_lidar_cones_marker",MarkerArray,self.cones_callback)
            # rospy.Subscriber("/lidar_cone_detection_cones",ConeDetections,self.cones_pipeline)

            self.shouldPublishWaypoints = rospy.get_param('~publishWaypoints', True)
            # Create publishers
            self.waypointsVisualPub = rospy.Publisher("/visual/waypoints", MarkerArray, queue_size=0)
            ##self.waypointsVisualPub = rospy.Publisher("/visual/waypoints", MarkerArray, queue_size=1)

            self.map_right=[] #Point of reference ==> CAR
            self.map_left=[]
            self.maps_right=[] #Point of reference ==> First Read Point
            self.maps_left=[]
            self.initialset=True
            self.z=0
            self.distance_coordinate=[]
            self.txt = 0
            self.circle_right_y = 0
            self.circle_right_x = 0
            self.circle_left_y = 0
            self.circle_left_x = 0
            self.ent_var = 0
            self.firstwarningdetect = 0
            self.flag2 = 0
            self.pont_exit_x=0
            self.pont_exit_y=0
            self.waypoints = []
            self.coordinate_x=0
            self.coordinate_y=0
            self.carpos_x = 0.0
            self.carpos_y = 0.0
            self.yaw= 0.0
            self.counterpoint_x=0
            self.counterpoint_y=0
            self.point_first_15=()
            circle_x_mid_coordinates=()



      def distance(self, x1, y1, x2, y2, shouldSqrt = True):
            distSq = (x1 - x2) ** 2 + (y1 - y2) ** 2
            return math.sqrt(distSq) ##if shouldSqrt else distSq




      def hor_dist(self,point1,point2):#on y mabeen ay 2 points
            xx= point1-point2
            if(xx>0):
                  return xx
            else:
                  return xx*-1

      def odometryCallback(self,odom_msg):#gets the current  coordinates and saves them to a tuple and then appends to movedpoints
            
            self.carpos_x = odom_msg.pose.pose.position.x
            self.carpos_y = odom_msg.pose.pose.position.y
            self.yaw= odom_msg.pose.pose.orientation.w

            orientation_q = odom_msg.pose.pose.orientation
            orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
            (roll, pitch, yaw)  = euler_from_quaternion(orientation_list)

      def cones_callback(self,cones_msg):
            print("zebyyyyy")
            cones_c = []
            for cone in cones_msg.markers:
                  if(self.hor_dist(0,cone.pose.position.y) <= 4):
                        cones_c.append((cone.pose.position.x,cone.pose.position.y))
                  else:
                        continue 
            for cone in cones_c:
                  if((cone[0]>self.coordinate_x)and(cone[1]>=0)):
                        self.maps_right.append((cone[0],cone[1]))
                  elif ((cone[1]<self.coordinate_y)and(cone[0]>self.coordinate_x)):
                        self.maps_left.append((cone[0],cone[1]))
                  else:
                        continue
           
            self.maps_left.sort(key=lambda x:x[0])
            self.maps_right.sort(key=lambda x:x[0])
            refrence_left=self.maps_left[0]
            refrence_right=self.maps_right[0]
            self.point_first_15=self.midpointer(refrence_left[0],refrence_left[1],refrence_right[0],refrence_right[1])
            self.circle_x_mid_coordinates=(self.point_first_15[0]+13,self.point_first_15[1])#tw7eed al x axis




            # pr=self.maps_left[1] ############################################################# CHECK ME PLZZ :**##
            # pl=self.maps_left[1]
            # z=self.midpointer(pr[0],pr[1],pl[0],pl[1])
            # self.coordinate_x=z[0]
            # self.coordinate_y=z[1]  

            self.right_cones(cones_c)
                  #print(i)
                  #self.addCone(cone,x,y)
                  # self.addCone(x,y)
  
            # self.cones_visuals(self.right_conesX, self.right_conesY, self.left_conesX, self.left_conesY)
            # self.acceleration_path_algorithm(self.right_conesX, self.right_conesY, self.left_conesX, self.left_conesY)

      
            

      def midpointer(self,x1,y1,x2,y2):
            return ((x1 + x2) / 2, (y1 + y2) / 2)              

      def right_cones(self,cones_list) :
            for cone  in cones_list:
                  if((cone[0]>self.coordinate_x)and(cone[1]>=0)): #@@@@@@@@@@@@@@@@@@@@@
                        self.map_right.append((cone[0],cone[1]))
                  elif ((cone[1]<self.coordinate_y)and(cone[0]>self.coordinate_x)):
                        self.map_left.append((cone[0],cone[1]))
                  else:
                        continue
            self.map_right.sort(key=lambda x:x[0] )
            self.map_left.sort(key=lambda x:x[0])#sorts according to the x coordinate
            try:
                  if(self.ent_var == 0):
                        self.entry_90_track(self.map_right,self.map_left)
                        self.ent_var = self.ent_var + 1
            except:
                  pass   


      def entry_90_track(self,map_right,map_left) :
            c=0
            x=zip(map_right,map_left) #Parallel Iteration on two list .. k2no by7wlhom 2D array ####################
            newlist_y = list(x)
            iR = 0
            #     for pt1,pt2 in newlist_y:#gets the cones as tupels
            #           if(iR != 1):
            #             z = self.hor_dist(pt1[1],pt2[1])
            #             midpoint = self.midpointer(pt1[0],pt1[1],pt2[0],pt2[1])
            #             self.distance_coordinate.append(z)
            #             iR = iR + 1
            #           else:
            #                 break
            #     newlist_y.pop(0)
          
            for pt1,pt2 in newlist_y:#gets the cones as tupels
                  z = self.hor_dist(pt1[1],pt2[1])
                  midpoint = self.midpointer(pt1[0],pt1[1],pt2[0],pt2[1])
                  self.distance_coordinate.append(z)
                  if(self.initialset):
                        if(self.distance_coordinate[-1]>=self.distance_coordinate[-2] and self.txt == 0): #or (self.firstwarningdetect == 0)):
                              ##new = self.initial_vector[0] + 1
                              ##print(new)
                              print("if 90 track")
                              newElnew = [midpoint]
                              #self.publishWaypointsVisuals(newElnew)
                              self.waypoints.append(midpoint)

                              ##################### 0.75 #######################
                        elif(((self.firstwarningdetect > 0) and ((self.distance_coordinate[-1] - self.distance_coordinate[-2])) * -1 > 0.75 )):#180
                              self.txt = 1
                              self.initialset=False
                              ##print("shift_enter")
                              degree=0
                              shift_enter=[]
                              pointmid=self.midpointer(pt1[0],pt1[1],pt2[0],pt2[1]) 
                              self.pont_exit_x=pointmid[0]########################
                              self.pont_exit_y=pointmid[1]########################
                              ##################get the midpoint and update..  
                              self.circle_right_y = self.circle_x_mid_coordinates[1] + (1.5 + (15.25/2))
                              self.circle_right_x = self.circle_x_mid_coordinates[0] 
                              self.circle_left_x = self.circle_x_mid_coordinates[0]  
                              self.circle_left_y = self.circle_x_mid_coordinates[1] - (1.5+(15.25/2))
                              print("pointmid",pointmid,"circle right y", self.circle_right_y, "circle right x", self.circle_right_x , "circle left y", self.circle_left_y, "circle left x", self.circle_left_x) 
                              # circle_center_right_x=pointmid[0]
                              # self.trueTurn_x = pointmid[0]
                              # circle_center_right_y=pointmid[1]+1.5+(15.25/2)
                              # self.trueTurn_y = pointmid[1]
                              while(degree < 90):
                                    next_point_x = self.circle_left_x + (((15.25/2)+1.5)*(math.sin(math.radians(degree))))#point every 2 degrees"0.31m"
                                    next_point_y = self.circle_left_y+(((15.25/2)+1.5)*(math.cos(math.radians(degree))))

                                    self.waypoints.append((next_point_x,next_point_y))

                                    ##print(next_point_x)
                                    shift_enter.append((next_point_x,next_point_y))
                                    degree=degree+1
                                    ##print(shift_enter)
                              print("ana shift a7a")
                              ##self.publishWaypointsVisuals(shift_enter)

                              self.vector_initializer()
                              break
                        else:
                              print("else el 90 track")
                              self.firstwarningdetect=self.firstwarningdetect+1
                              #elif(len(self.distance_coordinate) > 1):
                              #break                 

      def vector_initializer(self):#checks if the vector is orthogonal to the original
            if(self.z==0):#orthogonal
                  ##degree=90
                  local_var = 0
                  while(local_var != 2):#deals with the primary laps
                        degree1 =90
                        z1=[]
                        while(degree1 < 180):
                              next_point_x = self.circle_left_x + (((15.25/2)+1.5)*(math.sin(math.radians(degree1))))#point every 2 degrees"0.31m"
                              next_point_y = self.circle_left_y +(((15.25/2)+1.5)*(math.cos(math.radians(degree1))))
                              #   r = self.distance(next_point_x,next_point_y, self.carpos_x , self.carpos_y)
                              #   radian_shift = self.yaw - 1 ##radian 57.2957795
                              #   next_point_x = (self.carpos_x) + r*math.sin(radian_shift)
                              #   next_point_y = (self.carpos_y) + r*math.cos(radian_shift)
                              self.waypoints.append((next_point_x,next_point_y))
                              z1.append((next_point_x,next_point_y))
                              degree1 =degree1+1
                        #self.publishWaypointsVisuals(z1)
                        print("zzzzzzzzzzzzz1111111111111")
                        degree2 =180
                        z2=[]
                        while(degree2 < 360):
                              next_point_x = self.circle_left_x + (((15.25/2)+1.5)*(math.sin(math.radians(degree2))))#point every 2 degrees"0.31m"
                              next_point_y = self.circle_left_y +(((15.25/2)+1.5)*(math.cos(math.radians(degree2))))
                              #   r = self.distance(next_point_x,next_point_y, self.carpos_x , self.carpos_y)
                              #   radian_shift = self.yaw - 1 ##radian 57.2957795
                              #   next_point_x = (self.carpos_x) + r*math.sin(radian_shift)
                              #   next_point_y = (self.carpos_y) + r*math.cos(radian_shift)
                              self.waypoints.append((next_point_x,next_point_y))
                              z2.append((next_point_x,next_point_y))
                              degree2 = degree2 + 1
                        #self.publishWaypointsVisuals(z2)
                        print("zzzzzzzzzzz2222222222222")

                        if(local_var < 1):
                              degree3 = 0
                              z3=[]
                              while(degree3 != 90):
                                    next_point_x = self.circle_left_x + (((15.25/2)+1.5)*(math.sin(math.radians(degree3))))#point every 2 degrees"0.31m"
                                    next_point_y = self.circle_left_y +(((15.25/2)+1.5)*(math.cos(math.radians(degree3))))
                                    #  r = self.distance(next_point_x,next_point_y, self.carpos_x , self.carpos_y)
                                    #  radian_shift = self.yaw - 1 ##radian 57.2957795
                                    #  next_point_x = (self.carpos_x) + r*math.sin(radian_shift)
                                    #  next_point_y = (self.carpos_y) + r*math.cos(radian_shift)
                                    self.waypoints.append((next_point_x,next_point_y))
                                    z3.append((next_point_x,next_point_y))
                                    degree3 = degree3 + 1 
                              #self.publishWaypointsVisuals(z3)
                              print("zzzzzzzzzzz333333333333333333")
                              local_var = local_var + 1
                        else:
                              break
                  local_var = local_var + 1


            if(local_var == 2):
                  while(self.flag2!=2):#deals with the primary laps of the left circle
                        degree4 =0
                        z4=[]
                        while(degree4 < 360):
                              next_point_x = self.circle_right_x + (((15.25/2)+1.5)*(math.sin(math.radians(degree4))))#point every 2 degrees"0.31m"
                              next_point_y = self.circle_right_y -(((15.25/2)+1.5)*(math.cos(math.radians(degree4))))
                              # r = self.distance(next_point_x,next_point_y, self.carpos_x , self.carpos_y)
                              # radian_shift = self.yaw - 1 ##radian 57.2957795
                              # next_point_x = (self.carpos_x) + r*math.sin(radian_shift)
                              # next_point_y = (self.carpos_y) + r*math.cos(radian_shift)
                              self.waypoints.append((next_point_x,next_point_y))
                              z4.append((next_point_x,next_point_y))
                              degree4 = degree4 + 1


                        #self.publishWaypointsVisuals(z4)
                        px = z4[-1] ##px will excute exit track 
                        ##self.lap_count_exit()
                        self.flag2=self.flag2+1
                        print("zzzzz444444")
                  local_var = local_var + 1       
            self.z = self.z+1
            self.exit_er(px)                             


      def exit_er(self,pointbreak):
            #px = self.pont_exit_x
            #py= self.pont_exit_y
            #z=0
            final_cones_right=[]
            initial_point=self.waypoints[0]
            initial_point_x=self.circle_right_x+1
            initial_point_y=initial_point[1]

            for cones in self.map_right:
                  if((cones[0]>initial_point_x)and(cones[0]>self.circle_right_x)):
                        final_cones_right.append(cones)
                  else:
                        continue
            final_cones_left=[]           
            for cones in self.map_left:
                  if((cones[0]>initial_point_x)and(cones[0]>self.circle_right_x)):
                        final_cones_left.append(cones)
                  else:
                        continue   
            z=zip(final_cones_right,final_cones_left) 
            al_listat=list(z)

            for pt1,pt2 in al_listat:
                  way=self.midpointer(pt1[0],pt1[1],pt2[0],pt2[1]) 
                  if(self.hor_dist(way[1],initial_point_y)>0.5):
                        way1=(way[0],self.pont_exit_y)
                        self.waypoints.append(way1)  
                  else:  
                        pass   
                        self.waypoints.append(way) ###########################??????????????????        

            #  exit=[]
            #  while(z!=125):
            #        px=px+1
            #        self.waypoints.append((px,py))
            #        exit.append((px,py))
            #        z=z+1
            #  print("a7a 4lst")
            c=0
            exitPoint = self.waypoints[-1]
            exitPoint_X = exitPoint[0]
            while(c!=125):
                  exitPoint_X = exitPoint_X + 1
                  self.waypoints.append((exitPoint_X,initial_point_y))
                  #exit.append((px,py))
                  c=c+1
            i = 0#counter used to solve rvi_z__ publishing problem should be commented at implementation
            while(True):
                  self.publishWaypointsVisuals(self.waypoints) 





      def publishWaypointsVisuals(self, newWaypoints = None):
            # print("_______________waypoint_______________")
            print("waypoints gamed neek")
            markerArray = MarkerArray()

            savedWaypointsMarker = Marker()
            savedWaypointsMarker.header.frame_id = "map"
            savedWaypointsMarker.header.stamp = rospy.Time.now()
            savedWaypointsMarker.lifetime = rospy.Duration(100)
            savedWaypointsMarker.ns = "saved-publishWaypointsVisuals"
            savedWaypointsMarker.id = 1

            savedWaypointsMarker.type = savedWaypointsMarker.SPHERE_LIST
            savedWaypointsMarker.action = savedWaypointsMarker.ADD
            savedWaypointsMarker.scale.x = 0.3
            savedWaypointsMarker.scale.y = 0.3
            savedWaypointsMarker.scale.z = 0.3
            savedWaypointsMarker.pose.orientation.w = 1

            savedWaypointsMarker.color.a = 1
            savedWaypointsMarker.color.b = 1


            for waypoint in newWaypoints:
                  p = Point(waypoint[1], waypoint[0], 0.0)
                  savedWaypointsMarker.points.append(p)


            #print(savedWaypointsMarker)

            markerArray.markers.append(savedWaypointsMarker)

            self.waypointsVisualPub.publish(markerArray)





if __name__ == '__main__':
      try:
            planner()
      except rospy.ROSInterruptException:
            pass
      rospy.spin()
