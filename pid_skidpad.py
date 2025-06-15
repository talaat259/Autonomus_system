#!/usr/bin/env python

import time ,math
import numpy as np
import math
import rospy
import copy
from sensor_msgs import msg
import tf
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
import random
from tf.transformations import euler_from_quaternion
from scipy.spatial import Delaunay
import sys
from sensor_msgs.msg import Imu
import bisect
from ackermann_msgs.msg import AckermannDriveStamped
import matplotlib.pyplot as plt
from os import path
from nav_msgs.msg import Path






class stanley_controller:
  
  def __init__(self):


    rospy.init_node("stanley_controller", anonymous = True)
    
    self.carPosX=0.0
    self.carPosY=0.0
    
    self.now = []
    self.eta_plot = []

    self.Kp = 0.24
    self.Ki = 0.05
    self.Kd = 0.001
    self.dt = 0.1

    self.d_error=0.0
    self.i_error=0.0
    self.p_error=0.0
    self.target_sp=0.0

    self.L = 1.58  # [m] Wheel base of vehicle
    self.K = 0.78     # STANLEY GAIN:
        
    self.waypoints_x =  [0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]
    self.waypoints_y =  [0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]

    self.Vx = 0.0
    self.Vy = 0.0
    self.yaw = 0.0

    self.control_steering_angle = 0.0
    self.control_velocity = 0.0

    self.vel_cmd = 0.0
        
    self.steer_cmd = 0.0

    self.x = 0.0
    self.y = 0.0
    self.alpha = 0.0
    self.ld = 0.0
    self.look_ahead = 0.0
    self.gain = 1
    self.eta = 0.0
    self.velocity_now = []
    self.look_ahead = []
    # self.control_elf.Vx = 0.0
    self.carPosX = 0.0
    self.carPosY = 0.0
    self.acc_Waypoints_X = []
    self.acc_Waypoints_Y = []
    # self.waypoints_x = []
    # self.waypoints_y = []
    self.accel = 0.0

    rospy.Subscriber('/visual/waypoints',MarkerArray,self.waypoints_callback)
    rospy.Subscriber('/sensor_imu_hector',Imu,self.imu_callback)
    rospy.Subscriber("/ground_truth/state",Odometry,self.odom_callback)
    rospy.Subscriber("/robot_control/command",AckermannDriveStamped,self.control_callback)
    rospy.Subscriber('/car_pose', Path,self.getCarPos)
    self.robot_control_pub = rospy.Publisher("/robot_control/command",AckermannDriveStamped,queue_size=0)



  def getCarPos(self,path_msg):
    self.carPosX = path_msg.poses[-1].pose.position.x
    self.carPosY = path_msg.poses[-1].pose.position.y


  def control_callback(self,control_msg):
    self.control_steering_angle = control_msg.drive.steering_angle
    self.control_velocity = control_msg.drive.speed
   # print(self.control_velocity)
    



  def imu_callback(self,imu_msg):
    self.yaw = imu_msg.orientation.z
    



  def odom_callback(self,odom_msg):
    self.Vx = odom_msg.twist.twist.linear.x
    self.Vy = odom_msg.twist.twist.linear.y
    

  

  def waypoints_callback(self,waypoints_msg):
    
    print("waypoints callback")
    #all 500 waypoints
    
    i=0
    j=0#j iterates through waypoints and is a flag when equal to 3 i.e 3 waypoints sent break
  
    if len(waypoints_msg.markers[0].points)<2:
      return
    
    # for point in waypoints_msg.markers[0].points:
    #   self.acc_Waypoints_X.append(point.x)
    #   self.acc_Waypoints_Y.append(point.y)
    #   i+=1
    #   if(self.acc_Waypoints_X[i-1]-self.carPosX>=0 and j<3):
    #     self.waypoints_x[j]=self.acc_Waypoints_X[i-1]
    #     self.waypoints_y[j]=self.acc_Waypoints_Y[i-1]
    #     j+=1
    #   if(j>=3):
    #     break
        

    # if len(waypoints_msg.markers[0].points)<2:
    #   return
    # i=0
    # for point in waypoints_msg.markers[0].points:
    #   self.waypoints_x[i]=point.x
    #   self.waypoints_y[i]=point.y
    #   i+=1

    ### speed PID Controller ###
    self.vel_cmd = self.pid_control(self.target_speed(),self.control_velocity)
    
    ### Stanley Steering Controller ###
    self.steer_cmd = self.stanley_control(waypoints_msg)

    ### publish control command ###
    self.control_command(self.steer_cmd,self.vel_cmd) #dont forget self.vel_cmd
    self.file_saved()
  '''
  def refrence_callback(self,Path):   #first ccallback
    #print("nav callback")

    self.ref_x = []
    self.ref_y = []
    
    for point in Path.poses:
      self.ref_x.append(point.pose.position.x)
      self.ref_y.append(point.pose.position.y)
      
      '''
  
  ###Defining function for plotting
  def file_saved(self):
    i = 0;
    f = open(r'/home/fadi/dataaaa.txt','w')
    f.write("Time \n")
    
    while i<len(self.now):
      x = self.now[i]
      f.write( str(x)+"\n")
      i = i +1
    
    # f.write("Steering angle \n")
    # i=0;
    # while i<len(self.eta_plot):
    #   y = self.eta_plot[i]
    #   f.write( str(y)+"\n")
    #   i = i +1

    f.write("Speed Acceleration \n")
    i=0;
    while i<len(self.velocity_now):
      y = self.velocity_now[i]
      f.write( str(y)+"\n")
      i = i +1
    
    # f.write("Look Ahead Distance \n")
    # i=0;
    # while i<len(self.look_ahead):
    #   y = self.look_ahead[i]
    #   f.write( str(y)+"\n")
    #   i = i +1

    f.close()

    

  def stanley_control(self,waypoints_msg):

    # if self.control_velocity<3.5:
    #   self.x = self.waypoints_x[0]
    #   self.y = self.waypoints_y[0]
    
    # elif self.control_velocity<10.0:
    #   self.x = self.waypoints_x[1]
    #   self.y = self.waypoints_y[1]

    # elif self.control_velocity<30.0:
    #   self.x = self.waypoints_x[2]
    #   self.y = self.waypoints_y[2]

    # i=0
    # j=0     # j iterates through waypoints and is a flag when equal to 3 i.e 3 waypoints sent break
  
    if len(waypoints_msg.markers[0].points)<4:
      return


    for point in waypoints_msg.markers[0].points:
      self.acc_Waypoints_X.append(point.x)
      self.acc_Waypoints_Y.append(point.y)
      # i+=1
      # if(self.acc_Waypoints_X[i-1]-self.carPosX>=0 and j<3):
      #   self.waypoints_x[j]=self.acc_Waypoints_X[i-1]
      #   self.waypoints_y[j]=self.acc_Waypoints_Y[i-1]
      #   j+=1
      # if(j>=3):
      #   j=0
      #   break

    
    

    
    # print("waypoints",self.waypoints_x,self.waypoints_y)
    # print("x & y for car ",self.carPosX,self.carPosY )
    # print(self.L)
    self.Ld=math.sqrt( (self.waypoints_x[1] + self.L - (self.carPosX))**2 +  (self.waypoints_y[1] - self.carPosY)**2 )
    print(self.Ld)
    #self.Ld =1.6   # print(self.Ld)
    #self.look_ahead.append(self.Ld)
    self.alpha=math.atan((self.waypoints_y[1] - self.carPosY)/(self.waypoints_x[1]+self.L))
    print(self.alpha)
    self.eta=math.atan((2*self.L*math.sin(self.alpha))/self.Ld)
    print(self.eta)
    
    time = rospy.get_rostime()    ####
    self.now.append(time)     ####
         


    return self.eta



  def target_speed(self):
    x1=self.waypoints_x[0]
    x2=self.waypoints_x[1]
    x3=self.waypoints_x[2]
    y1=self.waypoints_y[0]
    y2=self.waypoints_y[1]
    y3=self.waypoints_y[2]

    CO_Friction=0.6
    Gravity= 9.81
    try:
      c = (x1-x2)**2 + (y1-y2)**2
      a = (x2-x3)**2 + (y2-y3)**2
      b = (x3-x1)**2 + (y3-y1)**2
      ar = a**0.5
      br = b**0.5
      cr = c**0.5 
      r = ar*br*cr / ((ar+br+cr)*(-ar+br+cr)*(ar-br+cr)*(ar+br-cr))**0.5
    except:
      self.accel=self.accel+0.065
      return self.accel
    """
    x12 = x1 - x2;
    x13 = x1 - x3;
 
    y12 = y1 - y2;
    y13 = y1 - y3;
 
    y31 = y3 - y1;
    y21 = y2 - y1;
 
    x31 = x3 - x1;
    x21 = x2 - x1;
 
    # x1^2 - x3^2
    sx13 = math.pow(x1, 2) - math.pow(x3, 2);
 
    # y1^2 - y3^2
    sy13 = math.pow(y1, 2) - math.pow(y3, 2);
 
    sx21 = math.pow(x2, 2) - math.pow(x1, 2);
    sy21 = math.pow(y2, 2) - math.pow(y1, 2);
 
    f = (((sx13) * (x12) + (sy13) *
          (x12) + (sx21) * (x13) +
          (sy21) * (x13)) // (2 *
          ((y31) * (x12) - (y21) * (x13))));
             
    g = (((sx13) * (y12) + (sy13) * (y12) +
          (sx21) * (y13) + (sy21) * (y13)) //
          (2 * ((x31) * (y12) - (x21) * (y13))));
 
    c = (-math.pow(x1, 2) - math.pow(y1, 2) -
         2 * g * x1 - 2 * f * y1);
 
        # eqn of circle be x^2 + y^2 + 2*g*x + 2*f*y + c = 0
        # where centre is (h = -g, k = -f) and
        # radius r as r^2 = h^2 + k^2 - c
    h = -g;
    k = -f;
    sqr_of_r = h * h + k * k - c;
 
    # r is the radius
    r = round(math.sqrt(sqr_of_r), 5);
    '''
    print("Centre = (", h, ", ", k, ")");
    print("Radius = ", r);  '''
 

    """
    return math.sqrt(CO_Friction * Gravity * r)




  '''def line_equation(self,x1,y1,x2,y2):

    dy = y2-y1
    dx = x2-x1

    m = dy/dx
    b = y2 - m*x2

    return m ,b'''




  def pid_control(self,target, current):
    """
    Proportional control for the speed.
    :param target: (float)
    :param current: (float)
    :return: (float)
    """
    self.d_error=(target-current)/self.dt
    self.i_error=((target-current)/2)*self.dt
    self.p_error=target-current
    return self.Kp*self.p_error + self.Ki*self.i_error + self.Kd*self.d_error
    



  def control_command(self,steering,velocity):
    control_msg = AckermannDriveStamped()
    
	    
    
    control_msg.drive.steering_angle = steering
    control_msg.drive.speed = velocity
    self.velocity_now.append(control_msg.drive.speed)
    self.robot_control_pub.publish(control_msg)
    











if __name__ == '__main__':
  try:
      control = stanley_controller()
  except rospy.ROSInterruptException:
      pass
  rospy.spin()
