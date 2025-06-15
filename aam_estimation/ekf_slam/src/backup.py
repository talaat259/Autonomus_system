#!/usr/bin/env python
from os import path
from re import U
from types import LambdaType
import numpy as np
import math
from numpy.core.fromnumeric import shape
from numpy.lib.function_base import blackman
import rospy
from aam_common_msgs.msg import Cone
from aam_common_msgs.msg import Map
from aam_common_msgs.msg import ConeDetections
from geometry_msgs.msg import Point
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
import sys
import ros_numpy
import time 
#import math
import matplotlib.pyplot as plt 
from nav_msgs.msg import Odometry
import tf 
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Vector3Stamped
from sensor_msgs.msg import Imu
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Path
#from geometry_msgs.msg import PoseStamped




class start_global_map():

  def __init__(self, namespace='data_gathering'):

    rospy.init_node("ekf_slam", anonymous = True)

    rospy.Subscriber("/lidar_cone_detection_cones",ConeDetections,callback=self.cones_callback)
    rospy.Subscriber("/robot_control/command",AckermannDriveStamped,self.control_callback)
    rospy.Subscriber("/sensor_imu_hector",Imu,self.imu_callback)
    rospy.Subscriber("/robot_control/odom",Odometry,self.odom_callback)
    
    

    self.car_pose_pub = rospy.Publisher("/car_pose", Path, queue_size=0)
    self.global_map_pub = rospy.Publisher("/global_map",MarkerArray, queue_size=0)

    
    # input control
    # control_velocity : velocity Command
    # control_steering_angle : steering angle command
    self.control_velocity = 0
    self.control_steering_angle = 0

    # measurements 
    # Vx : linear  Velocity in the X direction
    # VY : linear  Velocity in the Y direction
    # angular_vel : Vehicle angular velocity around z axis (yaw rate)
    self.Vx = 0
    self.Vy = 0
    self.angular_vel = 0

    # EKF parameters
    # STATE_SIZE : (X,Y,Yaw)
    # xEst : state estimate
    # pEst : Covariance
    self.STATE_SIZE = 3
    self.xEst = np.zeros((self.STATE_SIZE, 1)) #3*1>> will make all the values=0
    self.pEst = np.eye(self.STATE_SIZE)    #identity matrix
    
    #process initial uncertanity"used only once"
    self.Q=np.array([[0.3,0.0,0.0],
                    [0.0,0.3,0.0],
                    [0.0,0.0,0.3]])

    # time step measurement Start
    self.start_time = rospy.get_rostime().to_sec()

    # delta yaw calculation 
    # prev_yaw : previous yaw
    # delta_yaw : delta yaw
    self.prev_yaw = 0
    self.delta_yaw = 0

    # car pose predicted and logged for visualization
    # car_pose_x : car pose X
    # car_pose_y : car pose Y
    self.car_pose_x = []
    self.car_pose_y = []

    # global map constructed and logged for visualization
    # global_map_X : landmarks X
    # global_map_Y : landmarks Y
    self.mapx = []
    self.mapy = []










  def cones_callback(self,cone_detections):
    
    self.cones_x = []
    self.cones_y = []
    self.cones_color = []

    for cone in cone_detections.cone_detections:
      
      self.cones_x.append(cone.position.x)
      self.cones_y.append(cone.position.y)
      self.cones_color.append(cone.color)


    i = 0

    Xcones_filterd = []
    Ycones_filterd = []

    while i <= len(self.cones_x)-1:
      xcomp = self.cones_x[i]
      ycomp = self.cones_y[i]

      d_comp = math.sqrt((xcomp)**2 + (ycomp)**2)
      theta_comp = math.atan(ycomp/xcomp)

      j = i+1

      while j<= len(self.cones_x)-1:

        xiter = self.cones_x[j]
        yiter = self.cones_y[j]

        d_iter = math.sqrt((xiter)**2 + (yiter)**2)
        theta_iter = math.atan(yiter/xiter)
        
        if ( abs(d_iter-d_comp)<0.1 ) and  ((theta_comp-theta_iter)<0.01):
          break

        else :
          Xcones_filterd.append(xcomp)
          Ycones_filterd.append(ycomp)

        j+=1
      i+=1




    
    
    SLAM = ekf(self.xEst,self.pEst,Xcones_filterd,Ycones_filterd,self.delta_yaw,self.prev_yaw,self.angular_vel,self.control_steering_angle,self.control_velocity,self.Vx,self.Vy,self.start_time,self.Q)
    self.xEst , self.pEst , self.mapx , self.mapy = SLAM.ekf_slam()

    self.start_time = rospy.get_rostime().to_sec()



    self.car_pose_x.append(float(self.xEst[0]))
    self.car_pose_y.append(float(self.xEst[1]))

    #print(self.xEst[0],self.xEst[1])

    #plt.scatter(self.xEst[0],self.xEst[1])
    #plt.pause(0.1)
    

    # Visualize the logged path and global map on rviz
    self.viz_path()
    self.viualise_global_map(self.mapx,self.mapy)








  def control_callback(self,control_msg):
    self.control_steering_angle = control_msg.drive.steering_angle
    self.control_velocity = control_msg.drive.speed
    






  def imu_callback(self,imu_msg):

    angular_vel = imu_msg.angular_velocity.z

    self.angular_vel = angular_vel






  def odom_callback(self,odom_msg):
    self.Vx = odom_msg.twist.twist.linear.x
    self.Vy = odom_msg.twist.twist.linear.y







  def viz_path(self):
    path_msg = Path()
    path_msg.header.frame_id = 'odom'
    
    i = 0

    


    while i < len(self.car_pose_x):
      
      pose_msg = PoseStamped()
      pose_msg.pose.position.x = self.car_pose_x[i]
      pose_msg.pose.position.y = self.car_pose_y[i]

      path_msg.header.frame_id = 'odom'
      path_msg.poses.append(pose_msg)

      #print(self.car_pose_x[i],self.car_pose_y[i])


      i+=1
      

    
    self.car_pose_pub.publish(path_msg)
    







  def  viualise_global_map(self,mapx,mapy):
    cone_msg = Marker()
    global_map_msg = MarkerArray()

    c = 0

    while c < len(mapx):
      
      
      x_cone = mapx[c]
      y_cone = mapy[c]


      c +=1

      cone_msg.header.frame_id = "map"
      cone_msg.ADD
      cone_msg.SPHERE
      cone_msg.pose.position.x = x_cone
      cone_msg.pose.position.y = y_cone
      cone_msg.pose.position.z = 0
      cone_msg.pose.orientation.w = 1
      cone_msg.scale.x = 1
      cone_msg.scale.y = 1
      cone_msg.scale.z = 1
      cone_msg.color.a = 1
      cone_msg.color.r = 1
      cone_msg.color.g = 0
      cone_msg.color.b = 0
      cone_msg.mesh_resource = "package://aamfsd_description/meshes/cone_blue.dae"
      cone_msg.type = Marker.MESH_RESOURCE
      cone_msg.mesh_use_embedded_materials = True



      
      global_map_msg.markers.append(cone_msg)
      m = 0
      id = 0

      for m in global_map_msg.markers:
        m.id = id
        id += 1
    
    self.global_map_pub.publish(global_map_msg)

































class ekf():


  def __init__(self,xEst,pEst ,cones_x, cones_y,delta_yaw,prev_yaw,angular_vel,control_steering_angle,control_velocity,Vx,Vy,start_time,Q):
    

    self.xEst = xEst
    self.pEst = pEst

    self.cones_x = cones_x
    self.cones_y = cones_y

    self.delta_yaw = delta_yaw
    self.prev_yaw = prev_yaw
    self.angular_vel = angular_vel

    self.control_steering_angle = control_steering_angle
    self.control_velocity = control_velocity

    self.u = np.array([[math.sqrt(Vx**2+Vy**2), angular_vel]]).T

    self.Vx = Vx
    self.Vy = Vy
    
    self.dt = start_time - rospy.get_rostime().to_sec()
    self.Q = Q

    self.state_size = 3
    self.LM_SIZE = 2

    # EKF state covariance
    self.Cx = np.diag([0.5, 0.5, np.deg2rad(30.0)]) ** 2

    # maximum observation range
    self.MAX_RANGE = 20 

    # Threshold of Mahalanobis distance for data association.
    self.M_DIST_TH = 0.2

    self.mapx = []
    self.mapy = []









  def ekf_slam(self):
    # xEst : Estimated State
    # PEst : The uncertainty in last position
    # u : control input
    # z : landmarks measurements at this step
    # S : state size

    S = 3
    
    # get landmark EKF format from cones positions
    z = self.get_landmarks()


    ## Prediction ##
    G , Fx = self.predict(self.xEst[0:S],self.u)
    initP = np.eye(2)

    ## Update ##
    #self.update(self.xEst, self.pEst, self.u, z, initP)


    for i in range(self.calc_n_lm(self.xEst)):

      self.mapx.append(self.xEst[self.state_size + i * 2])
      self.mapy.append(self.xEst[self.state_size + i * 2 + 1])



    return self.xEst ,self.pEst , self.mapx , self.mapy
    
    

  def get_landmarks(self):
  
    z = np.zeros((0, 3))

    for i in range(len(self.cones_x)):
      x = self.cones_x[i]
      y = self.cones_y[i]
      d = math.sqrt(x**2 + y**2)
      angle = self.pi_2_pi(math.atan2(y, x))


      if d <= self.MAX_RANGE:
        zi = np.array([d, angle, i])
        z = np.vstack((z, zi))

    return z



  def pi_2_pi(self,angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi



  ################################### PREDICTION STEP ###################################



  def predict(self,x,u):
    # xEst : Estimated State
    # u : control input
    S = self.state_size

    G , Fx = self.jacob_motion(x, u)

    self.xEst[0:S] = self.motion_model(self.xEst[0:S], u)

    self.pEst[0:S, 0:S] = np.dot( G.T , np.dot( self.pEst[0:S, 0:S] , G )) + np.dot( Fx.T , np.dot(self.Cx , Fx) )

    return G, Fx
    


  def jacob_motion(self,x, u):

    """
    Calculates the jacobian of motion model. 
    Inputs:
    => x: The state, including the estimated position of the system
    => u: The control function
    Outputs: 
    => G:  Jacobian
    => Fx: STATE_SIZE x (STATE_SIZE + 2 * num_landmarks) matrix where the left side is an identity matrix
    """

    Fx = np.hstack(( np.eye(self.state_size), np.zeros((self.state_size, self.LM_SIZE * self.calc_n_lm(x))) ))

    jF = np.array([[0.0, 0.0, -self.dt * u[0, 0] * math.sin(x[2, 0])],
                   [0.0, 0.0, self.dt * u[0, 0] * math.cos(x[2, 0])],
                   [0.0, 0.0, 0.0]], dtype=float)

    #G = np.eye(self.state_size) + np.dot(Fx.T , np.dot(jF,Fx) )

    G = np.array([[0.0, 0.0,(self.Vx*math.sin(self.xEst[2])-self.Vy*math.cos(self.xEst[2]))*self.dt],
                   [0.0, 0.0,(self.Vx*math.cos(self.xEst[2])-self.Vy*math.sin(self.xEst[2]))*self.dt ],
                   [0.0, 0.0, self.dt]], dtype=float)


    return G, Fx



  def calc_n_lm(self,x):
    n = int((len(x) - self.state_size) / self.LM_SIZE)
    return n



  def motion_model(self,x, u):
    F = np.array([[1.0, 0, 0],
                  [0, 1.0, 0],
                  [0, 0, 1.0]])

    B = np.array([[self.dt * math.cos(x[2, 0]), 0],
                  [self.dt * math.sin(x[2, 0]), 0],
                  [0.0, self.dt]])

    
    #x = np.dot(F , x) + np.dot(B , u)
    phi = self.control_steering_angle 

    x[0] = x[0] - (self.Vx*math.cos(phi)-self.Vy*math.sin(phi))*self.dt
    x[1] = x[1] - (self.Vx*math.sin(phi)+self.Vy*math.cos(phi))*self.dt
    x[2] = x[2] + u[1]*self.dt

    return x






  ################################### UPDATE STEP ###################################

  def update(self,xEst, pEst, u, z, initP):
    """
    Performs the update step of EKF SLAM
    
    :param xEst:  nx1 the predicted pose of the system and the pose of the landmarks
    :param PEst:  nxn the predicted covariance
    :param u:     2x1 the control function 
    :param z:     the measurements read at new position
    :param initP: 2x2 an identity matrix acting as the initial covariance
    :returns:     the updated state and covariance for the system
    """

    for iz in range(len(z[:, 0])):  # for each observation

      min_id = self.search_correspond_landmark_id(self.xEst, self.pEst, z[iz, 0:2]) # associate to a known landmark

      nLM = self.calc_n_lm(self.xEst)

      if min_id == nLM:
        #print("New LM")
        # Extend state and covariance matrix
        xAug = np.vstack((self.xEst, self.calc_landmark_position(self.xEst, z[iz, :])))
        PAug = np.vstack((np.hstack((self.pEst, np.zeros((len(self.xEst), self.LM_SIZE)))),
                          np.hstack((np.zeros((self.LM_SIZE, len(self.xEst))), initP))))
        self.xEst = xAug
        self.pEst = PAug

      lm = self.get_landmark_position_from_state(self.xEst, min_id)
      y, S, H = self.calc_innovation(lm, self.xEst, self.pEst, z[iz, 0:2], min_id)
      
      K = np.dot(np.dot(self.pEst , H.T) , np.linalg.inv(S))
      self.xEst = self.xEst + np.dot(K , y)
      self.pEst = np.eye(len(self.xEst)) - np.dot(np.dot(K , H),self.pEst)

    self.xEst[2] = self.pi_2_pi(self.xEst[2])

    return self.xEst, self.pEst


      
  def search_correspond_landmark_id(self,xAug, PAug, zi):
    """
    Landmark association with Mahalanobis distance
    """

    nLM = self.calc_n_lm(xAug)
    
    min_dist = []

    for i in range(nLM):
      lm = self.get_landmark_position_from_state(xAug, i)
      y, S, H = self.calc_innovation(lm, xAug, PAug, zi, i)
      min_dist.append(np.dot( y.T , np.dot( np.linalg.inv(S) , y ) ) )

    min_dist.append(self.M_DIST_TH)  # new landmark

    min_id = min_dist.index(min(min_dist))

    return min_id



  def calc_n_lm(self,x):
    n = int((len(x) - self.state_size) / self.LM_SIZE)
    
    return n

  

  def get_landmark_position_from_state(self,x, ind):
      lm = x[self.state_size + self.LM_SIZE * ind: self.state_size + self.LM_SIZE * (ind + 1), :]

      return lm



  def calc_innovation(self,lm, xEst, PEst, z, LMid):
    delta = lm - xEst[0:2]
    q = np.dot(delta.T , delta)[0, 0]
    z_angle = math.atan2(delta[1, 0], delta[0, 0]) - xEst[2, 0]
    zp = np.array([[math.sqrt(q), self.pi_2_pi(z_angle)]])
    y = (z - zp).T
    y[1] = self.pi_2_pi(y[1])
    H = self.jacob_h(q, delta, xEst, LMid + 1)

    #print(H.shape,PEst.shape , H.T.shape ,self.Cx[0:2, 0:2].shape)
    S = np.dot( H , np.dot(PEst , H.T) ) + self.Cx[0:2, 0:2]

    return y, S, H



  def jacob_h(self,q, delta, x, i):
    sq = math.sqrt(q)
    G = np.array([[-sq * delta[0, 0], - sq * delta[1, 0], 0, sq * delta[0, 0], sq * delta[1, 0]],
                  [delta[1, 0], - delta[0, 0], - q, - delta[1, 0], delta[0, 0]]])

    G = G / q
    nLM = self.calc_n_lm(x)
    F1 = np.hstack((np.eye(3), np.zeros((3, 2 * nLM))))
    F2 = np.hstack((np.zeros((2, 3)), np.zeros((2, 2 * (i - 1))),
                    np.eye(2), np.zeros((2, 2 * nLM - 2 * i))))

    F = np.vstack((F1, F2))

    H = np.dot( G , F )

    return H



  def calc_landmark_position(self,x, z):
    zp = np.zeros((2, 1))

    zp[0, 0] = x[0, 0] + z[0] * math.cos(x[2, 0] + z[1])
    zp[1, 0] = x[1, 0] + z[0] * math.sin(x[2, 0] + z[1])

    return zp















if __name__ == '__main__':
  try:
    global_map = start_global_map()
    
  except rospy.ROSInterruptException:
    pass
  rospy.spin()