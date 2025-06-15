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
import csv












class RRT_Path_Plan_node:
  
  
  
  
  
  def __init__(self,namespace='rrt'):
    
    
  
    rospy.init_node("rrt_path_planner", anonymous = True)
    
    rospy.Subscriber("/centroid_lidar_cones_marker", MarkerArray,self.cones_pipeline)
    rospy.Subscriber("robot_control/odom",Odometry,self.odometryCallback)    

    
    self.shouldPublishWaypoints = rospy.get_param('~publishWaypoints', True)
     
    # Create publishers
    self.waypointsVisualPub = rospy.Publisher("/visual/waypoints", MarkerArray, queue_size=0)
    # visuals
    
    self.treeVisualPub = rospy.Publisher("/treeVisualPub",MarkerArray,queue_size=0)
    self.bestBranchVisualPub = rospy.Publisher("/best_tree_branch", Marker,queue_size=0)
    self.filteredBranchVisualPub = rospy.Publisher("/filtered_tree_branch", Marker,queue_size=0)
    self.delaunayLinesVisualPub = rospy.Publisher("/delaunay_lines", Marker,queue_size=1)
    self.waypointsVisualPub = rospy.Publisher("/visual/waypoints", MarkerArray, queue_size=1)
    
    
    # parameters
    
    # car initial conditions
    self.carPosX = 0
    self.carPosY = 0
    self.carPosYaw = 0
    
    self.savedWaypoints = []

    self.preliminaryLoopClosure = False
    self.loopClosure = False

    self.lastPublishWaypointsTime = 0
    
    self.filteredBestBranch = []

    self.discardAmount = 0
    self.intersection = None

    

  

  ####################################################
  # Getting the x and y cones from the lidar pipeline
  #####################################################
  def cones_pipeline(self,cones_msg):
            
    cones_x = []
    cones_y  = []
    
    global map_x
    global map_y
    
     
     
    
    
    map_x = cones_x
    map_y = cones_y
    
    for cone in cones_msg.markers:
      cones_x.append(cone.pose.position.x) 
      cones_y.append(cone.pose.position.y) 
    if((len(cones_x))or(len(cones_y))>=0):
     self.sample_tree()
    else:
      pass   
      
    #print(self.map_x,self.map_y)

    #print('x cone =',cones_x)
    #print('y cone = ',cones_y)
 






  
  #########################################################
  #Getting vehicle orientation from the odometry messages
  #########################################################
  def odometryCallback(self, odom_msg):
    # rospy.loginfo("odometryCallback")
    orientation_q = odom_msg.pose.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    self.car_orientation_w = odom_msg.pose.pose.orientation.w
    (roll, pitch, yaw)  = euler_from_quaternion(orientation_list)


    
    '''
    self.carPosX = odom_msg.pose.pose.position.x
    self.carPosY = odom_msg.pose.pose.position.y
    self.carPosYaw = yaw
    '''
    #print(self.carPosX,self.carPosY,self.carPosYaw)
    







  ##################################################################################################### 
  #solving it in a trigonmetric way to obtain the distance between the 2 points in x and y direction 
  #####################################################################################################    
  def dist(self, x1, y1, x2, y2, shouldSqrt = True):       
    
    distSq = (x1 - x2) ** 2 + (y1 - y2) ** 2
    return math.sqrt(distSq) if shouldSqrt else distSq






  
  
  
  ##############################################################################################
  #calculating the heading vector of the vechile and the distance between it and the front cones
  ##############################################################################################
  def getFrontConeObstacles(self, mapX,mapY, frontDist):
        
    if not mapX:
      return []

    headingVector = self.getHeadingVector()
    # print("headingVector:", headingVector)

    headingVectorOrt = [-headingVector[1], headingVector[0]]
    # print("headingVectorOrt:", headingVectorOrt)

    behindDist = 0.5
    carPosBehindPoint = [self.carPosX - behindDist * headingVector[0], self.carPosY - behindDist * headingVector[1]]

    # print "carPos:", [self.carPosX, self.carPosY]
    # print "carPosBehindPoint:", carPosBehindPoint

    frontDistSq = frontDist ** 2
    #print(frontDist)
    frontConeList = [] 
    
    i = 0 
    while i < len(mapX):
      if (headingVectorOrt[0] * (mapY[i] - carPosBehindPoint[1]) - headingVectorOrt[1] * (mapX[i] - carPosBehindPoint[0])) < 0:
        if ((mapX[i] - self.carPosX) ** 2 + (mapY[i] - self.carPosY) ** 2) < frontDistSq:
          cone = Cone()
          cone.position.x = mapX[i]
          cone.position.y = mapY[i]
          frontConeList.append(cone)
          
      i+=1
      


    return frontConeList








  ##################################################
  #Calculation of the 2D array of the heading vector 
  ###################################################
  def getHeadingVector(self):
    headingVector = [1.0, 0]
    carRotMat = np.array([[math.cos(self.carPosYaw), -math.sin(self.carPosYaw)], [math.sin(self.carPosYaw), math.cos(self.carPosYaw)]])
    headingVector = np.dot(carRotMat, headingVector)
    return headingVector

  def findBestBranch(self,leafNodes, nodeList, largerGroupFrontCones, coneObstacleSize, expandDistance, planDistance):
      if not leafNodes:
        return

      coneDistLimit = 4.0
      coneDistanceLimitSq = coneDistLimit * coneDistLimit;
      #print(coneDistLimit)
      #print(coneDistanceLimitSq)
      bothSidesImproveFactor = 3
      minAcceptableBranchRating = 80 
      #print(bothSidesImproveFactor)
      #print(minAcceptableBranchRating)
      # everyPointDistChangeLimit = 2.0
      # print(everyPointDistChangeLimit)
      
      leafRatings = []
      
      for leaf in leafNodes:
          branchRating = 0
          node = leaf
          # print " ===== calculating leaf node {0} ====== ".format(leaf)
          while node.parent is not None:
          
              nodeRating = 0
              # print "---- check node {0}".format(node)

              leftCones = []
              rightCones = []
              for cone in largerGroupFrontCones:
                  coneDistSq = ((cone.position.x - node.x) ** 2 + (cone.position.y - node.y) ** 2)
                  #print(coneDistSq)
                  if coneDistSq < coneDistanceLimitSq:  
                      actualDist = math.sqrt(coneDistSq)
                      #print(actualDist)
                      if actualDist < coneObstacleSize:
                          continue
                      nodeRating += (coneDistLimit - actualDist)
                      #print(nodeRating)
                      # print "found close cone({1},{2}), rating: {0}".format(nodeRating, cone.x, cone.y)          
                      
                      if self.isLeftCone(node, nodeList[node.parent], cone):
                          leftCones.append(cone)
                      else:
                          rightCones.append(cone)
                        
  
              if ((len(leftCones) == 0 and len(rightCones)) > 0 or (len(leftCones) > 0 and len(rightCones) == 0)):
                  # print "cones are only from one side, penalize rating"
                  nodeRating /= bothSidesImproveFactor
    
              if (len(leftCones) > 0 and len(rightCones) > 0):
                  # print "cones are from both sides, improve rating"
                  nodeRating *= bothSidesImproveFactor
              
              nodeFactor = (node.cost - expandDistance)/(planDistance - expandDistance) + 1
              #print(nodeFactor)
              branchRating += nodeRating * nodeFactor
              node = nodeList[node.parent]
              #print(node)
          leafRatings.append(branchRating)
      maxRating = max(leafRatings)
      #print(maxRating)
      maxRatingInd = leafRatings.index(maxRating)
      #print(maxRatingInd)
      node = leafNodes[maxRatingInd]
      #print(node)
    
    # # print "!!maxRating leaf node {0}, rating: {1}".format(node, maxRating)
      if maxRating < minAcceptableBranchRating:
          return  
      self.publishBestBranchVisual(nodeList, node)
      reverseBranch = []
      reverseBranch.append(node)
      while node.parent is not None:
          node = nodeList[node.parent]
          reverseBranch.append(node)

      directBranch = []
      for n in reversed(reverseBranch):
          directBranch.append(n)
          # print n

      return directBranch
  def isLeftCone(self, node, parentNode, cone):
      # //((b.X - a.X)*(cone.Y - a.Y) - (b.Y - a.Y)*(cone.X - a.X)) > 0;
      return ((node.x - parentNode.x) * (cone.position.y - parentNode.y) - (node.y - parentNode.y) * (cone.position.x - parentNode.x)) > 0;
  
  def getFilteredBestBranch(self, bestBranch):
      
      shouldDiscard = 0
      if not bestBranch:
          return
      changeRate = 0
      everyPointDistChangeLimit = 2.0
      #print(everyPointDistChangeLimit)
      newPointFilter = 0.2
      #print(newPointFilter)
      maxDiscardAmountForReset = 2
      
      if not self.filteredBestBranch:
          self.filteredBestBranch = list(bestBranch)
      else:
        changeRate = 0
        #print(changeRate)
        shouldDiscard = False
      
      for i in range(len(bestBranch)):
       node = bestBranch[i]
       filteredNode = self.filteredBestBranch[i]

       dist = math.sqrt((node.x - filteredNode.x) ** 2 + (node.y - filteredNode.y) ** 2)
       
       if dist > everyPointDistChangeLimit: # changed too much, skip this branch
           shouldDiscard = True
           self.discardAmount += 1
           # print "above DistChangeLimit:, shouldDiscard!,", "discAmount:", self.discardAmount
      
           if self.discardAmount >= maxDiscardAmountForReset:
               self.discardAmount = 0
               self.filteredBestBranch = list(bestBranch)
               # print "broke maxDiscardAmountForReset:, Reset!"
           break
      
       changeRate += (everyPointDistChangeLimit - dist)
       #print(changeRate)
      # print "branch changeRate: {0}".format(changeRate);

      if not shouldDiscard:
          
          for i in range(len(bestBranch)):      
    
              self.filteredBestBranch[i].x = self.filteredBestBranch[i].x * (1 - newPointFilter) + newPointFilter * bestBranch[i].x
              self.filteredBestBranch[i].y = self.filteredBestBranch[i].y * (1 - newPointFilter) + newPointFilter * bestBranch[i].y
          self.discardAmount = 0
      
      self.publishFilteredBranchVisual()

      return list(self.filteredBestBranch) # return copy
  
  
  
  def getDelaunayEdges(self,cones_x,cones_y):
    
    if len(cones_x)<4:
      return

    conePoints = np.zeros((len(cones_x), 2))

    for i in range(len(cones_x)):
      conePoints[i] = ([cones_x[i], cones_y[i]])
    
    tri = Delaunay(conePoints)

    delaunayEdges = []
    for simp in tri.simplices:
        for i in range(3):
          j = i + 1
          if j == 3:
            j = 0
          edge = Edge(conePoints[simp[i]][0], conePoints[simp[i]][1], conePoints[simp[j]][0], conePoints[simp[j]][1])

          if edge not in delaunayEdges:
            delaunayEdges.append(edge)
    
    #print(delaunayEdges)
    return delaunayEdges
  
  def ccw(self, A, B, C):
        # if three points are listed in a counterclockwise order.
        # return (C.y-A.y) * (B.x-A.x) > (B.y-A.y) * (C.x-A.x)
        return (C[1]-A[1]) * (B[0]-A[0]) > (B[1]-A[1]) * (C[0]-A[0])

  
  def getLineSegmentIntersection(self, a1, a2, b1, b2):
      # https://bryceboe.com/2006/10/23/line-segment-intersection-algorithm/
      # Return true if line segments a1a2 and b1b2 intersect
      # return ccw(A,C,D) != ccw(B,C,D) and ccw(A,B,C) != ccw(A,B,D)
      return self.ccw(a1,b1,b2) != self.ccw(a2,b1,b2) and self.ccw(a1,a2,b1) != self.ccw(a1,a2,b2)
  
  
  def getLineIntersection(self, a1, a2, b1, b2):
      """
      Returns the point of intersection of the lines passing through a2,a1 and b2,b1.
      a1: [x, y] a point on the first line
      a2: [x, y] another point on the first line
      b1: [x, y] a point on the second line
      b2: [x, y] another point on the second line
      https://stackoverflow.com/questions/3252194/numpy-and-line-intersections
      """
      s = np.vstack([a1,a2,b1,b2])        # s for stacked
      h = np.hstack((s, np.ones((4, 1)))) # h for homogeneous
      l1 = np.cross(h[0], h[1])           # get first line
      l2 = np.cross(h[2], h[3])           # get second line
      x, y, z = np.cross(l1, l2)          # point of intersection
      if z == 0:                          # lines are parallel
          return (float('inf'), float('inf'))
      return (x/z, y/z)
  
  
  
  def getWaypointsFromEdges(self, filteredBranch, delaunayEdges):
      if not delaunayEdges:
          return

      waypoints = []
      for i in range (len(filteredBranch) - 1):
          node1 = filteredBranch[i]
          node2 = filteredBranch[i+1]
          a1 = np.array([node1.x, node1.y])
          a2 = np.array([node2.x, node2.y])

          # print "node1:", node1
          # print "node2:", node2

          maxAcceptedEdgeLength = 7
          maxEdgePartsRatio = 3

          intersectedEdges = []
          for edge in delaunayEdges:
              # print "edge:", edge

              b1 = np.array([edge.x1, edge.y1])
              b2 = np.array([edge.x2, edge.y2])
              if self.getLineSegmentIntersection(a1, a2, b1, b2):
                  if edge.length() < maxAcceptedEdgeLength:
                      edge.intersection = self.getLineIntersection(a1, a2, b1, b2)
             
                      edgePartsRatio = edge.getPartsLengthRatio()
                      if edgePartsRatio < maxEdgePartsRatio:
                          intersectedEdges.append(edge)

          if intersectedEdges:
              # print "len(intersectedEdges):", len(intersectedEdges)
              # print "intersectedEdges:", intersectedEdges

              if len(intersectedEdges) == 1:
                  edge = intersectedEdges[0]

                  # print "edge middle:", edge.getMiddlePoint()
                  waypoints.append(edge.getMiddlePoint())
                  
              else:
                  # print "initial:", intersectedEdges
                  intersectedEdges.sort(key=lambda edge: self.dist(node1.x, node1.y, edge.intersection[0], edge.intersection[1], shouldSqrt = False))
                  # print "sorted:", intersectedEdges

                  for edge in intersectedEdges:
                      waypoints.append(edge.getMiddlePoint())
      return waypoints

  
  
  def mergeWaypoints(self, newWaypoints):
      # print "mergeWaypoints:", "len(saved):", len(self.savedWaypoints), "len(new):", len(newWaypoints)
      if not newWaypoints:
          return;

      maxDistToSaveWaypoints = 2.0
      maxWaypointAmountToSave = 2
      waypointsDistTollerance = 0.5

      # check preliminary loopClosure
      if len(self.savedWaypoints) > 15:
          firstSavedWaypoint = self.savedWaypoints[0]

          for waypoint in reversed(newWaypoints):
              distDiff = self.dist(firstSavedWaypoint[0], firstSavedWaypoint[1], waypoint[0], waypoint[1])
              if distDiff < waypointsDistTollerance:
                  self.preliminaryLoopClosure = True
                  # print "preliminaryLoopClosure = True"
                  break

      # print "savedWaypoints before:", self.savedWaypoints
      # print "newWaypoints:", newWaypoints

      newSavedPoints = []

      for i in range(len(newWaypoints)):
          waypointCandidate = newWaypoints[i]

          carWaypointDist = self.dist(self.carPosX, self.carPosY, waypointCandidate[0], waypointCandidate[1])
          # print "check candidate:", waypointCandidate, "with dist:", carWaypointDist

          if i >= maxWaypointAmountToSave or carWaypointDist > maxDistToSaveWaypoints:
              # print "condition to break:", i, i >= maxWaypointAmountToSave,  "or", (carWaypointDist > maxDistToSaveWaypoints)
              break
          else:
              for savedWaypoint in reversed(self.savedWaypoints):
                  waypointsDistDiff = self.dist(savedWaypoint[0], savedWaypoint[1], waypointCandidate[0], waypointCandidate[1])
                  if waypointsDistDiff < waypointsDistTollerance:
                      self.savedWaypoints.remove(savedWaypoint) #remove similar
                      # print "remove this point:", savedWaypoint, "with diff:", waypointsDistDiff
                      break

              if (self.preliminaryLoopClosure):
                  distDiff = self.dist(firstSavedWaypoint[0], firstSavedWaypoint[1], waypointCandidate[0], waypointCandidate[1])
                  if distDiff < waypointsDistTollerance:
                      self.loopClosure = True
                      # print "loopClosure = True"
                      break

              # print "add this point:", waypointCandidate
              self.savedWaypoints.append(waypointCandidate)
              newSavedPoints.append(waypointCandidate)

      if newSavedPoints: # make self.savedWaypoints and newWaypoints having no intersection
          for point in newSavedPoints:
              newWaypoints.remove(point)

      # print "savedWaypoints after:", self.savedWaypoints
      # print "newWaypoints after:", newWaypoints

  
  ##########################################################################
  #Generating my sample nodes through the track so that I can create a tree
  ###########################################################################
  def sample_tree(self):
    if not map_x:
      return
    
    
    frontConesDist = 5
    frontCones = self.getFrontConeObstacles(map_x,map_y,frontConesDist)
    #print(frontCones)

    coneObstacleSize = 1.6####################changed was 1.1 
    coneObstacleList = []
    rrtConeTargets = []
    coneTargetsDistRatio = 1.2
    
    for cone in frontCones:
        coneObstacleList.append((cone.position.x, cone.position.y, coneObstacleSize))
        #print(coneObstacleList)
      
        coneDist = self.dist(self.carPosX, self.carPosY, cone.position.x, cone.position.y)
        #print(coneDist)
      
      
        if coneDist > frontConesDist * coneTargetsDistRatio:
            rrtConeTargets.append((cone.position.x, cone.position.x, coneObstacleSize))

      #print('cone targets = ',rrtConeTargets) 
      

    #Set Initial parameters
    start = [self.carPosX, self.carPosY, self.carPosYaw]
    iterationNumber = 1200
    planDistance = 12
    expandDistance = 0.5
    expandAngle = 25###############
      
    ########################################################################
    #This class is imported to this code from the other file called ma_rrt#
    ########################################################################
    rrt = RRT(start, planDistance, obstacleList=coneObstacleList, expandDis=expandDistance, turnAngle=expandAngle, maxIter=iterationNumber, rrtTargets = rrtConeTargets)
    nodeList, leafNodes = rrt.planning()
    self.publishTreeVisual(nodeList, leafNodes)

    frontConesBiggerDist = 15
    largerGroupFrontCones = self.getFrontConeObstacles(map_x,map_y, frontConesBiggerDist)
    
    # BestBranch
    # findBesttBranchStartTime = time.time()
    bestBranch = self.findBestBranch(leafNodes, nodeList, largerGroupFrontCones, coneObstacleSize, expandDistance, planDistance)
    # print "find best branch time: {0} ms".format((time.time() - findBesttBranchStartTime) * 1000);

    # print "best branch", bestBranch    
    

    if bestBranch:
        filteredBestBranch = self.getFilteredBestBranch(bestBranch)
        # print "filteredBestBranch", filteredBestBranch
        
        if filteredBestBranch:
            # Delaunay
            # delaunayStartTime = time.time()
            delaunayEdges = self.getDelaunayEdges(map_x,map_y)
            # print "delaunay time: {0} ms".format((time.time() - delaunayStartTime) * 1000);

        self.publishDelaunayEdgesVisual(delaunayEdges)

        # findWaypointsStartTime = time.time()


        if delaunayEdges:
            # print "len(delaunayEdges):", len(delaunayEdges)
            # print delaunayEdges

            newWaypoints = self.getWaypointsFromEdges(filteredBestBranch, delaunayEdges)
        # else:
        #     print "newWaypoints from filteredBestBranch", newWaypoints
        #     newWaypoints = [(node.x, node.y) for node in filteredBestBranch]

        # print "find waypoints time: {0} ms".format((time.time() - findWaypointsStartTime) * 1000)
        if newWaypoints:
            # print "newWaypoints:", waypoints

            # mergeWaypointsStartTime = time.time()
            self.mergeWaypoints(newWaypoints)
            # print "merge waypoints time: {0} ms".format((time.time() - mergeWaypointsStartTime) * 1000);
            self.publishWaypointsVisuals(newWaypoints)
  
  def publishTreeVisual(self, nodeList, leafNodes):


      if not nodeList and not leafNodes:
        return
      

      markerArray = MarkerArray()

      # tree lines marker
      treeMarker = Marker()
      treeMarker.header.frame_id = "base_link"
      treeMarker.header.stamp = rospy.Time.now()
      treeMarker.ns = "rrt"

      treeMarker.type = treeMarker.LINE_LIST
      treeMarker.action = treeMarker.ADD
      treeMarker.scale.x = 0.03

      treeMarker.pose.orientation.w = 1

      treeMarker.color.a = 0.7
      treeMarker.color.g = 0.7

      treeMarker.lifetime = rospy.Duration(0.2)

      for node in nodeList:
        if node.parent is not None:
          p = Point(node.x, node.y, 0)
          treeMarker.points.append(p)

          p = Point(nodeList[node.parent].x, nodeList[node.parent].y, 0)
          treeMarker.points.append(p)

      markerArray.markers.append(treeMarker)

      # leaves nodes marker
      leavesMarker = Marker()
      leavesMarker.header.frame_id = "base_link"
      leavesMarker.header.stamp = rospy.Time.now()
      leavesMarker.lifetime = rospy.Duration(0.2)
      leavesMarker.ns = "rrt-leaves"

      leavesMarker.type = leavesMarker.SPHERE_LIST
      leavesMarker.action = leavesMarker.ADD
      leavesMarker.pose.orientation.w = 1
      leavesMarker.scale.x = 0.15
      leavesMarker.scale.y = 0.15
      leavesMarker.scale.z = 0.15

      leavesMarker.color.a = 0.5
      leavesMarker.color.b = 0.1

      for node in leafNodes:
          p = Point(node.x, node.y, 0)
          leavesMarker.points.append(p)

      markerArray.markers.append(leavesMarker)

      # publis marker array
      self.treeVisualPub.publish(markerArray)

  def publishBestBranchVisual(self, nodeList, leafNode):
      marker = Marker()
      marker.header.frame_id = "base_link"
      marker.header.stamp = rospy.Time.now()
      marker.lifetime = rospy.Duration(0.2)
      marker.ns = "publishBestBranchVisual"

      marker.type = marker.LINE_LIST
      marker.action = marker.ADD
      marker.scale.x = 0.07

      marker.pose.orientation.w = 1

      marker.color.a = 0.7
      marker.color.r = 1.0

      node = leafNode

      parentNodeInd = node.parent
      while parentNodeInd is not None:
          parentNode = nodeList[parentNodeInd]
          p = Point(node.x, node.y, 0)
          marker.points.append(p)

          p = Point(parentNode.x, parentNode.y, 0)
          marker.points.append(p)

          parentNodeInd = node.parent
          node = parentNode

      self.bestBranchVisualPub.publish(marker)

  def publishFilteredBranchVisual(self):
    
        if not self.filteredBestBranch:
            return

        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.header.stamp = rospy.Time.now()
        marker.lifetime = rospy.Duration(0.2)
        marker.ns = "publisshFilteredBranchVisual"

        marker.type = marker.LINE_LIST
        marker.action = marker.ADD
        marker.scale.x = 0.07

        marker.pose.orientation.w = 1

        marker.color.a = 0.7
        marker.color.b = 1.0

        for i in range(len(self.filteredBestBranch)):
            node = self.filteredBestBranch[i]
            p = Point(node.x, node.y, 0)
            if i != 0:
                marker.points.append(p)

            if i != len(self.filteredBestBranch) - 1:
                marker.points.append(p)

        self.filteredBranchVisualPub.publish(marker)

  def publishDelaunayEdgesVisual(self, edges):
      if not edges:
          return

      marker = Marker()
      marker.header.frame_id = "base_link"
      marker.header.stamp = rospy.Time.now()
      marker.lifetime = rospy.Duration(1)
      marker.ns = "publishDelaunayLinesVisual"

      marker.type = marker.LINE_LIST
      marker.action = marker.ADD
      marker.scale.x = 0.05

      marker.pose.orientation.w = 1

      marker.color.a = 0.5
      marker.color.r = 1.0
      marker.color.b = 1.0

      for edge in edges:
          # print edge

          p1 = Point(edge.x1, edge.y1, 0)
          p2 = Point(edge.x2, edge.y2, 0)

          marker.points.append(p1)
          marker.points.append(p2)

      self.delaunayLinesVisualPub.publish(marker)
      return
  
  def publishWaypointsVisuals(self, newWaypoints = None):
    
    markerArray = MarkerArray()

    savedWaypointsMarker = Marker()
    savedWaypointsMarker.header.frame_id = "base_link"
    savedWaypointsMarker.header.stamp = rospy.Time.now()
    savedWaypointsMarker.lifetime = rospy.Duration(0.1)
    savedWaypointsMarker.ns = "saved-publishWaypointsVisuals"
    savedWaypointsMarker.id = 1

    savedWaypointsMarker.type = savedWaypointsMarker.SPHERE_LIST
    savedWaypointsMarker.action = savedWaypointsMarker.ADD
    savedWaypointsMarker.scale.x = 0.3
    savedWaypointsMarker.scale.y = 0.3
    savedWaypointsMarker.scale.z = 0.3
    savedWaypointsMarker.pose.orientation.w = 1
    
    

    savedWaypointsMarker.color.a = 1.0
    savedWaypointsMarker.color.b = 1.0
    for waypoint in newWaypoints:
        p = Point(waypoint[0], waypoint[1], 0.0)
        #print(waypoint)
        savedWaypointsMarker.points.append(p)

    markerArray.markers.append(savedWaypointsMarker)

    self.waypointsVisualPub.publish(markerArray)


class RRT():


  def __init__(self, start, planDistance, obstacleList, expandDis=0.5, turnAngle=20, maxIter=1000, rrtTargets = None):

    self.start = Node(start[0],start[1],start[2])
    self.startYaw = start[2]

    
    self.planDistance = planDistance
    self.expandDis = expandDis
    self.turnAngle = math.radians(turnAngle)
    
    self.maxDepth = int(planDistance / expandDis)
    # print(self.maxDepth)
    
    self.maxIter = maxIter
    self.obstacleList = obstacleList
    self.rrtTargets = rrtTargets
    # self.end = Node(0, planDistance)
    
    self.aboveMaxDistance = 0
    self.belowMaxDistance = 0
    self.collisionHit = 0
    self.doubleNodeCount = 0
    
    self.savedRandoms = []
      
      
      
      
      
  

  def planning(self):
    #print(self.nodeList)
    #print(self.start)
    
    self.nodeList = [self.start]
    self.leafNodes = []
    

    for i in range(self.maxIter):
      
      rnd = self.get_random_point_from_target_list()#
      #print(rnd)
      #print((self.nodeList))
      
      nind = self.GetNearestListIndex(self.nodeList, rnd)
      #print(nind)
      
      nearestNode = self.nodeList[nind]
      #print(nearestNode.x,nearestNode.y,nearestNode.yaw,nearestNode.cost,nearestNode.parent)
      
      if (nearestNode.cost >= self.planDistance):
        continue
      
      newNode = self.steerConstrained(rnd, nind)
            
      if newNode in self.nodeList:
        # self.doubleNodeCount += 1
        continue
      
      if self.__CollisionCheck(newNode, self.obstacleList):
            
        # nearinds = self.find_near_nodes(newNode)
        # newNode = self.choose_parent(newNode, nearinds)
        self.nodeList.append(newNode)
        # self.rewire(newNode, nearinds)
        
        
        #print(newNode.cost,self.planDistance)
        if (newNode.cost >= self.planDistance):

          #print("got a leaf " + str(newNode))
          self.leafNodes.append(newNode)
    
      # print(len(self.nodeList))
          return self.nodeList, self.leafNodes
      
      
        
        
        
        
            
            
  def __CollisionCheck(self, node, obstacleList):
    for (ox, oy, size) in obstacleList:
        dx = ox - node.x
        dy = oy - node.y
        d = dx * dx + dy * dy
        if d <= size ** 2:
            return False  # collision
    return True 






  def get_random_point_from_target_list(self):
      
    maxTargetAroundDist = 3
  
    if not self.rrtTargets:
      return self.get_random_point()

    targetId = np.random.randint(len(self.rrtTargets))
    x, y, oSize = self.rrtTargets[targetId]

    # square idea
    # randX = random.uniform(-maxTargetAroundDist, maxTargetAroundDist)
    # randY = random.uniform(-maxTargetAroundDist, maxTargetAroundDist)
    # finalRnd = [x + randX, y + randY]

    # circle idea
    randAngle = random.uniform(0, 2 * math.pi)
    randDist = random.uniform(oSize, maxTargetAroundDist)
    finalRnd = [x + randDist * math.cos(randAngle), y + randDist * math.sin(randAngle)]

    return finalRnd
            
            

            
            

  def GetNearestListIndex(self, nodeList, rnd):
    dlist = [(node.x - rnd[0]) ** 2 + (node.y - rnd[1]) ** 2 for node in nodeList]
    minind = dlist.index(min(dlist))
    #print(minind)
    return minind






  def get_random_point(self):

    randX = random.uniform(0, self.planDistance)
    randY = random.uniform(-self.planDistance, self.planDistance)
    rnd = [randX, randY]

    car_rot_mat = np.array([[math.cos(self.startYaw), -math.sin(self.startYaw)], [math.sin(self.startYaw), math.cos(self.startYaw)]])
    rotatedRnd = np.dot(car_rot_mat, rnd)

    rotatedRnd = [rotatedRnd[0] + self.start.x, rotatedRnd[1] + self.start.y]
    return rotatedRnd







  def steerConstrained(self, rnd, nind):
    # expand tree
    nearestNode = self.nodeList[nind]
    theta = math.atan2(rnd[1] - nearestNode.y, rnd[0] - nearestNode.x)

    # print "theta: {0}".format(math.degrees(theta));
    # print "nearestNode.yaw: {0}".format(math.degrees(nearestNode.yaw));

    # dynamic constraints
    #print(theta,nearestNode.yaw)
    angleChange = self.pi_2_pi(theta - nearestNode.yaw)

    # print "angleChange: {0}".format(math.degrees(angleChange));

    angle30degree = math.radians(30)

    if angleChange > angle30degree:
        angleChange = self.turnAngle
    elif angleChange >= -angle30degree:
        angleChange = 0
    else:
        angleChange = -self.turnAngle
    # print "angleChange2: {0}".format(math.degrees(angleChange));

    newNode = copy.deepcopy(nearestNode)
    newNode.yaw += angleChange
    newNode.x += self.expandDis * math.cos(newNode.yaw)
    newNode.y += self.expandDis * math.sin(newNode.yaw)

    newNode.cost += self.expandDis
    newNode.parent = nind
    #print(newNode.x,newNode.yaw,newNode.y,newNode.cost,newNode.parent)

    # print "newNode: {0}".format(newNode)
    return newNode
  
  
  




  def pi_2_pi(self, angle):
    return (angle + math.pi) % (2*math.pi) - math.pi






class Node():


  def __init__(self, x, y, yaw):
      self.x = x
      self.y = y
      self.yaw = yaw
      self.cost = 0.0
      self.parent = None



class Edge():
    def __init__(self, x1, y1, x2, y2):
        self.x1 = x1
        self.y1 = y1
        self.x2 = x2
        self.y2 = y2
        self.intersection = []

    def getMiddlePoint(self):
        return (self.x1 + self.x2) / 2, (self.y1 + self.y2) / 2

    def length(self):
        return math.sqrt((self.x1 - self.x2) ** 2 + (self.y1 - self.y2) ** 2)

    def getPartsLengthRatio(self):
        import math

        part1Length = math.sqrt((self.x1 - self.intersection[0]) ** 2 + (self.y1 - self.intersection[1]) ** 2)
        part2Length = math.sqrt((self.intersection[0] - self.x2) ** 2 + (self.intersection[1] - self.y2) ** 2)

        return max(part1Length, part2Length) / min(part1Length, part2Length)


    def __eq__(self, other):
        return (self.x1 == other.x1 and self.y1 == other.y1 and self.x2 == other.x2 and self.y2 == other.y2
              or self.x1 == other.x2 and self.y1 == other.y2 and self.x2 == other.x1 and self.y2 == other.y1)

    def __str__(self):
        return "(" + str(round(self.x1, 2)) + "," + str(round(self.y1,2)) + "),(" + str(round(self.x2, 2)) + "," + str(round(self.y2,2)) + ")"

    def __repr__(self):
        return str(self)




if __name__ == '__main__':
  try:
      my_path = RRT_Path_Plan_node()
  except BaseException:
      pass    
  except rospy.ROSInterruptException:
      pass
  rospy.spin()
