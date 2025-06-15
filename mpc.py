#!/usr/bin/env python3

from os import path
from nav_msgs.msg import Path
import matplotlib.pyplot as plt
import cvxpy
import math
import numpy as np
import rospy
import sys
import pandas as pd
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray 
from ackermann_msgs.msg import AckermannDriveStamped






sys.path.append('/home/fadi/nasserr/aamfsd_cv_2022/src/control/src/CubicSpline/')

try:
    import cubic_spline_planner
except:
    raise




NX = 4  # x = x, y, v, yaw
NU = 2  # a = [accel, steer]
T = 5  # horizon length

# mpc parameters
R = np.diag([0.01, 0.01])  # input cost matrix
Rd = np.diag([0.01, 1.0])  # input difference cost matrix
Q = np.diag([1.0, 1.0, 0.5, 0.5])  # state cost matrix
Qf = Q  # state final matrix
GOAL_DIS = 1.5  # goal distance
STOP_SPEED = 0.5 / 3.6  # stop speed
MAX_TIME = 500.0  # max simulation time

# iterative paramter
MAX_ITER = 3  # Max iteration
DU_TH = 0.1  # iteration finish param

TARGET_SPEED = 10.0 / 3.6  # [m/s] target speed
N_IND_SEARCH = 10  # Search index number

DT = 0.2  # [s] time tick

# Vehicle parameters                   # TO BE CHANGED
LENGTH = 4.5  # [m]
WIDTH = 2.0  # [m]
BACKTOWHEEL = 1.0  # [m]
WHEEL_LEN = 0.3  # [m]
WHEEL_WIDTH = 0.2  # [m]
TREAD = 0.7  # [m]
WB = 2.5  # [m]

MAX_STEER = np.deg2rad(45.0)  # maximum steering angle [rad]
MAX_DSTEER = np.deg2rad(30.0)  # maximum steering speed [rad/s]
MAX_SPEED = 55.0 / 3.6  # maximum speed [m/s]
MIN_SPEED = -20.0 / 3.6  # minimum speed [m/s]
MAX_ACCEL = 1.0  # maximum accel [m/ss]

show_animation = True


class State:
    """
    vehicle state class
    """
    
    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v
        self.predelta = None
        self.speed_profile = 0.0




class Function:

    def __init__(self):
        self.dl = 1.0
        # self.refrence_of_x = []
        # self.refrence_of_y = []
        # self.refrence_of_yaw = []

        self.waypoints_of_x = []
        self.waypoints_of_y = []



        self.Vx=0.0
        self.initial_state = State(x=0.0, y=0.0, yaw=0.0, v=0.0)

        rospy.init_node("mpc_controller", anonymous = True)
        rospy.Subscriber("/visual/waypoints", MarkerArray, self.waypoints_callback) 
        rospy.Subscriber('/car_pose', Path,self.refrence_callback)
        rospy.Subscriber("/robot_control/command",AckermannDriveStamped,self.control_callback)
        self.robot_control_pub = rospy.Publisher("/robot_control/command",AckermannDriveStamped,queue_size=0)

    def waypoints_callback(self,waypoints_msg):    #callback 0
        

        #waypoints_of_x = []
        #waypoints_of_y = []

       # print(len(waypoints_msg.markers[0].points))
        print("ana fe wayback")
        for point in waypoints_msg.markers[0].points:
            
            
            self.waypoints_of_x.append(point.x)
            self.waypoints_of_y.append(point.y)
            if (len(self.waypoints_of_x)<=500 and len(self.waypoints_of_y)<=500):
                continue
            else:
                break
            
            

        ##self.waypoints_of_x=[]
        ##self.waypoints_of_y=[]
        print(len(self.waypoints_of_x),len(self.waypoints_of_y))
        print("length")
        #cx , cy, cyaw, ck,  these varibles are calculated from the function of cubic spline
        cx, cy, cyaw, ck, s = cubic_spline_planner.calc_spline_course(self.waypoints_of_x,self.waypoints_of_y, ds=self.dl)
        print("cubic")
        #sp is the variable of array lel speed profile 
        sp = self.calc_speed_profile(cx, cy, cyaw, TARGET_SPEED)
        print("speed")
        t, x, y, yaw, v, d, a = self.do_simulation(cx, cy, cyaw, ck, sp, self.dl, self.initial_state)
        print("do simulation")

        control_msg = AckermannDriveStamped()

        control_msg.drive.steering_angle = d[-1] ##variable steering to be changed to variable 
        control_msg.drive.speed = a[-1]  ##variable speed to be changed done
        print("alo")
        self.robot_control_pub.publish(control_msg)
        

    def refrence_callback(self,Path):   #first ccallback
        
        #print("nav callback ana fel localizatiooooonnnnnn")

        self.refrence_of_x = []
        self.refrence_of_y = []
        self.refrence_of_yaw = []


        for point in Path.poses:
            self.refrence_of_x.append(point.pose.position.x)
            self.refrence_of_y.append(point.pose.position.y)
            self.refrence_of_yaw.append(point.pose.orientation.w)
            
        # print("Ana f reference callback")
        # print(len(self.waypoints_of_x),len(self.waypoints_of_y))

        '''    
        #print(len(self.waypoints_of_x),len(self.waypoints_of_y))
        #cx , cy, cyaw, ck,  these varibles are calculated from the function of cubic spline
        cx, cy, cyaw, ck, s = cubic_spline_planner.calc_spline_course(self.waypoints_of_x,self.waypoints_of_y, ds=self.dl)
        #sp is the variable of array lel speed profile 
        sp = self.calc_speed_profile(cx, cy, cyaw, TARGET_SPEED)

        t, x, y, yaw, v, d, a = self.do_simulation(cx, cy, cyaw, ck, sp, self.dl, self.initial_state)


        control_msg = AckermannDriveStamped()

        control_msg.drive.steering_angle = d[-1] ##variable steering to be changed to variable 
        control_msg.drive.speed = a[-1]  ##variable speed to be changed done
    
        self.robot_control_pub.publish(control_msg) '''

        

        # t, x, y, yaw, v, d, a = self.do_simulation(cx, cy, cyaw, ck, sp, self.dl, self.initial_state)


        # control_msg = AckermannDriveStamped()

        # control_msg.drive.steering_angle = d[-1] ##variable steering to be changed to variable 
        # control_msg.drive.speed = a[-1]  ##variable speed to be changed done
    
        # self.robot_control_pub.publish(control_msg) 



        

    def control_callback(self,control_msg):
        self.Vx = control_msg.drive.speed
        # self.robot_control_pub.publish(speed_profile)     ############################

    def pi_2_pi(self,angle):
        while(angle > math.pi):
            angle = angle - 2.0 * math.pi

        while(angle < -math.pi):
            angle = angle + 2.0 * math.pi

        return angle


    def get_linear_model_matrix(self,v, phi, delta):

        A = np.zeros((NX, NX))
        A[0, 0] = 1.0
        A[1, 1] = 1.0
        A[2, 2] = 1.0
        A[3, 3] = 1.0
        A[0, 2] = DT * math.cos(phi)
        A[0, 3] = - DT * v * math.sin(phi)
        A[1, 2] = DT * math.sin(phi)
        A[1, 3] = DT * v * math.cos(phi)
        A[3, 2] = DT * math.tan(delta) / WB

        B = np.zeros((NX, NU))
        B[2, 0] = DT
        B[3, 1] = DT * v / (WB * math.cos(delta) ** 2)

        C = np.zeros(NX)
        C[0] = DT * v * math.sin(phi) * phi
        C[1] = - DT * v * math.cos(phi) * phi
        C[3] = - DT * v * delta / (WB * math.cos(delta) ** 2)

        return A, B, C


    

    def update_state(self,state, a, delta):

        # input check
        if delta >= MAX_STEER:
            delta = MAX_STEER
        elif delta <= -MAX_STEER:
            delta = -MAX_STEER

        state.x = state.x + state.v * math.cos(state.yaw) * DT
        state.y = state.y + state.v * math.sin(state.yaw) * DT
        state.yaw = state.yaw + state.v / WB * math.tan(delta) * DT
        state.v = state.v + a * DT

        if state. v > MAX_SPEED:
            state.v = MAX_SPEED
        elif state. v < MIN_SPEED:
            state.v = MIN_SPEED

        return state


    def get_nparray_from_matrix(self,x):
        return np.array(x).flatten()


    def calc_nearest_index(self,state, cx, cy, cyaw, pind):

        dx = [state.x - icx for icx in cx[pind:(pind + N_IND_SEARCH)]]
        dy = [state.y - icy for icy in cy[pind:(pind + N_IND_SEARCH)]]

        d = [idx ** 2 + idy ** 2 for (idx, idy) in zip(dx, dy)]

        mind = min(d)

        ind = d.index(mind) + pind

        mind = math.sqrt(mind)

        dxl = cx[ind] - state.x
        dyl = cy[ind] - state.y

        angle = self.pi_2_pi(cyaw[ind] - math.atan2(dyl, dxl))
        if angle < 0:
            mind *= -1

        return ind, mind


    def predict_motion(self,x0, oa, od, xref):
        xbar = xref * 0.0
        for i, _ in enumerate(x0):
            xbar[i, 0] = x0[i]

        state = State(x=x0[0], y=x0[1], yaw=x0[3], v=x0[2])
        for (ai, di, i) in zip(oa, od, range(1, T + 1)):
            state = self.update_state(state, ai, di)
            xbar[0, i] = state.x
            xbar[1, i] = state.y
            xbar[2, i] = state.v
            xbar[3, i] = state.yaw

        return xbar


    def iterative_linear_mpc_control(self,xref, x0, dref, oa, od):
        """
        MPC contorl with updating operational point iteraitvely
        """

        if oa is None or od is None:
             oa = [0.0] * T
             od = [0.0] * T

            # oa_d = [0.0]
            # oa = [np.dot(value,T) for value in oa_d]

            # od_d = [0.0]
            # od = [np.dot(value,T) for value in od_d]

            # oa_d = [0.0]
            # oa = np.array(oa_d)
            # oa = oa * T

            # od_d = [0.0]
            # od = np.array(od_d)
            # od = od * T
            # print(od)

        for i in range(MAX_ITER):
            xbar = self.predict_motion(x0, oa, od, xref)
            poa, pod = oa[:], od[:]
            oa, od, ox, oy, oyaw, ov = self.linear_mpc_control(xref, xbar, x0, dref)
            du = sum(abs(oa - poa)) + sum(abs(od - pod))  # calc u change value
            if du <= DU_TH:
                break
        else:
            print("Iterative is max iter")

        return oa, od, ox, oy, oyaw, ov


    def linear_mpc_control(self,xref, xbar, x0, dref):
        """
        linear mpc control

        xref: reference point
        xbar: operational point
        x0: initial state
        dref: reference steer angle
        """

        x = cvxpy.Variable((NX, T + 1))
        u = cvxpy.Variable((NU, T))

        cost = 0.0
        constraints = []

        for t in range(T):
            cost += cvxpy.quad_form(u[:, t], R)

            if t != 0:
                cost += cvxpy.quad_form(xref[:, t] - x[:, t], Q)

            A, B, C = self.get_linear_model_matrix(
                xbar[2, t], xbar[3, t], dref[0, t])
            constraints += [x[:, t + 1] == A * x[:, t] + B * u[:, t] + C]

            if t < (T - 1):
                cost += cvxpy.quad_form(u[:, t + 1] - u[:, t], Rd)
                constraints += [cvxpy.abs(u[1, t + 1] - u[1, t]) <=
                                MAX_DSTEER * DT]

        cost += cvxpy.quad_form(xref[:, T] - x[:, T], Qf)

        constraints += [x[:, 0] == x0]
        constraints += [x[2, :] <= MAX_SPEED]
        constraints += [x[2, :] >= MIN_SPEED]
        constraints += [cvxpy.abs(u[0, :]) <= MAX_ACCEL]
        constraints += [cvxpy.abs(u[1, :]) <= MAX_STEER]

        prob = cvxpy.Problem(cvxpy.Minimize(cost), constraints)
        prob.solve(solver=cvxpy.ECOS, verbose=False)

        if prob.status == cvxpy.OPTIMAL or prob.status == cvxpy.OPTIMAL_INACCURATE:
            ox = self.get_nparray_from_matrix(x.value[0, :])
            oy = self.get_nparray_from_matrix(x.value[1, :])
            ov = self.get_nparray_from_matrix(x.value[2, :])
            oyaw = self.get_nparray_from_matrix(x.value[3, :])
            oa = self.get_nparray_from_matrix(u.value[0, :])
            odelta = self.get_nparray_from_matrix(u.value[1, :])

        else:
            print("Error: Cannot solve mpc..")
            oa, odelta, ox, oy, oyaw, ov = None, None, None, None, None, None

        return oa, odelta, ox, oy, oyaw, ov


    def calc_ref_trajectory(self,state, cx, cy, cyaw, ck, sp, dl, pind):
        xref = np.zeros((NX, T + 1))
        dref = np.zeros((1, T + 1))
        ncourse = len(cx)

        ind, _ = self.calc_nearest_index(state, cx, cy, cyaw, pind)

        if pind >= ind:
            ind = pind

        xref[0, 0] = cx[ind]
        xref[1, 0] = cy[ind]
        xref[2, 0] = sp[ind]
        xref[3, 0] = cyaw[ind]
        dref[0, 0] = 0.0  # steer operational point should be 0

        travel = 0.0

        for i in range(T + 1):
            travel += abs(state.v) * DT
            dind = int(round(travel / dl))

            if (ind + dind) < ncourse:
                xref[0, i] = cx[ind + dind]
                xref[1, i] = cy[ind + dind]
                xref[2, i] = sp[ind + dind]
                xref[3, i] = cyaw[ind + dind]
                dref[0, i] = 0.0
            else:
                xref[0, i] = cx[ncourse - 1]
                xref[1, i] = cy[ncourse - 1]
                xref[2, i] = sp[ncourse - 1]
                xref[3, i] = cyaw[ncourse - 1]
                dref[0, i] = 0.0

        return xref, ind, dref


    def check_goal(self,state, goal, tind, nind):

        # check goal
        dx = state.x - goal[0]
        dy = state.y - goal[1]
        d = math.hypot(dx, dy)

        isgoal = (d <= GOAL_DIS)

        if abs(tind - nind) >= 5:
            isgoal = False

        isstop = (abs(state.v) <= STOP_SPEED)

        if isgoal and isstop:
            return True

        return False

    
    def do_simulation(self,cx, cy, cyaw, ck, sp, dl, initial_state):
        """
        Simulation

        cx: course x position list
        cy: course y position list
        cy: course yaw position list
        ck: course curvature list
        sp: speed profile
        dl: course tick [m]

        """

        goal = [cx[-1], cy[-1]]

        state = initial_state

        # initial yaw compensation
        if state.yaw - cyaw[0] >= math.pi:
            state.yaw -= math.pi * 2.0
        elif state.yaw - cyaw[0] <= -math.pi:
            state.yaw += math.pi * 2.0

        time = 0.0
        x = [state.x]
        y = [state.y]
        yaw = [state.yaw]
        v = [state.v]
        t = [0.0]
        d = [0.0]
        a = [0.0]
        target_ind, _ = self.calc_nearest_index(state, cx, cy, cyaw, 0)

        odelta, oa = None, None

        cyaw = self.smooth_yaw(cyaw)

        while MAX_TIME >= time:
            xref, target_ind, dref = self.calc_ref_trajectory(
                state, cx, cy, cyaw, ck, sp, dl, target_ind)

            x0 = [state.x, state.y, state.v, state.yaw]  # current state

            oa, odelta, ox, oy, oyaw, ov = self.iterative_linear_mpc_control(
                xref, x0, dref, oa, odelta)

            if odelta is not None:
                di, ai = odelta[0], oa[0]

            state = self.update_state(state, ai, di)
            time = time + DT

            x.append(state.x)
            y.append(state.y)
            yaw.append(state.yaw)
            v.append(state.v)
            t.append(time)
            d.append(di)
            a.append(ai)

            if self.check_goal(state, goal, target_ind, len(cx)):
                print("Goal")
                break
            

        return t, x, y, yaw, v, d, a


    def calc_speed_profile(self,cx, cy, cyaw, target_speed):

        speed_profile = [target_speed] * len(cx)
        direction = 1.0  # forward

        # Set stop point
        for i in range(len(cx) - 1):
            dx = cx[i + 1] - cx[i]
            dy = cy[i + 1] - cy[i]

            move_direction = math.atan2(dy, dx)

            if dx != 0.0 and dy != 0.0:
                dangle = abs(self.pi_2_pi(move_direction - cyaw[i]))
                if dangle >= math.pi / 4.0:
                    direction = -1.0
                else:
                    direction = 1.0

            if direction != 1.0:
                speed_profile[i] = - target_speed
            else:
                speed_profile[i] = target_speed

        speed_profile[-1] = 0.0            #########

        return speed_profile


    def smooth_yaw(self,yaw):

        for i in range(len(yaw) - 1):
            dyaw = yaw[i + 1] - yaw[i]

            while dyaw >= math.pi / 2.0:
                yaw[i + 1] -= math.pi * 2.0
                dyaw = yaw[i + 1] - yaw[i]

            while dyaw <= -math.pi / 2.0:
                yaw[i + 1] += math.pi * 2.0
                dyaw = yaw[i + 1] - yaw[i]

        return yaw








def main():
    print(__file__ + " start!!")

    try:
        control = Function()
    except rospy.ROSInterruptException:
      pass
    rospy.spin()
    #print(len(waypoints_of_x)+ len(waypoints_of_y))
      # course tick
    '''
    #cx , cy, cyaw, ck,  these varibles are calculated from the function of cubic spline
    cx, cy, cyaw, ck, s = cubic_spline_planner.calc_spline_course(waypoints_of_x,waypoints_of_y, ds=dl)
    
    #sp is the variable of array lel speed profile 
    sp = calc_speed_profile(cx, cy, cyaw, TARGET_SPEED)

    #
    

    t, x, y, yaw, v, d, a = do_simulation(cx, cy, cyaw, ck, sp, dl, initial_state)



    
    control_msg = AckermannDriveStamped()

    control_msg.drive.steering_angle = d[-1] ##variable steering to be changed to variable 
    control_msg.drive.speed = a[-1]  ##variable speed to be changed done
    
    robot_control_pub.publish(control_msg)
    rospy.spin()
        '''
    


if __name__ == '__main__':
    main()
    # main2()
