#! /usr/bin/env python3

from ast import Pass
from tkinter import Y
import rospy
from geometry_msgs.msg import Twist, PoseStamped, Quaternion
from carla_msgs.msg import CarlaEgoVehicleControl
import numpy as np
from collections import namedtuple
from utils.curve_generator import curve_generator
from nav_msgs.msg import Odometry, Path
from math import atan2, sin, cos, pi
from std_msgs.msg import Float32, String
from gazebo_msgs.msg import ModelStates
from visualization_msgs.msg import MarkerArray, Marker
import time
import numpy as np
import rospy
from std_msgs.msg import Float64, Bool
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry, Path 
from collections import namedtuple
from math import atan2
from mpc_path_tracking import mpc_path_tracking
import yaml
import os
from derived_object_msgs.msg import ObjectArray
import sys
from rosgraph_msgs.msg import Clock


car = namedtuple('car', 'G g cone_type wheelbase abs_speed abs_steer abs_acce abs_acce_steer')
obstacle = namedtuple('obstacle', 'A b cone_type ')

class mpc_core:
    def __init__(self, robot_id):
        self.robot_id = robot_id
        # ros parameter
        receding = rospy.get_param('receding', 3)
        sample_time = rospy.get_param('sample_time', 0.5)

        max_speed = rospy.get_param('max_speed', 15.0)
        ref_speed = rospy.get_param('ref_speed', 10.0)
        max_acce = rospy.get_param('max_acce', 4)
        max_acce_steer = rospy.get_param('max_acce_steer', 0.01)
        max_steer = rospy.get_param('max_steer', 0.3)
        wheelbase = rospy.get_param('wheelbase', 2.87)
        self.shape = rospy.get_param('shape', [4.7, 1.85, 2.87, 1.85])   # [length, width, wheelbase, wheelbase_w]

        cs = rospy.get_param('cs', [1, 1, 1])
        cs = np.diag(cs)
        cu = rospy.get_param('cu', 1)
        cst = rospy.get_param('cst', [1, 1, 1])
        cst = np.diag(cst)
        cut = rospy.get_param('cut', 1)

        # goals
        self.name = 'Town04'
        self.quat = None
        self.clock = 0.0
        self.start_time = 0.0
        self.nav_time = 0.0
        self.vel_to_throttle = max_speed  # convert from 0-15 to 0-1
        # mpc
        self.mpc_track = mpc_path_tracking(receding=receding, max_speed=max_speed, ref_speed=ref_speed, max_acce=max_acce, max_acce_steer=max_acce_steer, sample_time=sample_time, max_steer=max_steer, wheelbase=wheelbase, cs=np.diag([1, 1, 1]), cu=cu, cst=cst, cut=cut)
      
        rospy.init_node('agent_node_' + self.robot_id)
        rospy.Subscriber("/carla/" +  self.robot_id + "/odometry", Odometry, self.robot_state_callback)
        rospy.Subscriber("/clock", Clock, self.clock_callback)

        # goals
        txt_path = os.path.dirname(sys.path[0]) + '/planner/data/refpath_' + self.robot_id + '.txt'

        # Initialize lists to store data
        timestep, ref_x, ref_y, actual_x, actual_y = [], [], [], [], []

        with open(txt_path, 'r') as file:
            lines = file.readlines()
            
            # Skip header lines (first 3 lines)
            for line in lines[4:]:
                # Skip empty lines
                if line.strip():
                    values = line.split()
                    timestep.append(int(values[0]))
                    ref_x.append(float(values[1]))
                    ref_y.append(float(values[2]))
                    actual_x.append(float(values[3]))
                    actual_y.append(float(values[4]))

        # Convert to numpy arrays if needed
        timestep = np.array(timestep)
        ref_x = np.array(ref_x)
        ref_y = np.array(ref_y)
        actual_x = np.array(actual_x)
        actual_y = np.array(actual_y)

        print(f"Loaded {len(timestep)} data points")

        point_list = []

        for index, point in enumerate(actual_x):
            # transform into the carla coordinate system
            point_np_tmp = np.c_[[411.25 - actual_y[index], actual_x[index], 3.14/2]]
            point_list.append(point_np_tmp)

        start_point = point_list[0]
        cg = curve_generator(point_list=point_list, curve_style='dubins', min_radius=0.5)

        self.IL_flag = False
        self.first = True
        self.restart_flag = 0
        self.start_t = time.time()
        self.ref_path_list = cg.generate_curve(step_size=1.0)

        self.vel = Twist()
        self.output = CarlaEgoVehicleControl()
        self.robot_state = start_point.copy()
        self.x = 0
        self.y = 0
        self.z = 0
        self.angle = 0
        
        self.pub_mode = rospy.Publisher('mpc_state', Float32, queue_size=10)

        # rviz show markers
        self.pub_vel = rospy.Publisher('/carla/' + self.robot_id + '/vehicle_control_cmd', CarlaEgoVehicleControl, queue_size=10)     
        self.pub_path = rospy.Publisher('dubin_path_' + self.robot_id, Path, queue_size=10)
        self.pub_opt_path = rospy.Publisher('opt_path_' + self.robot_id, Path, queue_size=10)
        self.pub_marker_obs = rospy.Publisher('obs_marker', MarkerArray, queue_size=10)
        self.pub_braking = rospy.Publisher('braking_' + self.robot_id, Float64, queue_size=10)
        self.pub_flag = rospy.Publisher('flag_' + self.robot_id, Float64, queue_size=10)

        # path update
        self.path = self.generate_path(self.ref_path_list)

    def cal_vel(self, freq=20, **kwargs):
        
        rate = rospy.Rate(freq)
        while not rospy.is_shutdown():
            opt_vel, info, flag, _ = self.mpc_track.controller(self.robot_state, self.ref_path_list, iter_num=3) 
            
            if flag == True:
                self.vel.linear.x = 0
                self.vel.angular.z = 0
                self.output.throttle = 0
                self.output.steer = 0
                self.output.brake = 1
                self.nav_time = self.clock - self.start_time
                rospy.loginfo(self.robot_id + ' stops moving')
            else:
                self.vel.linear.x = round(opt_vel[0, 0], 2)
                self.vel.angular.z = round(opt_vel[1, 0], 2)
                self.output.throttle = round(opt_vel[0, 0], 2) / self.vel_to_throttle
                self.output.steer = - round(opt_vel[1, 0], 2)
                self.output.brake = 0

            opt_state = self.generate_opt_path(info)

            flag_msg = Float64()
            flag_msg.data = float(flag)
            self.pub_flag.publish(flag_msg)
            self.pub_path.publish(self.path)
            self.pub_opt_path.publish(opt_state)

            self.pub_vel.publish(self.output)
                
            rate.sleep()


    def robot_state_callback(self, data):
        
        x = data.pose.pose.position.x
        y = data.pose.pose.position.y
        z = data.pose.pose.position.z

        quat = data.pose.pose.orientation
        self.quat = quat

        raw = mpc_core.quat_to_yaw(quat)

        raw_degree = mpc_core.quat_to_yaw(quat)*180/pi

        self.x = x
        self.y = y
        self.z = z
        self.angle = raw_degree

        offset = self.shape[2] / 2
        
        self.robot_state[0] = x - offset * cos(raw)
        self.robot_state[1] = y - offset * sin(raw)
        self.robot_state[2] = raw


    def clock_callback(self, msg):
        nsecs_string = str(msg.clock.nsecs)
        nsecs_float = 0.001* float(nsecs_string[0:3])
        self.clock = msg.clock.secs + nsecs_float

    @staticmethod
    def generate_path(ref_path_list):
        path = Path()

        path.header.seq = 0
        path.header.stamp = rospy.get_rostime()
        path.header.frame_id = 'map'

        for i in range(len(ref_path_list)):
            ps = PoseStamped()
            ps.pose.position.x = ref_path_list[i][0, 0]
            ps.pose.position.y = ref_path_list[i][1, 0]
            ps.pose.position.z = 0

            path.poses.append(ps)

        return path

    def generate_opt_path(self, ref_path_list):
        path = Path()

        path.header.seq = 0
        path.header.stamp = rospy.get_rostime()
        path.header.frame_id = 'map'

        for i in range(len(ref_path_list[0])):

            raw = self.robot_state[2]
            offset = self.shape[2]

            ps = PoseStamped()
            ps.pose.position.x = ref_path_list[0][i] + offset * cos(raw)
            ps.pose.position.y = ref_path_list[1][i] + offset * sin(raw)
            ps.pose.position.z = self.z + 1.5
            ps.pose.orientation.w = 1

            path.poses.append(ps)

        return path


    @staticmethod
    def quat_to_yaw(quater):
         
        w = quater.w
        x = quater.x
        y = quater.y
        z = quater.z

        raw = atan2(2* ( w*z + x *y), 1 - 2 * (pow(z, 2) + pow(y, 2)))

        return raw

    @staticmethod
    def yaw_to_quat(yaw):
         
        w = cos(yaw/2)
        x = 0
        y = 0
        z = sin(yaw/2)

        quat = Quaternion(w=w, x=x, y=y, z=z)

        return quat
