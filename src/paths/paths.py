#!/usr/bin/env python

"""
Starter script for lab1. 
Author: Chris Correa
"""
import numpy as np
import math
import matplotlib.pyplot as plt
from bisect import bisect_left

from utils.utils import *

try:
    import rospy
    from moveit_msgs.msg import RobotTrajectory
    from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
    import tf
except:
    pass

class MotionPath:
    def __init__(self, limb, kin, total_time):
        """
        Parameters
        ----------
        limb : :obj:`baxter_interface.Limb` or :obj:`intera_interface.Limb`
        kin : :obj:`baxter_pykdl.baxter_kinematics` or :obj:`sawyer_pykdl.sawyer_kinematics`
            must be the same arm as limb
        total_time : float
            number of seconds you wish the trajectory to run for
        """
        self.limb = limb
        self.kin = kin
        self.total_time = total_time

    def target(self, time, points, dt, num_way):
        """
        Get the target point from points by linearly interpolating with time.
        """
        start_index = min(int(time / dt), num_way - 1)
        end_index = min(start_index + 1, num_way - 1)
        start_point = points[start_index]
        end_point = points[end_index]
        fraction = time % dt
        return linear_interpolation_two_points(start_point, end_point, fraction).reshape(3)

    def target_position(self, time):
        """
        Returns where the arm end effector should be at time t

        Parameters
        ----------
        time : float        

        Returns
        -------
        3x' :obj:`numpy.ndarray`
            desired x,y,z position in workspace coordinates of the end effector
        """
        pass

    def target_velocity(self, time):
        """
        Returns the arm's desired x,y,z velocity in workspace coordinates
        at time t

        Parameters
        ----------
        time : float

        Returns
        -------
        3x' :obj:`numpy.ndarray`
            desired velocity in workspace coordinates of the end effector
        """
        pass

    def target_acceleration(self, time):
        """
        Returns the arm's desired x,y,z acceleration in workspace coordinates
        at time t

        Parameters
        ----------
        time : float

        Returns
        -------
        3x' :obj:`numpy.ndarray`
            desired acceleration in workspace coordinates of the end effector
        """
        pass

    def plot_path(self):
        coords = ['x', 'y', 'z']
        plt.figure()
        # import pdb
        # pdb.set_trace()
        for i, (x_name, x) in enumerate(zip(['positions', 'velocities', 'accelerations'], [self.positions, self.velocities, self.accelerations])):
            plt.subplot(3, 1, i + 1)
            for j in range(3):
                plt.plot(np.arange(self.num_way) * self.dt, x[:,j].reshape(-1), label=x_name + ' ' + coords[j])
            plt.xlabel('time')
            plt.ylabel(x_name)
            # plt.legend()
            # plt.show()

        print "Close the plot window to continue"
        # plt.legend()
        plt.show()

    def plot(self, num=300):
        times = np.linspace(0, self.total_time, num=num)
        target_positions = np.vstack([self.target_position(t) for t in times])
        target_velocities = np.vstack([self.target_velocity(t) for t in times])

        plt.figure()
        plt.subplot(3,2,1)
        plt.plot(times, target_positions[:,0], label='Desired')
        plt.xlabel("Time (t)")
        plt.ylabel("X Position")

        plt.subplot(3,2,2)
        plt.plot(times, target_velocities[:,0], label='Desired')
        plt.xlabel("Time (t)")
        plt.ylabel("X Velocity")
            
        plt.subplot(3,2,3)
        plt.plot(times, target_positions[:,1], label='Desired')
        plt.xlabel("time (t)")
        plt.ylabel("Y Position")

        plt.subplot(3,2,4)
        plt.plot(times, target_velocities[:,1], label='Desired')
        plt.xlabel("Time (t)")
        plt.ylabel("Y Velocity")
            
        plt.subplot(3,2,5)
        plt.plot(times, target_positions[:,2], label='Desired')
        plt.xlabel("time (t)")
        plt.ylabel("Z Position")

        plt.subplot(3,2,6)
        plt.plot(times, target_velocities[:,2], label='Desired')
        plt.xlabel("Time (t)")
        plt.ylabel("Z Velocity")

        plt.show()

    def to_robot_trajectory(self, num_waypoints=300, jointspace=True):
        """
        Parameters
        ----------
        num_waypoints : float
            how many points in the :obj:`moveit_msgs.msg.RobotTrajectory`
        jointspace : bool
            What kind of trajectory.  Joint space points are 7x' and describe the
            angle of each arm.  Workspace points are 3x', and describe the x,y,z
            position of the end effector.  
        """
        traj = JointTrajectory()
        traj.joint_names = self.limb.joint_names()    
        points = []
        for t in np.linspace(0, self.total_time, num=num_waypoints):
            point = self.trajectory_point(t, jointspace)
            points.append(point)

        # We want to make a final point at the end of the trajectory so that the 
        # controller has time to converge to the final point.
        extra_point = self.trajectory_point(self.total_time, jointspace)
        extra_point.time_from_start = rospy.Duration.from_sec(self.total_time + 1)
        points.append(extra_point)

        traj.points = points
        traj.header.frame_id = 'base'
        robot_traj = RobotTrajectory()
        robot_traj.joint_trajectory = traj
        return robot_traj

    def trajectory_point(self, t, jointspace):
        """
        takes a discrete point in time, and puts the position, velocity, and
        acceleration into a ROS JointTrajectoryPoint() to be put into a 
        RobotTrajectory.  
        
        Parameters
        ----------
        t : float
        jointspace : bool
            What kind of trajectory.  Joint space points are 7x' and describe the
            angle of each arm.  Workspace points are 3x', and describe the x,y,z
            position of the end effector.  

        Returns
        -------
        :obj:`trajectory_msgs.msg.JointTrajectoryPoint`
        """
        point = JointTrajectoryPoint()
        delta_t = .01
        if jointspace:
            x_t, x_t_1, x_t_2 = None, None, None
            ik_attempts = 0
            theta_t = theta_t_1 = theta_t_2 = None
            while theta_t_2 is None:
                theta_t_2 = self.get_ik(self.target_position(t-2*delta_t))
            while theta_t_1 is None:
                theta_t_1 = self.get_ik(self.target_position(t-delta_t))
            while theta_t is None:
                theta_t   = self.get_ik(self.target_position(t))
            
            # we said you shouldn't simply take a finite difference when creating
            # the path, why do you think we're doing that here?
            point.positions = theta_t
            # print 'theta_t: {0}, {1}'.format(theta_t.shape, theta_t)
            # print 'theta_t_1: {0}, {1}'.format(theta_t_1.shape, theta_t_1)

            vel_t_prev = (theta_t_1 - theta_t_2) / delta_t
            vel_t_curr = (theta_t - theta_t_1) / delta_t
            vel_t_avg = (vel_t_prev + vel_t_curr) / 2.

            point.velocities = vel_t_avg #(theta_t - theta_t_1) / delta_t
            point.accelerations = (theta_t - 2*theta_t_1 + theta_t_2) / (2*delta_t)
            # if t >= 3.:
            #     import pdb; pdb.set_trace()
        else:
            point.positions = self.target_position(t)
            point.velocities = self.target_velocity(t)
            point.accelerations = self.target_acceleration(t)
        point.time_from_start = rospy.Duration.from_sec(t)
        return point

    def get_ik(self, x, max_ik_attempts=10):
        """
        gets ik
        
        Parameters
        ----------
        x : 3x' :obj:`numpy.ndarray`
            workspace position of the end effector
        max_ik_attempts : int
            number of attempts before short circuiting

        Returns
        -------
        7x' :obj:`numpy.ndarray`
            joint values to achieve the passed in workspace position
        """
        ik_attempts, theta = 0, None
        while theta is None and not rospy.is_shutdown():
            theta = self.kin.inverse_kinematics(
                position=x,
                orientation=[0, 1, 0, 0]
            )
            ik_attempts += 1
            if ik_attempts > max_ik_attempts:
                rospy.signal_shutdown(
                    'MAX IK ATTEMPTS EXCEEDED AT x(t)={}'.format(x)
                )
        return theta

class LinearPath(MotionPath):
    def __init__(self, limb, kin, total_time, goal_pos, num_way, start_pos=None):
        """
        Remember to call the constructor of MotionPath

        Parameters
        ----------
        goal_pos : 3x' :obj:`numpy.ndarray`
            Position of the goal in relation to the robot's base frame
        num_way : int
            number of waypoints
        """
        MotionPath.__init__(self, limb, kin, total_time)
        self.start_pos = start_pos
        self.goal_pos = goal_pos
        self.num_way = num_way
        self.base_frame = 'base'
        self.tool_frame = 'left_hand_camera' # 'left_gripper'
        self.plan_path()
        self.plot_path()

    def plan_path(self):
        if self.start_pos is None:
            r = rospy.Rate(200)
            listener = tf.TransformListener()
            while not listener.frameExists(self.base_frame) or not listener.frameExists(self.tool_frame):
                print 'Cannot find transformation from {0} frame to {1} frame, retrying'.format(self.base_frame, self.tool_frame)
                r.sleep()

            print 'Found transformation. Sleeping for a bit...'
            for i in range(200 * 1):
                r.sleep()

            t = listener.getLatestCommonTime(self.base_frame, self.tool_frame)
            # print 'Got latest common time between base and tool frames.'
            self.start_pos, _ = np.array(listener.lookupTransform(self.base_frame, self.tool_frame, t)) # self.start_pos = (x, y, z)
        
        print 'self.start_pos:'
        print self.start_pos

        self.dt = float(self.total_time) / self.num_way # time between waypoints
        distance = self.goal_pos - self.start_pos # self.positions[-1] - self.positions[0] # distance along (x, y, z) : shape (3,)
        amplitudes = distance / 2. # amplitude of sine function : shape (3,)
        vx = np.linspace(0, np.pi, self.num_way) # points at which to evaluate position, velocity, and acceleration functions : shape (self.num_way,)

        # positions
        # constant velocity
        # self.positions = np.squeeze(linear_interpolation(self.start_pos, self.goal_pos, self.num_way))
        # sin velocity
        self.positions = np.outer(np.cos(vx) - 1, -amplitudes) + np.vstack([self.start_pos for i in range(self.num_way)])

        # velocities
        # constant velocity
        # velocity = distance / self.total_time
        # self.velocities = np.squeeze(np.vstack([velocity for i in range(self.num_way)]))
        # sin velocity
        self.velocities = np.outer(np.sin(vx), amplitudes)
        
        # accelerations
        # zero acceleration
        # self.accelerations = np.squeeze(np.zeros((self.num_way, 3)))
        # cos acceleration
        self.accelerations = np.outer(np.cos(vx), amplitudes)

    def target_position(self, time):
        """
        Returns where the arm end effector should be at time t

        Parameters
        ----------
        time : float        
    
        Returns
        -------
        3x' :obj:`numpy.ndarray`
            desired x,y,z position in workspace coordinates of the end effector
        """
        return self.target(time, self.positions, self.dt, self.num_way)
        

    def target_velocity(self, time):
        """
        Returns the arm's desired x,y,z velocity in workspace coordinates
        at time t.  You should NOT simply take a finite difference of
        self.target_position()

        Parameters
        ----------
        time : float

        Returns
        -------
        3x' :obj:`numpy.ndarray`
            desired velocity in workspace coordinates of the end effector
        """
        return self.target(time, self.velocities, self.dt, self.num_way)

    def target_acceleration(self, time):
        """
        Returns the arm's desired x,y,z acceleration in workspace coordinates
        at time t.  You should NOT simply take a finite difference of
        self.target_velocity()

        Parameters
        ----------
        time : float

        Returns
        -------
        3x' :obj:`numpy.ndarray`
            desired acceleration in workspace coordinates of the end effector
        """
        return self.target(time, self.accelerations, self.dt, self.num_way)


class CircularPath(MotionPath):
    def __init__(self, limb, kin, total_time, center_pos, num_way, radius, z_height=0.1):
        """
        Remember to call the constructor of MotionPath

        Parameters
        ----------
        ????? You're going to have to fill these in how you see fit
        """
        MotionPath.__init__(self, limb, kin, total_time)
        self.center_pos = center_pos
        self.num_way = num_way
        self.radius = radius
        self.z_height = z_height
        self.base_frame = 'base'
        self.tool_frame = 'left_gripper'
        self.plan_path()
        self.plot_path()


    def plan_path(self):
        r = rospy.Rate(200)
        listener = tf.TransformListener()
        while not listener.frameExists(self.base_frame) or not listener.frameExists(self.tool_frame):
            print 'Cannot find transformation from {0} frame to {1} frame, retrying'.format(self.base_frame, self.tool_frame)
            r.sleep()

        print 'Found transformation. Sleeping for a bit...'
        for i in range(200 * 1):
            r.sleep()

        t = listener.getLatestCommonTime(self.base_frame, self.tool_frame)
        # print 'Got latest common time between base and tool frames.'
        tool_pos, _ = listener.lookupTransform(self.base_frame, self.tool_frame, t) # tool_pos = (x, y, z)
        print 'tool_pos:'
        print tool_pos

        self.dt = float(self.total_time) / self.num_way # time between waypoints

        thetas = np.linspace(0, 2*np.pi, self.num_way)
        x = self.radius * np.cos(thetas) + self.center_pos[0]
        y = self.radius * np.sin(thetas) + self.center_pos[1]
        z = (self.z_height + self.center_pos[2]) * np.ones(self.num_way)

        # positions
        # constant velocity
        self.positions = np.vstack([x, y, z]).T
        print 'positions: {0}'.format(self.positions.shape)

        # velocities
        distance = 2 * np.pi * self.radius # distance along (x, y, z) : shape (3,)
        
        # constant angular velocity
        absolute_velocities = distance / self.total_time

        # sin angular velocity
        # amplitude = distance / 2. # amplitude of sine function : shape (3,)
        # vx = np.linspace(0, np.pi, self.num_way) # points at which to evaluate velocity (sin) : shape (self.num_way,)
        # absolute_velocities = np.sin(vx) * amplitude

        x_dot = -1 * np.multiply(np.sin(thetas), absolute_velocities)
        y_dot = np.multiply(np.cos(thetas), absolute_velocities)
        z_dot = np.zeros(self.num_way)

        self.velocities = np.vstack([x_dot, y_dot, z_dot]).T
        print 'velocities: {0}'.format(self.velocities.shape)
        
        # accelerations
        # constant angular velocity
        x_acc = -1 * np.multiply(np.cos(thetas), absolute_velocities)
        y_acc = -1 * np.multiply(np.sin(thetas), absolute_velocities)
        z_acc = np.zeros(self.num_way)

        self.accelerations = np.vstack([x_acc, y_acc, z_acc]).T
        print 'accelerations: {0}'.format(self.accelerations.shape)

    def target_position(self, time):
        """
        Returns where the arm end effector should be at time t

        Parameters
        ----------
        time : float        

        Returns
        -------
        3x' :obj:`numpy.ndarray`
           desired x,y,z position in workspace coordinates of the end effector
        """
        return self.target(time, self.positions, self.dt, self.num_way)

    def target_velocity(self, time):
        """
        Returns the arm's desired velocity in workspace coordinates
        at time t.  You should NOT simply take a finite difference of
        self.target_position()

        Parameters
        ----------
        time : float

        Returns
        -------
        3x' :obj:`numpy.ndarray`
           desired x,y,z velocity in workspace coordinates of the end effector
        """
        return self.target(time, self.velocities, self.dt, self.num_way)

    def target_acceleration(self, time):
        """
        Returns the arm's desired x,y,z acceleration in workspace coordinates
        at time t.  You should NOT simply take a finite difference of
        self.target_velocity()

        Parameters
        ----------
        time : float

        Returns
        -------
        3x' :obj:`numpy.ndarray`
           desired acceleration in workspace coordinates of the end effector
        """
        return self.target(time, self.accelerations, self.dt, self.num_way)


class MultiplePaths(MotionPath):
    """
    Remember to call the constructor of MotionPath
    
    You can implement multiple paths a couple ways.  The way I chose when I took
    the class was to create several different paths and pass those into the 
    MultiplePaths object, which would determine when to go onto the next path.
    """
    def __init__(self, limb, kin, paths):
        self.paths = paths
        self.path_times = np.cumsum([path.total_time for path in self.paths])
        MotionPath.__init__(self, limb, kin, self.path_times[-1])

    def get_current_path(self, time):
        path_index = bisect_left(self.path_times, time)
        if path_index != 0:
            path_time = time - self.path_times[path_index - 1]
        else:
            path_time = time
        return self.paths[path_index], path_time

    def target_position(self, time):
        """
        Returns where the arm end effector should be at time t

        Parameters
        ----------
        time : float        

        Returns
        -------
        3x' :obj:`numpy.ndarray`
            desired position in workspace coordinates of the end effector
        """
        path, path_time = self.get_current_path(time)
        return path.target_position(path_time)

    def target_velocity(self, time):
        """
        Returns the arm's desired velocity in workspace coordinates
        at time t

        Parameters
        ----------
        time : float

        Returns
        -------
        3x' :obj:`numpy.ndarray`
            desired velocity in workspace coordinates of the end effector
        """
        path, path_time = self.get_current_path(time)
        return path.target_velocity(path_time)

    def target_acceleration(self, time):
        """
        Returns the arm's desired acceleration in workspace coordinates
        at time t

        Parameters
        ----------
        time : float

        Returns
        -------
        3x' :obj:`numpy.ndarray`
            desired acceleration in workspace coordinates of the end effector
        """
        path, path_time = self.get_current_path(time)
        return path.target_acceleration(path_time)
