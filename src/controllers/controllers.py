#!/usr/bin/env python

"""
Starter script for lab1. 
Author: Chris Correa, Valmik Prabhu
"""

# Python imports
import sys
import numpy as np
import itertools
import matplotlib.pyplot as plt

# Lab imports
from utils.utils import *

# ROS imports
try:
    import tf
    import rospy
    import baxter_interface
    import intera_interface
    from geometry_msgs.msg import PoseStamped
    from moveit_msgs.msg import RobotTrajectory
except:
    pass

NUM_JOINTS = 7

class Controller:

    def __init__(self, limb, kin):
        """
        Constructor for the superclass. All subclasses should call the superconstructor

        Parameters
        ----------
        limb : :obj:`baxter_interface.Limb` or :obj:`intera_interface.Limb`
        kin : :obj:`baxter_pykdl.baxter_kinematics` or :obj:`sawyer_pykdl.sawyer_kinematics`
            must be the same arm as limb
        """

        # Run the shutdown function when the ros node is shutdown
        rospy.on_shutdown(self.shutdown)
        self._limb = limb
        self._kin = kin

    def step_control(self, target_position, target_velocity, target_acceleration):
        """
        makes a call to the robot to move according to it's current position and the desired position 
        according to the input path and the current time. Each Controller below extends this 
        class, and implements this accordingly.  

        Parameters
        ----------
        target_position : 7x' or 6x' :obj:`numpy.ndarray` 
            desired positions
        target_velocity : 7x' or 6x' :obj:`numpy.ndarray` 
            desired velocities
        target_acceleration : 7x' or 6x' :obj:`numpy.ndarray` 
            desired accelerations
        """
        pass

    def interpolate_path(self, path, t, current_index = 0):
        """
        interpolates over a :obj:`moveit_msgs.msg.RobotTrajectory` to produce desired
        positions, velocities, and accelerations at a specified time

        Parameters
        ----------
        path : :obj:`moveit_msgs.msg.RobotTrajectory`
        t : float
            the time from start
        current_index : int
            waypoint index from which to start search

        Returns
        -------
        target_position : 7x' or 6x' :obj:`numpy.ndarray` 
            desired positions
        target_velocity : 7x' or 6x' :obj:`numpy.ndarray` 
            desired velocities
        target_acceleration : 7x' or 6x' :obj:`numpy.ndarray` 
            desired accelerations
        current_index : int
            waypoint index at which search was terminated 
        """

        # a very small number (should be much smaller than rate)
        epsilon = 0.0001

        max_index = len(path.joint_trajectory.points)-1

        # If the time at current index is greater than the current time,
        # start looking from the beginning
        if (path.joint_trajectory.points[current_index].time_from_start.to_sec() > t):
            current_index = 0

        # Iterate forwards so that you're using the latest time
        while (
            not rospy.is_shutdown() and \
            current_index < max_index and \
            path.joint_trajectory.points[current_index+1].time_from_start.to_sec() < t+epsilon
        ):
            current_index = current_index+1

        # Perform the interpolation
        if current_index < max_index:
            time_low = path.joint_trajectory.points[current_index].time_from_start.to_sec()
            time_high = path.joint_trajectory.points[current_index+1].time_from_start.to_sec()

            target_position_low = np.array(
                path.joint_trajectory.points[current_index].positions
            )
            target_velocity_low = np.array(
                path.joint_trajectory.points[current_index].velocities
            )
            target_acceleration_low = np.array(
                path.joint_trajectory.points[current_index].accelerations
            )

            target_position_high = np.array(
                path.joint_trajectory.points[current_index+1].positions
            )
            target_velocity_high = np.array(
                path.joint_trajectory.points[current_index+1].velocities
            )
            target_acceleration_high = np.array(
                path.joint_trajectory.points[current_index+1].accelerations
            )

            target_position = target_position_low + \
                (t - time_low)/(time_high - time_low)*(target_position_high - target_position_low)
            target_velocity = target_velocity_low + \
                (t - time_low)/(time_high - time_low)*(target_velocity_high - target_velocity_low)
            target_acceleration = target_acceleration_low + \
                (t - time_low)/(time_high - time_low)*(target_acceleration_high - target_acceleration_low)

        # If you're at the last waypoint, no interpolation is needed
        else:
            target_position = np.array(path.joint_trajectory.points[current_index].positions)
            target_velocity = np.array(path.joint_trajectory.points[current_index].velocities)
            target_acceleration = np.array(path.joint_trajectory.points[current_index].velocities)

        return (target_position, target_velocity, target_acceleration, current_index)


    def shutdown(self):
        """
        Code to run on shutdown. This is good practice for safety
        """
        rospy.loginfo("Stopping Controller")

        # Set velocities to zero
        self.stop_moving()
        rospy.sleep(0.1)

    def stop_moving(self):
        """
        Set robot joint velocities to zero
        """
        zero_vel_dict = joint_array_to_dict(np.zeros(NUM_JOINTS), self._limb)
        self._limb.set_joint_velocities(zero_vel_dict)



    def plot_results(
        self,
        times,
        actual_positions, 
        actual_velocities, 
        target_positions, 
        target_velocities
    ):
        """
        Plots results.
        If the path is in joint space, it will plot both workspace and jointspace plots.
        Otherwise it'll plot only workspace

        Inputs:
        times : nx' :obj:`numpy.ndarray`
        actual_positions : nx7 or nx6 :obj:`numpy.ndarray`
            actual joint positions for each time in times
        actual_velocities: nx7 or nx6 :obj:`numpy.ndarray`
            actual joint velocities for each time in times
        target_positions: nx7 or nx6 :obj:`numpy.ndarray`
            target joint or workspace positions for each time in times
        target_velocities: nx7 or nx6 :obj:`numpy.ndarray`
            target joint or workspace velocities for each time in times
        """

        # Make everything an ndarray
        times = np.array(times)
        actual_positions = np.array(actual_positions)
        actual_velocities = np.array(actual_velocities)
        target_positions = np.array(target_positions)
        target_velocities = np.array(target_velocities)

        # Find the actual workspace positions and velocities
        actual_workspace_positions = np.zeros((len(times), 3))
        actual_workspace_velocities = np.zeros((len(times), 3))

        for i in range(len(times)):
            positions_dict = joint_array_to_dict(actual_positions[i], self._limb)
            actual_workspace_positions[i] = \
                self._kin.forward_position_kinematics(joint_values=positions_dict)[:3]
            actual_workspace_velocities[i] = \
                self._kin.jacobian(joint_values=positions_dict)[:3].dot(actual_velocities[i])
        # check if joint space
        if target_positions.shape[1] > 3:
            # it's joint space

            target_workspace_positions = np.zeros((len(times), 3))
            target_workspace_velocities = np.zeros((len(times), 3))

            for i in range(len(times)):
                positions_dict = joint_array_to_dict(target_positions[i], self._limb)
                target_workspace_positions[i] = \
                    self._kin.forward_position_kinematics(joint_values=positions_dict)[:3]
                target_workspace_velocities[i] = \
                    self._kin.jacobian(joint_values=positions_dict)[:3].dot(target_velocities[i])

            # Plot joint space
            plt.figure()
            # print len(times), actual_positions.shape()
            joint_num = len(self._limb.joint_names())
            for joint in range(joint_num):
                plt.subplot(joint_num,2,2*joint+1)
                plt.plot(times, actual_positions[:,joint], label='Actual')
                plt.plot(times, target_positions[:,joint], label='Desired')
                plt.xlabel("Time (t)")
                plt.ylabel("Joint " + str(joint) + " Position Error")

                plt.subplot(joint_num,2,2*joint+2)
                plt.plot(times, actual_velocities[:,joint], label='Actual')
                plt.plot(times, target_velocities[:,joint], label='Desired')
                plt.xlabel("Time (t)")
                plt.ylabel("Joint " + str(joint) + " Velocity Error")

            print "Close the plot window to continue"
            plt.show()

        else:
            # it's workspace
            target_workspace_positions = target_positions
            target_workspace_velocities = target_velocities

        plt.figure()
        workspace_joints = ('X', 'Y', 'Z')
        joint_num = len(workspace_joints)
        for joint in range(joint_num):
            plt.subplot(joint_num,2,2*joint+1)
            plt.plot(times, actual_workspace_positions[:,joint], label='Actual')
            plt.plot(times, target_workspace_positions[:,joint], label='Desired')
            plt.xlabel("Time (t)")
            plt.ylabel(workspace_joints[joint] + " Position Error")

            plt.subplot(joint_num,2,2*joint+2)
            plt.plot(times, actual_velocities[:,joint], label='Actual')
            plt.plot(times, target_velocities[:,joint], label='Desired')
            plt.xlabel("Time (t)")
            plt.ylabel(workspace_joints[joint] + " Velocity Error")

        plt.legend()
        print "Close the plot window to continue"
        plt.show()




    def execute_path(self, path, rate=200, timeout=None, log=False):
        """
        takes in a path and moves the baxter in order to follow the path.  

        Parameters
        ----------
        path : :obj:`moveit_msgs.msg.RobotTrajectory`
        rate : int
            This specifies how many ms between loops.  It is important to
            use a rate and not a regular while loop because you want the
            loop to refresh at a constant rate, otherwise you would have to
            tune your PD parameters if the loop runs slower / faster
        timeout : int
            If you want the controller to terminate after a certain number
            of seconds, specify a timeout in seconds.
        log : bool
            whether or not to display a plot of the controller performance

        Returns
        -------
        bool
            whether the controller completes the path or not
        """

        # For plotting
        if log:
            times = list()
            actual_positions = list()
            actual_velocities = list()
            target_positions = list()
            target_velocities = list()

        # For interpolation
        max_index = len(path.joint_trajectory.points)-1
        current_index = 0

        # For timing
        start_t = rospy.Time.now()
        r = rospy.Rate(rate)

        # plot_robot_trajectory(path)
        
        while not rospy.is_shutdown():
            # Find the time from start
            t = (rospy.Time.now() - start_t).to_sec()

            # If the controller has timed out, stop moving and return false
            if timeout is not None and t >= timeout:
                # Set velocities to zero
                self.stop_moving()
                return False

            current_position = get_joint_positions(self._limb)
            current_velocity = get_joint_velocities(self._limb)

            # Get the desired position, velocity, and effort
            (
                target_position, 
                target_velocity, 
                target_acceleration, 
                current_index
            ) = self.interpolate_path(path, t, current_index)

            # For plotting
            if log:
                times.append(t)
                actual_positions.append(current_position)
                actual_velocities.append(current_velocity)
                target_positions.append(target_position)
                target_velocities.append(target_velocity)

            # Run controller
            self.step_control(target_position, target_velocity, target_acceleration)

            # Sleep for a bit (to let robot move)
            r.sleep()

            if current_index >= max_index:
                self.stop_moving()
                break
        
        if log:
            self.plot_results(
                times,
                actual_positions, 
                actual_velocities, 
                target_positions, 
                target_velocities
            )
        return True

    def follow_ar_tag_step(self, tag, limb, kin, num_way=5, rate=200, timeout=None, log=False):
        """
        takes in an AR tag number and follows it with the baxter's arm.  You 
        should look at execute_path() for inspiration on how to write this. 

        Parameters
        ----------
        tag : int
            which AR tag to use
        rate : int
            This specifies how many ms between loops.  It is important to
            use a rate and not a regular while loop because you want the
            loop to refresh at a constant rate, otherwise you would have to
            tune your PD parameters if the loop runs slower / faster
        timeout : int
            If you want the controller to terminate after a certain number
            of seconds, specify a timeout in seconds.
        log : bool
            whether or not to display a plot of the controller performance

        Returns
        -------
        bool
            whether the controller completes the path or not
        """

        # For plotting
        if log:
            times = list()
            actual_positions = list()
            actual_velocities = list()
            target_positions = list()
            target_velocities = list()

        # For interpolation
        max_index = len(path.joint_trajectory.points)-1
        current_index = 0

        # For timing
        start_t = rospy.Time.now()
        r = rospy.Rate(rate)

        # plot_robot_trajectory(path)
        step_fraction = 0.05
        target_time = rate / 1000. # time in seconds
        target_velocity = step_fraction / target_time
        target_acceleration = 0.05

        while not rospy.is_shutdown():
            # Find the time from start
            t = (rospy.Time.now() - start_t).to_sec()

            # If the controller has timed out, stop moving and return false
            if timeout is not None and t >= timeout:
                # Set velocities to zero
                self.stop_moving()
                return False

            current_position = get_joint_positions(self._limb)
            current_velocity = get_joint_velocities(self._limb)

            # Get the desired position, velocity, and effort
            # (
            #     target_position, 
            #     target_velocity, 
            #     target_acceleration, 
            #     current_index
            # ) = self.interpolate_path(path, t, current_index)

            tag_pos = np.array(lookup_tag(tag)).reshape(-1)
            current_workspace_position = np.array(kin.forward_position_kinematics()).reshape(-1)[:3]
            direction = tag_pos - current_workspace_position
            direction = direction / np.linalg.norm(direction)

            target_position = current_position + step_fraction * direction
            
            # For plotting
            if log:
                times.append(t)
                actual_positions.append(current_position)
                actual_velocities.append(current_velocity)
                target_positions.append(target_position)
                target_velocities.append(target_velocity)

            # Run controller
            self.step_control(target_position, target_velocity, target_acceleration)

            # Sleep for a bit (to let robot move)
            r.sleep()

            if current_index >= max_index:
                self.stop_moving()
                break
        
        if log:
            self.plot_results(
                times,
                actual_positions, 
                actual_velocities, 
                target_positions, 
                target_velocities
            )
        return True

    def follow_ar_tag(self, tag, limb, kin, num_way=5, rate=200, timeout=None, log=False):
        """
        takes in an AR tag number and follows it with the baxter's arm.  You 
        should look at execute_path() for inspiration on how to write this. 

        Parameters
        ----------
        tag : int
            which AR tag to use
        rate : int
            This specifies how many ms between loops.  It is important to
            use a rate and not a regular while loop because you want the
            loop to refresh at a constant rate, otherwise you would have to
            tune your PD parameters if the loop runs slower / faster
        timeout : int
            If you want the controller to terminate after a certain number
            of seconds, specify a timeout in seconds.
        log : bool
            whether or not to display a plot of the controller performance

        Returns
        -------
        bool
            whether the controller completes the path or not
        """
        # For plotting
        if log:
            times = list()
            actual_positions = list()
            actual_velocities = list()
            target_positions = list()
            target_velocities = list()

        # For interpolation
        max_index = len(path.joint_trajectory.points)-1
        current_index = 0

        # For timing
        start_t = rospy.Time.now()
        r = rospy.Rate(rate)

        constant_velocity = 0.1

        while not rospy.is_shutdown():
            # Find the time from start
            t = (rospy.Time.now() - start_t).to_sec()

            # If the controller has timed out, stop moving and return false
            if timeout is not None and t >= timeout:
                # Set velocities to zero
                self.stop_moving()
                return False

            current_position = get_joint_positions(self._limb)
            current_velocity = get_joint_velocities(self._limb)


            tag_pos = np.array(lookup_tag(tag)).reshape(-1)
            current_workspace_position = np.array(kin.forward_position_kinematics()).reshape(-1)[:3]
            distance = np.linalg.norm(tag_pos - current_workspace_position)
            total_time = distance / constant_velocity

            path = LinearPath(limb, kin, total_time, tag_pos, num_way, start_pos=None)
            robot_trajectory = path.to_robot_trajectory(num_way, type(self)!=PDWorkspaceVelocityController)

            # Get the desired position, velocity, and effort
            (
                target_position, 
                target_velocity, 
                target_acceleration, 
                current_index
            ) = self.interpolate_path(robot_trajectory, 0.2 * total_time, 0)

            # For plotting
            if log:
                times.append(t)
                actual_positions.append(current_position)
                actual_velocities.append(current_velocity)
                target_positions.append(target_position)
                target_velocities.append(target_velocity)

            # Run controller
            self.step_control(target_position, target_velocity, target_acceleration)

            # Sleep for a bit (to let robot move)
            r.sleep()

            if current_index >= max_index:
                self.stop_moving()
                break
        
        if log:
            self.plot_results(
                times,
                actual_positions, 
                actual_velocities, 
                target_positions, 
                target_velocities
            )
        return True

class FeedforwardJointVelocityController(Controller):
    def step_control(self, target_position, target_velocity, target_acceleration):
        """
        Parameters
        ----------
        target_position: 7x' ndarray of desired positions
        target_velocity: 7x' ndarray of desired velocities
        target_acceleration: 7x' ndarray of desired accelerations
        """
        self._limb.set_joint_velocities(joint_array_to_dict(target_velocity, self._limb))

class PDWorkspaceVelocityController(Controller):
    """
    Look at the comments on the Controller class above.  The difference between this controller and the
    PDJointVelocityController is that this controller compares the baxter's current WORKSPACE position and
    velocity desired WORKSPACE position and velocity to come up with a WORKSPACE velocity command to be sent
    to the baxter.  Then this controller should convert that WORKSPACE velocity command into a joint velocity
    command and sends that to the baxter.  Notice the shape of Kp and Kv
    """
    def __init__(self, limb, kin, Kp, Kv):
        """
        Parameters
        ----------
        limb : :obj:`baxter_interface.Limb`
        kin : :obj:`BaxterKinematics`
        Kp : 6x' :obj:`numpy.ndarray`
        Kv : 6x' :obj:`numpy.ndarray`
        """
        Controller.__init__(self, limb, kin)
        self.Kp = np.diag(Kp)
        self.Kv = np.diag(Kv)

    def step_control(self, target_position, target_velocity, target_acceleration):
        """
        makes a call to the robot to move according to it's current position and the desired position 
        according to the input path and the current time. Each Controller below extends this 
        class, and implements this accordingly. This method should call
        self._kin.forward_psition_kinematics() and self._kin.forward_velocity_kinematics() to get 
        the current workspace position and velocity and self._limb.set_joint_velocities() to set 
        the joint velocity to something.  you may have to look at 
        http://docs.ros.org/diamondback/api/kdl/html/python/geometric_primitives.html to convert the 
        output of forward_velocity_kinematics() to a numpy array.  You may find joint_array_to_dict() 
        in utils.py useful 

        MAKE SURE TO CONVERT QUATERNIONS TO EULER IN forward_position_kinematics().  
        you can use tf.transformations.euler_from_quaternion()

        your target orientation should be (0,0,0) in euler angles and (0,1,0,0) as a quaternion.  

        Parameters
        ----------
        target_position: 3x' ndarray of desired positions
        target_velocity: 3x' ndarray of desired velocities
        target_acceleration: 3x' ndarray of desired accelerations
        """
        curr_workspace_pos_quat = self._kin.forward_position_kinematics()
        curr_workspace_pos = curr_workspace_pos_quat[:3]
        curr_workspace_quat = curr_workspace_pos_quat[3:]
        curr_workspace_euler = tf.transformations.euler_from_quaternion(curr_workspace_quat)
        curr_workspace_coord = np.hstack([curr_workspace_pos, curr_workspace_euler])

        target_workspace_coord = np.hstack([target_position, tf.transformations.euler_from_quaternion([0, 1, 0, 0])])
        
        # curr_twist = self._kin.forward_velocity_kinematics()
        # curr_twist_vel = np.array([curr_twist.vel.x(), curr_twist.vel.y(), curr_twist.vel.z()])
        # curr_twist_rot = np.array([curr_twist.rot.x(), curr_twist.rot.y(), curr_twist.rot.z()])

        curr_twist = np.array(get_workspace_velocities(self._limb, self._kin)).reshape(6)
        curr_twist_vel = curr_twist[:3]
        curr_twist_rot = curr_twist[3:]

        print 'curr_twist_vel" {0}'.format(curr_twist_vel)
        print 'curr_twist_rot" {0}'.format(curr_twist_rot)


        curr_vel = np.hstack([curr_twist_vel, curr_twist_rot])

        p_err = curr_workspace_coord - target_workspace_coord
        v_err = curr_vel - np.hstack([target_velocity, np.zeros(3)])

        
        feedback = -self.Kv.dot(v_err) - self.Kp.dot(p_err) 
        pd_control_workspace = np.hstack([target_velocity, np.zeros(3)]) + feedback
        pd_control_jointspace = np.array(self._kin.jacobian_pseudo_inverse().dot(pd_control_workspace)).reshape(7)

        self._limb.set_joint_velocities(joint_array_to_dict(pd_control_jointspace, self._limb))

class PDJointVelocityController(Controller):
    """
    Look at the comments on the Controller class above.  The difference between this controller and the 
    PDJointVelocityController is that this controller turns the desired workspace position and velocity
    into desired JOINT position and velocity.  Then it compares the difference between the baxter's 
    current JOINT position and velocity and desired JOINT position and velocity to come up with a
    joint velocity command and sends that to the baxter.  notice the shape of Kp and Kv
    """
    def __init__(self, limb, kin, Kp, Kv):
        """
        Parameters
        ----------
        limb : :obj:`baxter_interface.Limb`
        kin : :obj:`BaxterKinematics`
        Kp : 7x' :obj:`numpy.ndarray`
        Kv : 7x' :obj:`numpy.ndarray`
        """
        Controller.__init__(self, limb, kin)
        self.Kp = np.diag(Kp)
        self.Kv = np.diag(Kv)

    def step_control(self, target_position, target_velocity, target_acceleration):
        """
        makes a call to the robot to move according to it's current position and the desired position 
        according to the input path and the current time. Each Controller below extends this 
        class, and implements this accordingly. This method should call
        self._limb.joint_angle and self._limb.joint_velocity to get the current joint position and velocity
        and self._limb.set_joint_velocities() to set the joint velocity to something.  You may find
        joint_array_to_dict() in utils.py useful

        Parameters
        ----------
        target_position: 7x' :obj:`numpy.ndarray` of desired positions
        target_velocity: 7x' :obj:`numpy.ndarray` of desired velocities
        target_acceleration: 7x' :obj:`numpy.ndarray` of desired accelerations
        """
        curr_joint_angles = np.array([self._limb.joint_angle(joint) for joint in self._limb.joint_names()]).reshape(7)
        curr_joint_velocities = np.array([self._limb.joint_velocity(joint) for joint in self._limb.joint_names()]).reshape(7)

        p_err = curr_joint_angles - target_position
        v_err = curr_joint_velocities - target_velocity

        feedback = -self.Kv.dot(v_err) - self.Kp.dot(p_err) 
        pd_control_jointspace = target_velocity + feedback

        self._limb.set_joint_velocities(joint_array_to_dict(pd_control_jointspace, self._limb))
        

class PDJointTorqueController(Controller):
    def __init__(self, limb, kin, Kp, Kv):
        """
        Parameters
        ----------
        limb : :obj:`baxter_interface.Limb`
        kin : :obj:`BaxterKinematics`
        Kp : 7x' :obj:`numpy.ndarray`
        Kv : 7x' :obj:`numpy.ndarray`
        """
        Controller.__init__(self, limb, kin)
        self.Kp = np.diag(Kp)
        self.Kv = np.diag(Kv)

    def step_control(self, target_position, target_velocity, target_acceleration):
        """
        makes a call to the robot to move according to it's current position and the desired position 
        according to the input path and the current time. Each Controller below extends this 
        class, and implements this accordingly. This method should call
        self._limb.joint_angle and self._limb.joint_velocity to get the current joint position and velocity
        and self._limb.set_joint_velocities() to set the joint velocity to something.  You may find
        joint_array_to_dict() in utils.py useful

        Look in section 4.5 of MLS.

        Parameters
        ----------
        target_position: 7x' :obj:`numpy.ndarray` of desired positions
        target_velocity: 7x' :obj:`numpy.ndarray` of desired velocities
        target_acceleration: 7x' :obj:`numpy.ndarray` of desired accelerations
        """
        
        curr_joint_angles = np.array([self._limb.joint_angle(joint) for joint in self._limb.joint_names()]).reshape(7)
        curr_joint_velocities = np.array([self._limb.joint_velocity(joint) for joint in self._limb.joint_names()]).reshape(7)

        M = np.array(self._kin.inertia()).reshape((7, 7))
        # C = self._kin.coriolis()
        JT = np.array(self._kin.jacobian_transpose()).reshape((7, 6))
        M_cartesian = np.array(self._kin.cart_inertia()).reshape((6, 6))
        N = JT.dot(M_cartesian).dot(np.array([0., 0., -0.981, 0., 0., 0.]))

        feedforward = M.dot(target_acceleration) + N # M.dot(target_acceleration) + C.dot(target_velocity) + N

        # print 'M: {0}'.format(M)
        # print 'N: {0}'.format(N)
        print 'feedforward: {0}'.format(feedforward)


        curr_joint_angles = np.array([self._limb.joint_angle(joint) for joint in self._limb.joint_names()]).reshape(7)
        curr_joint_velocities = np.array([self._limb.joint_velocity(joint) for joint in self._limb.joint_names()]).reshape(7)

        p_err = curr_joint_angles - target_position
        v_err = curr_joint_velocities - target_velocity

        feedback = -self.Kv.dot(v_err) - self.Kp.dot(p_err) 
        pd_joint_torque_control = feedforward + feedback

        # import pdb; pdb.set_trace()

        self._limb.set_joint_torques(joint_array_to_dict(pd_joint_torque_control, self._limb))









######################
# GRAD STUDENTS ONLY #
######################

class WorkspaceImpedanceController(Controller):
    def __init__(self, limb, kin, Kp, Kv):
        """
        Parameters
        ----------
        limb : :obj:`baxter_interface.Limb`
        kin : :obj:`BaxterKinematics`
        Kp : 6x' :obj:`numpy.ndarray`
        Kv : 6x' :obj:`numpy.ndarray`
        """
        Controller.__init__(self, limb, kin)
        self.Kp = np.diag(Kp)
        self.Kv = np.diag(Kv)

    def step_control(self, target_position, target_velocity, target_acceleration):
        """

        Parameters
        ----------
        target_position: 7x' ndarray of desired positions
        target_velocity: 7x' ndarray of desired velocities
        target_acceleration: 7x' ndarray of desired accelerations
        """
        raise NotImplementedError

class JointspaceImpedanceController(Controller):
    def __init__(self, limb, kin, Kp, Kv):
        """
        Parameters
        ----------
        limb : :obj:`baxter_interface.Limb`
        kin : :obj:`BaxterKinematics`
        Kp : 7x' :obj:`numpy.ndarray`
        Kv : 7x' :obj:`numpy.ndarray`
        """
        Controller.__init__(self, limb, kin)
        self.Kp = np.diag(Kp)
        self.Kv = np.diag(Kv)

    def step_control(self, target_position, target_velocity, target_acceleration):
        """
        Parameters
        ----------
        target_position: 7x' :obj:`numpy.ndarray` of desired positions
        target_velocity: 7x' :obj:`numpy.ndarray` of desired velocities
        target_acceleration: 7x' :obj:`numpy.ndarray` of desired accelerations
        """
        raise NotImplementedError