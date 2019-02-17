#!/usr/bin/env python
"""
Starter script for lab1. 
Author: Chris Correa
"""
import copy
import sys
import argparse
import time
import numpy as np
import signal

from paths.paths import LinearPath, CircularPath, MultiplePaths
from controllers.controllers import (
    PDWorkspaceVelocityController, 
    PDJointVelocityController, 
    PDJointTorqueController, 
    FeedforwardJointVelocityController
)
from utils.utils import *
from path_planner import PathPlanner

try:
    import rospy
    import tf
    import baxter_interface
    import moveit_commander
    from moveit_msgs.msg import DisplayTrajectory, RobotState
    from baxter_pykdl import baxter_kinematics
except:
    print 'Couldn\'t import ROS, I assume you\'re working on just the paths on your own computer'

def lookup_tag(tag_number):
    """
    Given an AR tag number, this returns the position of the AR tag in the robot's base frame.
    You can use either this function or try starting the scripts/tag_pub.py script.  More info
    about that script is in that file.  

    Parameters
    ----------
    tag_number : int

    Returns
    -------
    3x' :obj:`numpy.ndarray`
        tag position
    """
    listener = tf.TransformListener()
    from_frame = 'base'
    to_frame = 'ar_marker_{}'.format(tag_number)

    r = rospy.Rate(200)
    while (
        not listener.frameExists(from_frame) or not listener.frameExists(to_frame)
    ) and (
        not rospy.is_shutdown()
    ):
        print 'Cannot find AR marker {}, retrying'.format(tag_number)
        r.sleep()

    print 'Found transformation to AR marker. Sleeping for a bit...'
    for i in range(200 * 1):
        r.sleep()

    t = listener.getLatestCommonTime(from_frame, to_frame)
    tag_pos, _ = listener.lookupTransform(from_frame, to_frame, t)
    return vec(tag_pos)

def get_trajectory(limb, kin, total_time, task, goal_pos, num_way, controller_name, radius=None, circle_height=None, start_pos=None):
    """
    Returns an appropriate robot trajectory for the specified task.  You should 
    be implementing the path functions in paths.py and call them here
    
    Parameters
    ----------
    task : string
        name of the task.  Options: line, circle, square
    goal_pos : 3x' :obj:`numpy.ndarray`
        
    Returns
    -------
    :obj:`moveit_msgs.msg.RobotTrajectory`
    """
    if task == 'line':
        path = LinearPath(limb, kin, total_time, goal_pos, num_way, start_pos=start_pos)
    elif task == 'circle':
        assert radius is not None and circle_height is not None
        path = CircularPath(limb, kin, total_time, goal_pos, num_way, radius, circle_height)
    elif task == 'square':
        assert goal_pos.shape == (4, 3)
        paths = []
        for i in range(len(goal_pos)):
            start_pos_i = goal_pos[i].reshape(3)
            if i == len(goal_pos) - 1:
                goal_pos_i = goal_pos[0].reshape(3)
            else:
                goal_pos_i = goal_pos[i + 1].reshape(3)
            paths.append(LinearPath(limb, kin, total_time, goal_pos_i, num_way, start_pos=start_pos_i))
        path = MultiplePaths(limb, kin, paths)
    else:
        raise ValueError('task {} not recognized'.format(task))
    return path.to_robot_trajectory(num_way, controller_name!='workspace')

def get_controller(controller_name):
    """
    Gets the correct controller from controllers.py

    Parameters
    ----------
    controller_name : string

    Returns
    -------
    :obj:`Controller`
    """
    if controller_name == 'workspace':
        # YOUR CODE HERE
        # start pos: [0.5839751304446147, 0.629241775857017, 0.4323214883062389]
        Kp = [1., 0.7, 1., 0., 0., 0.]
        Kv = [0., 0., 0., 0., 0., 0.]
        controller = PDWorkspaceVelocityController(limb, kin, Kp, Kv)
    elif controller_name == 'jointspace':
        # YOUR CODE HERE
        Kp = [0.1, 0.1, 0.1, 0.05, 0.05, 0.05, 0.05]
        Kv = [0., 0., 0., 0., 0., 0., 0.]
        controller = PDJointVelocityController(limb, kin, Kp, Kv)
    elif controller_name == 'torque':
        # YOUR CODE HERE
        Kp = [20., 20., 20., 10., 10., 10., 5.]
        Kv = [5., 3., 5., 3., 1., 1., 1.]
        controller = PDJointTorqueController(limb, kin, Kp, Kv)
    elif controller_name == 'open_loop':
        controller = FeedforwardJointVelocityController(limb, kin)
    else:
        raise ValueError('Controller {} not recognized'.format(controller_name))
    return controller

if __name__ == "__main__":
    """
    Examples of how to run me:
    python scripts/main.py --help <------This prints out all the help messages
    and describes what each parameter is
    python scripts/main.py -t 1 -ar 1 -c workspace -a left --log
    python scripts/main.py -t 2 -ar 2 -c velocity -a left --log
    python scripts/main.py -t 3 -ar 3 -c torque -a right --log
    python scripts/main.py -t 1 -ar 4 5 --path_only --log

    You can also change the rate, timeout if you want
    """
    parser = argparse.ArgumentParser()
    parser.add_argument('-task', '-t', type=str, default='line', help=
        'Options: line, circle, square.  Default: line'
    )
    parser.add_argument('-ar_marker', '-ar', nargs='+', default=None, help=
        'Which AR marker to use.  Default: None'
    )
    parser.add_argument('-controller_name', '-c', type=str, default='workspace', 
        help='Options: open_loop, workspace, jointspace, or torque.  Default: workspace'
    )
    parser.add_argument('-arm', '-a', type=str, default='left', help=
        'Options: left, right.  Default: left'
    )
    parser.add_argument('-rate', type=int, default=200, help="""
        This specifies how many ms between loops.  It is important to use a rate
        and not a regular while loop because you want the loop to refresh at a
        constant rate, otherwise you would have to tune your PD parameters if 
        the loop runs slower / faster.  Default: 200"""
    )
    parser.add_argument('-timeout', type=int, default=None, help=
        """after how many seconds should the controller terminate if it hasn\'t already.  
        Default: None"""
    )
    parser.add_argument('-total_time', type=float, default=5., help=
        """For how long the path should be followed (seconds).  
        Default: 2 seconds"""
    )
    parser.add_argument('-num_way', type=int, default=300, help=
        'How many waypoints for the :obj:`moveit_msgs.msg.RobotTrajectory`.  Default: 300'
    )
    parser.add_argument('--moveit', action='store_true', help=
        """If you set this flag, moveit will take the path you plan and execute it on 
        the real robot"""
    )
    parser.add_argument('-radius', type=float, default=0.1, help=
        """Radius of the circle for the circle path task.  
        Default: 0.1 meters"""
    )
    parser.add_argument('-circle_height', type=float, default=0.2, help=
        """Height above the center of the circle at which the circle path task should be performed. 
        Default: 0.2 meters"""
    )
    parser.add_argument('--log', action='store_true', help='plots controller performance')
    args = parser.parse_args()


    rospy.init_node('moveit_node')
    # this is used for sending commands (velocity, torque, etc) to the robot 
    limb = baxter_interface.Limb(args.arm)
    # this is used to get the dynamics (inertia matrix, manipulator jacobian, etc) from the robot
    # in the current position, UNLESS you specify other joint angles.  see the source code
    # https://github.com/valmik/baxter_pykdl/blob/master/src/baxter_pykdl/baxter_pykdl.py
    # for info on how to use each method
    kin = baxter_kinematics(args.arm)


    # line
    line_tag_pos = np.array([0.6, 0.3, 0.2])

    # circle
    circle_tag_pos = np.array([0.6377519969937113, 0.36179129980361535, -0.225032243019325])

    # square
    goal_pos_1 = np.array([0.6, 0.3, 0.])
    goal_pos_2 = np.array([0.5, 0.3, 0.])
    goal_pos_3 = np.array([0.5, 0.2, 0.])
    goal_pos_4 = np.array([0.6, 0.2, 0.])
    square_tag_pos = np.array([goal_pos_1, goal_pos_2, goal_pos_3, goal_pos_4]).reshape((4, 3))

    if args.ar_marker:
        tag_pos = [lookup_tag(marker) for marker in args.ar_marker]
        if args.task != 'square':
            assert len(tag_pos) == 1
        if len(tag_pos) == 1:
            tag_pos = np.array(tag_pos[0]).reshape(3)
        else:
            tag_pos = np.array(tag_pos).reshape((len(tag_pos), 3))
        tag_pos[2] += 0.05 # add to height so as not to collide with AR marker/table
        print 'AR marker pos: {0}'.format(tag_pos)
    else:
        tag_pos = eval('{0}_tag_pos'.format(args.task))
        print 'goal pos: {0}'.format(tag_pos)

    workspace_start_pos = np.array([0.3, 0.5, 0.6])
    jointspace_start_pos = np.array([0.3, 0.4, 0.6]) # this is actually in workspace for the joinstpace controller
    torque_start_pos = np.array([0.4, 0.4, 0.5])
    if args.ar_marker:
        start_pos = None
    else:
        start_pos = eval('{0}_start_pos'.format(args.controller_name))

    # Get an appropriate RobotTrajectory for the task (circular, linear, or square)
    # If the controller is a workspace controller, this should return a trajectory where the
    # positions and velocities are workspace positions and velocities.  If the controller
    # is a jointspace or torque controller, it should return a trajectory where the positions
    # and velocities are the positions and velocities of each joint.
    robot_trajectory = get_trajectory(limb, kin, args.total_time, args.task, tag_pos, args.num_way, args.controller_name, 
            radius=args.radius, circle_height=args.circle_height, start_pos=start_pos)

    # if args.controller_name == 'jointspace':
    #     plot_robot_trajectory(robot_trajectory)

    planned_path_pub = rospy.Publisher('move_group/display_planned_path', DisplayTrajectory, queue_size=10)
    
    # This is a wrapper around MoveIt! for you to use.  We use MoveIt! to go to the start position
    # of the trajectory
    planner = PathPlanner('{}_arm'.format(args.arm))
    if args.controller_name == "workspace":
        pose = create_pose_stamped_from_pos_quat(
            robot_trajectory.joint_trajectory.points[0].positions,
            [0, 1, 0, 0],
            'base'
        )
        plan = planner.plan_to_pose(pose)
    else:
        # import pdb; pdb.set_trace()
        plan = planner.plan_to_joint_pos(robot_trajectory.joint_trajectory.points[0].positions, limb)
    try:
        raw_input('Press <Enter> to move to start of the trajectory using MOVEIT')
    except KeyboardInterrupt:
        sys.exit()
    planner.execute_plan(plan)

    curr_joint_positions = get_joint_positions(limb)
    print "curr_joint_positions"
    print curr_joint_positions

    if args.moveit:
        # LAB 1 PART A
        # by publishing the trajectory to the move_group/display_planned_path topic, you should 
        # be able to view it in RViz.  You will have to click the "loop animation" setting in 
        # the planned path section of MoveIt! in the menu on the left side of the screen.
        disp_traj = DisplayTrajectory()
        disp_traj.trajectory.append(robot_trajectory)
        # disp_traj.trajectory_start = planner._group.get_current_joint_values()
        disp_traj.trajectory_start = RobotState()
        planned_path_pub.publish(disp_traj)

        try:
            raw_input('Press <Enter> to execute the trajectory using MOVEIT')
        except KeyboardInterrupt:
            sys.exit()
        # uses MoveIt! to execute the trajectory.  make sure to view it in RViz before running this.
        # the lines above will display the trajectory in RViz

        planner.execute_plan(robot_trajectory)
    else:
        # LAB 1 PART B
        disp_traj = DisplayTrajectory()
        disp_traj.trajectory.append(robot_trajectory)
        # disp_traj.trajectory_start = planner._group.get_current_joint_values()
        disp_traj.trajectory_start = RobotState()
        planned_path_pub.publish(disp_traj)
        
        controller = get_controller(args.controller_name)
        try:
            raw_input('Press <Enter> to execute the trajectory using YOUR OWN controller')
        except KeyboardInterrupt:
            sys.exit()
        # execute the path using your own controller.
        done = controller.execute_path(
            robot_trajectory, 
            rate=args.rate, 
            timeout=args.timeout, 
            log=args.log
        )
        if not done:
            print 'Failed to move to position'
            sys.exit(0)

