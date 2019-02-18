#!/usr/bin/env python

"""
Starter script for lab1. 
Author: Chris Correa
"""
import numpy as np
from math import sin, cos, atan2
import itertools
import matplotlib.pyplot as plt

try:
    import rospy
    import tf
    from geometry_msgs.msg._Point import Point
    import tf.transformations as tfs
    from geometry_msgs.msg import Pose, PoseStamped
    ros_enabled = True
except:
    ros_enabled = False

def length(vec):
    """
    Returns the length of a 1 dimensional numpy vector

    Parameters
    ----------
    vec : nx1 :obj:`numpy.ndarray`

    Returns
    -------
    float
        ||vec||_2^2
    """
    return np.linalg.norm(vec)

def normalize(vec):
    """
    Returns a normalized version of a numpy vector

    Parameters
    ----------
    vec : nx' :obj:`numpy.ndarray

    Returns
    -------
    nx' :obj:`numpy.ndarray`
    """
    return vec / length(vec)

def joint_array_to_dict(vel_torque_array, limb):
    """
    the baxter interface requires you to send the joint velocities / torques
    as a dictionary, this turns and array of velocities and torques into a 
    dictionary with joint names.

    Parameters
    ----------
    vel_torque_array : 7x' :obj:`numpy.ndarray`
        numpy array of velocities or torques to be sent to the baxter
    limb : :obj:`baxter_interface.Limb`
        Limb object

    Returns
    -------
    :obj:`dict` of string->float
        mapping of joint names to joint velocities / torques
    """

    return dict(itertools.izip(limb.joint_names(), vel_torque_array))

def get_joint_positions(limb):
    """
    Returns the baxter joint positions IN ORDER (execute order 66)

    Parameters
    ----------
    limb : :obj:`baxter_interface.Limb`
        Limb object

    Returns
    -------
    7x' :obj:`numpy.ndarray`
        joint positions
    """
    return np.array([limb.joint_angles()[joint_name] for joint_name in limb.joint_names()])

def get_joint_velocities(limb):
    """
    Returns the baxter joint velocities IN ORDER (execute order 66)

    Parameters
    ----------
    limb : :obj:`baxter_interface.Limb`
        Limb object

    Returns
    -------
    7x' :obj:`numpy.ndarray`
        joint velocities
    """
    return np.array([limb.joint_velocities()[joint_name] for joint_name in limb.joint_names()])

def get_workspace_velocities(limb, kin):
    joint_velocities = get_joint_velocities(limb)
    return kin.jacobian().dot(joint_velocities)

def vec(*args):
    """
    all purpose function to get a numpy array of random things.  you can pass
    in a list, tuple, ROS Point message.  you can also pass in:
    vec(1,2,3,4,5,6) which will return a numpy array of each of the elements 
    passed in: np.array([1,2,3,4,5,6])
    """
    if len(args) == 1:
        if type(args[0]) == tuple:
            return np.array(args[0])
        elif ros_enabled and type(args[0]) == Point:
            return np.array((args[0].x, args[0].y, args[0].z))
        else:
            return np.array(args)
    else:
        return np.array(args)

def hat(v):
    """
    See https://en.wikipedia.org/wiki/Hat_operator or the MLS book

    Parameters
    ----------
    v : :obj:`numpy.ndarrray`
        vector form of shape 3x1, 3x, 6x1, or 6x

    Returns
    -------
    3x3 or 6x6 :obj:`numpy.ndarray`
        hat version of the vector v
    """
    if v.shape == (3, 1) or v.shape == (3,):
        return np.array([
                [0, -v[2], v[1]],
                [v[2], 0, -v[0]],
                [-v[1], v[0], 0]
            ])
    elif v.shape == (6, 1) or v.shape == (6,):
        return np.array([
                [0, -v[5], v[4], v[0]],
                [v[5], 0, -v[3], v[1]],
                [-v[4], v[3], 0, v[2]],
                [0, 0, 0, 0]
            ])
    else:
        raise ValueError

def adj(g):
    """
    Adjoint of a rotation matrix.  See the MLS book

    Parameters
    ----------
    g : 4x4 :obj:`numpy.ndarray`
        Rotation matrix

    Returns
    -------
    6x6 :obj:`numpy.ndarray` 
    """
    if g.shape != (4, 4):
        raise ValueError

    R = g[0:3,0:3]
    p = g[0:3,3]
    result = np.zeros((6, 6))
    result[0:3,0:3] = R
    result[0:3,3:6] = hat(p) * R
    result[3:6,3:6] = R
    return result

def twist_from_tf(g):
    """
    Returns the twist version of a 2D rotation matrix
    Parameters
    ----------
    g : 3x3 :obj:`numpy.ndarray`
        2D rotation matrix

    Returns
    -------
    3x' :obj:`numpy.ndarray`
    """
    return vec(g[0,2], g[1,2], atan2(g[1,0], g[0,0]))

def rotation2d(theta):
    """
    2D rotation matrix from a single theta around the origin

    Parameters
    ----------
    theta : float

    Returns
    -------
    2x2 :obj:`numpy.ndarray`
    """
    return np.array([
            [cos(theta), -sin(theta)],
            [sin(theta), cos(theta)]
        ])

def rigid(twist):
    """
    Returns a 3x3 Rotation Matrix version of a 2D twist

    Parameters
    ----------
    twist : 3x1 :obj:`numpy.ndarray`

    Returns
    -------
    3x3 :obj:`numpy.ndarray`
    """
    return np.array([
            [cos(twist[2]), -sin(twist[2]), twist[0]],
            [sin(twist[2]), cos(twist[2]), twist[1]],
            [0, 0, 1]
        ])

def look_at_general(origin, direction):
    """
    Creates a 3D Rotation Matrix at the origin such that the z axis is the same
    as the direction specified.  There are infinitely many of such matrices, 
    but we choose the one where the x axis is as vertical as possible.  

    Parameters
    ----------
    origin : 3x1 :obj:`numpy.ndarray`
    direction : 3x1 :obj:`numpy.ndarray`

    Returns
    -------
    4x4 :obj:`numpy.ndarray`
    """
    up = vec(0,0,1)
    z = normalize(direction)
    x = normalize(np.cross(up, z))
    y = np.cross(z, x) 

    result = np.eye(4)
    result[0:3,0] = x
    result[0:3,1] = y
    result[0:3,2] = z
    result[0:3,3] = origin
    return result

def create_pose_from_rigid_transform(g):
    """
    takes a rotation matrix and turns it into a ROS Pose

    Parameters
    ----------
    g : 4x4 : :obj:`numpy.ndarray`

    Returns
    -------
    :obj:`geometry_msgs.msg.Pose`
    """
    position = tfs.translation_from_matrix(g)
    quaternion = tfs.quaternion_from_matrix(g)
    wpose = Pose()
    wpose.position.x = position[0]
    wpose.position.y = position[1]
    wpose.position.z = position[2]
    wpose.orientation.x = quaternion[0]
    wpose.orientation.y = quaternion[1]
    wpose.orientation.z = quaternion[2]
    wpose.orientation.w = quaternion[3]
    return wpose

def create_pose_stamped_from_pos_quat(pos, quat, frame_id):
    """
    takes a position and quaternion and turns it into a ROS PoseStamped

    Parameters
    ----------
    pos : 3x1 : :obj:`numpy.ndarray`
    quat : 4x1 : :obj:`numpy.ndarray`


    Returns
    -------
    :obj:`geometry_msgs.msg.PoseStamped`
    """
    wpose = PoseStamped()
    wpose.header.frame_id = frame_id
    wpose.pose.position.x = pos[0]
    wpose.pose.position.y = pos[1]
    wpose.pose.position.z = pos[2]
    wpose.pose.orientation.x = quat[0]
    wpose.pose.orientation.y = quat[1]
    wpose.pose.orientation.z = quat[2]
    wpose.pose.orientation.w = quat[3]
    return wpose

def linear_interpolation(start, end, num_way):
    """
    Parameters
    ----------
    start : (x, y, z) start point
    end : (x, y, z) end point
    num_way : int number of waypoints
    """
    increments = np.linspace(0., 1., num_way)
    start = np.array(start)
    end = np.array(end)
    return np.array([(1. - inc) * start + inc * end for inc in increments])

def linear_interpolation_two_points(start, end, fraction):
    """
    Linearly interpolate between start and end using fraction.
    """
    assert fraction >= 0 and fraction <= 1
    return (1. - fraction) * start + fraction * end

def plot_robot_trajectory(robot_traj):
    points = robot_traj.joint_trajectory.points
    positions = []
    velocities = []
    accelerations = []
    for p in points:
        positions.append(p.positions)
        velocities.append(p.velocities)
        accelerations.append(p.accelerations)


    positions = np.array(positions).reshape((len(positions), len(positions[0])))
    velocities = np.array(velocities).reshape((len(velocities), len(velocities[0])))
    accelerations = np.array(accelerations).reshape((len(accelerations), len(accelerations[0])))

    plt.figure()
    for joint in range(7):
        for i, (x_name, x) in enumerate(zip(['positions', 'velocities', 'accelerations'], [positions, velocities, accelerations])):
            plt.subplot(x.shape[1], 3, 3*joint + i + 1)
            plt.plot(np.arange(x.shape[0]) * (5. / x.shape[0]), x[:,joint].reshape(-1))
            plt.xlabel('time')
            plt.ylabel('joint {0}'.format(joint))

    print "Close the plot window to continue"
    # plt.legend()
    plt.show()
    # import pdb; pdb.set_trace()

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
    for i in range(200):
        r.sleep()

    t = listener.getLatestCommonTime(from_frame, to_frame)
    tag_pos, _ = listener.lookupTransform(from_frame, to_frame, t)
    return vec(tag_pos)