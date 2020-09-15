#!/usr/bin/env python

"""
Trajectory execution code for my thesis experiment.
This code is developed on top of the Baxter SDK library.
A few things to ensure before you can execute the trajectory 
and see the expected result. Those points are as follows,
1. change the traj_file_name variable as per the file name containing the trajectory.
2. make sure the trajectory file is located in the path: ~/ros_ws/traj_files/
3. you can play with the tm values while creating the trajectory elements.
Author: Anirban Sinha
Affiliation: Stony Brook University
"""

import argparse
import sys

from copy import copy
from os.path import expanduser
import rospy

import actionlib

from control_msgs.msg import (
    FollowJointTrajectoryAction,
    FollowJointTrajectoryGoal,
)
from trajectory_msgs.msg import (
    JointTrajectoryPoint,
)

import baxter_interface

from baxter_interface import CHECK_VERSION


home_dir = expanduser("~")


class Trajectory(object):
    def __init__(self, limb):
        ns = 'robot/limb/' + limb + '/'
        self._client = actionlib.SimpleActionClient(
            ns + "follow_joint_trajectory",
            FollowJointTrajectoryAction,
        )
        self._goal = FollowJointTrajectoryGoal()
        self._goal_time_tolerance = rospy.Time(0.1)
        self._goal.goal_time_tolerance = self._goal_time_tolerance
        server_up = self._client.wait_for_server(timeout=rospy.Duration(10.0))
        if not server_up:
            rospy.logerr("Timed out waiting for Joint Trajectory"
                         " Action Server to connect. Start the action server"
                         " before running example.")
            rospy.signal_shutdown("Timed out waiting for Action Server")
            sys.exit(1)
        self.clear(limb)

    def add_point(self, positions, time):
        point = JointTrajectoryPoint()
        point.positions = copy(positions)
        point.time_from_start = rospy.Duration(time)
        self._goal.trajectory.points.append(point)

    def start(self):
        self._goal.trajectory.header.stamp = rospy.Time.now()
        self._client.send_goal(self._goal)

    def stop(self):
        self._client.cancel_goal()

    def wait(self, timeout=15.0):
        self._client.wait_for_result(timeout=rospy.Duration(timeout))

    def result(self):
        return self._client.get_result()

    def clear(self, limb):
        self._goal = FollowJointTrajectoryGoal()
        self._goal.goal_time_tolerance = self._goal_time_tolerance
        self._goal.trajectory.joint_names = [limb + '_' + joint for joint in \
            ['s0', 's1', 'e0', 'e1', 'w0', 'w1', 'w2']]


def main():
    arg_fmt = argparse.RawDescriptionHelpFormatter
    parser = argparse.ArgumentParser(formatter_class=arg_fmt,
                                     description=main.__doc__)
    required = parser.add_argument_group('required arguments')
    required.add_argument(
        '-l', '--limb', required=True, choices=['left', 'right'],
        help='send joint trajectory to which limb'
    )
    args = parser.parse_args(rospy.myargv()[1:])
    limb = args.limb

    print("Initializing node... ")
    rospy.init_node("rsdk_joint_trajectory_client_%s" % (limb,))
    print("Getting robot state... ")
    rs = baxter_interface.RobotEnable(CHECK_VERSION)
    print("Enabling robot... ")
    rs.enable()
    print("Running. Ctrl-c to quit")
    
    initial_position = [0.3080,-0.5000,-0.7860,0.8020,0.2520,0.3920,0.1160]
    move_limb = baxter_interface.Limb(limb)
    # Acquire current joint angles
    current_js = move_limb.joint_angles()
    current_js[limb + "_s0"] = initial_position[0]
    current_js[limb + "_s1"] = initial_position[1]
    current_js[limb + "_e0"] = initial_position[2]
    current_js[limb + "_e1"] = initial_position[3]
    current_js[limb + "_w0"] = initial_position[4]
    current_js[limb + "_w1"] = initial_position[5]
    current_js[limb + "_w2"] = initial_position[6]
    # Execute
    move_limb.move_to_joint_positions(current_js)

    traj = Trajectory(limb)
    rospy.on_shutdown(traj.stop)

    # Read joint angle file
    print("Reading joint angle file")
    
    # IK_flag = "robustIK"
    IK_flag = "naturalIK"
    
    if IK_flag == "robustIK":
        traj_file_name = "joint_path_obs_robustIK_RealBaxter.txt"
        grasp_position = [-0.3961,0.0676,-2.5451,1.0459,2.3148,2.0278,-0.1543]
    elif IK_flag == "naturalIK":
        traj_file_name = "joint_path_obs_naturalIK_RealBaxter.txt"
        grasp_position = [-0.3205,-0.9157,-0.7849,1.0243,0.4434,1.3282,-0.0216]

    data = open(home_dir+"/ros_ws/traj_files/"+traj_file_name, 'r')
    tm = 0.0
    line_num = 0
    while True:
        line = data.readline()
        if not line:
            break
        angles = line.split(",")
        pt = []
        if line_num < 100:  # first 100 data points are chosen by trial and error
            tm += 0.090     # time increament is chosen manually
            # tm += 0.015     # time increament is chosen manually
        elif line_num < 600:
            tm += 0.019     # time increament is chosen manually
            # tm += 0.015     # time increament is chosen manually
        else:
            tm += 0.020
        pt = [float(j) for j in angles]
        traj.add_point(pt, tm)
        line_num += 1
    print('Reading joint angle file completed')

    traj.start()
    traj.wait(120.0)
        
    # Acquire current joint angles
    current_js[limb + "_s0"] = grasp_position[0]
    current_js[limb + "_s1"] = grasp_position[1]
    current_js[limb + "_e0"] = grasp_position[2]
    current_js[limb + "_e1"] = grasp_position[3]
    current_js[limb + "_w0"] = grasp_position[4]
    current_js[limb + "_w1"] = grasp_position[5]
    current_js[limb + "_w2"] = grasp_position[6]

    # Execute
    move_limb.move_to_joint_positions(current_js)
    
    print("Exiting - Joint Trajectory Action Test Complete")

if __name__ == "__main__":
    main()
