#!/usr/bin/python

# You may have to run the following commands to have this stuff work:
# rosrun baxter_tools enable_robot.py -e
# rosrun baxter_interface joint_trajectory_action_server.py
# roslaunch baxter_moveit_config move_group.launch

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import baxter_interface
from baxter_interface import CHECK_VERSION
import csv
import math

def main():

    #Start a node
    rospy.init_node('moveit_node')

    rs = baxter_interface.RobotEnable(CHECK_VERSION)
    rs.enable()

    #Initialize moveit_commander
    moveit_commander.roscpp_initialize(sys.argv)

    # robot = moveit_commander.RobotCommander()
    # scene = moveit_commander.PlanningSceneInterface()

    

    #filename = parse_filename_from_args()
    points = [[0.5 -0.2 0.0][0.6 -0.2 0.0] [0.7 -0.2 0.0] [0.8 -0.2 0.0]  ]
    quaternions = [[0.5 0.5 0.5 -0.5][0.5 0.5 0.5 -0.5][0.5 0.5 0.5 -0.5][0.5 0.5 0.5 -0.5]] 
    execute_trajectory_from_points(points, quaternions)
    #execute_trajectory(points,quaternions)

def execute_trajectory(waypoints):
    #Initialize both arms    
    left_arm = moveit_commander.MoveGroupCommander('left_arm')
    right_arm = moveit_commander.MoveGroupCommander('right_arm')

    jump_threshold = 0      # Does nothing?
    eef_step = 0.05 # Does nothing?
    dist_threshold = 0.03

    (plan, fraction) = left_arm.compute_cartesian_path(waypoints, eef_step, jump_threshold)
    
    left_arm.execute(plan)


def execute_trajectory_from_points(points, quaternions):
    min_distance = 0.03    
    old_point = []    
    waypoints = []
    wpose = geometry_msgs.msg.Pose()    
    for i in range(len(points)):
        wpose.orientation.x = quaternions[i, 0]
        wpose.orientation.y = quaternions[i, 1]
        wpose.orientation.z = quaternions[i, 2]
        wpose.orientation.w = quaternions[i, 3]
        wpose.position.x = points[i, 0]
        wpose.position.y = point[i, 1]
        wpose.position.z = point[i, 2]
        
        
        if len(old_point) != 0:
            if distance(old_point, point) >= min_dist:
                waypoints.append(copy.deepcopy(wpose))
                old_point = point   
        else:
            waypoints.append(copy.deepcopy(wpose))
            old_point = point

    execute_trajectory(waypoints)

def execute_trajectory_from_file(filename):
    waypoints = get_waypoints(filename, dist_threshold)
    execute_trajectory(waypoints)    

def parse_filename_from_args():
    arg_fmt = argparse.RawDescriptionHelpFormatter
    parser = argparse.ArgumentParser(formatter_class=arg_fmt,
                                     description=main.__doc__)
    parser.add_argument('-f', dest='filename', help='the filename to record to')
    args = parser.parse_args(rospy.myargv()[1:])

    if (args.filename is None):
        return 'test000.csv'
    elif (args.filename[-4:] == '.csv'):
        return args.filename[:-4]
    else:
        return args.filename

# Creates a list of cartesian waypoints for the path
def get_waypoints(filename, min_dist):

    # The list of columns from which to extract the data
    relevant_columns = ['end_left_x', 'end_left_y', 'end_left_z', 
                        'end_left_rotx', 'end_left_roty', 'end_left_rotz', 'end_left_rotw',
                        'end_right_x', 'end_right_y', 'end_right_z',
                        'end_right_rotx', 'end_right_roty', 'end_right_rotz', 'end_right_rotw']
    old_point = []
    column_indices = {}    
    waypoints = []
    wpose = geometry_msgs.msg.Pose()
    with open(filename, 'rb') as csvfile:
        reader = csv.reader(csvfile, delimiter=',')
        first_column = reader.next()

        # Constructs a dictionary linking column index to column name        
        for column in relevant_columns:         
            index = first_column.index(column)
            column_indices[column] = index     
        
        # Extracts each row of the csv file
        for row in reader:
            point = [float(row[column_indices['end_left_x']]),
                    float(row[column_indices['end_left_y']]), 
                    float(row[column_indices['end_left_z']])]

            wpose.orientation.x = float(row[column_indices['end_left_rotx']])
            wpose.orientation.y = float(row[column_indices['end_left_roty']])
            wpose.orientation.z = float(row[column_indices['end_left_rotz']])
            wpose.orientation.w = float(row[column_indices['end_left_rotw']])
            wpose.position.x = point[0]
            wpose.position.y = point[1]
            wpose.position.z = point[2]
            
            
            if len(old_point) != 0:
                if distance(old_point, point) >= min_dist:
                    waypoints.append(copy.deepcopy(wpose))
                    old_point = point   
            else:
                waypoints.append(copy.deepcopy(wpose))
                old_point = point
    return waypoints

def distance(point1, point2):
    dist_squared = 0
    for dimension in range(len(point1)):
        dist_squared += (point1[dimension] - point2[dimension])**2
    return math.sqrt(dist_squared)


if __name__ == '__main__':
    main()
