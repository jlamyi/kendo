#!/usr/bin/python
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import baxter_interface
import math
import numpy as np
from geometry_msgs.msg import PoseStamped
from baxter_interface import CHECK_VERSION

#Initialize both arms  
left_arm = moveit_commander.MoveGroupCommander('left_arm')
right_arm = moveit_commander.MoveGroupCommander('right_arm')

def main():

    #Start a node
    rospy.init_node('sowrd_pulling_node')

    rs = baxter_interface.RobotEnable(CHECK_VERSION)
    rs.enable()

    #Set up interface
    left_gripper = baxter_interface.Gripper("left")
    right_gripper = baxter_interface.Gripper("right")
    left_button_thin = baxter_interface.DigitalIO('left_upper_button')
    right_button_thin = baxter_interface.DigitalIO('right_upper_button')
    left_button_circ = baxter_interface.DigitalIO('left_lower_button')
    right_button_circ = baxter_interface.DigitalIO('right_lower_button')
    
    #Initialize moveit_commander
    moveit_commander.roscpp_initialize(sys.argv)
   
    #Put the arms aside
    #left = np.array([[0.0,0.5,-0.6],[0.0,1.0,0.0,0.0]])
    #right = np.array([[0.0,-0.5,-0.6],[0.0,1.0,0.0,0.0]]) 

    #Position 1: Baoquan Action   
    left = np.array([[0.541,0.138,0.103],[0.479,-0.124,0.469,0.731]])
    right = np.array([[0.598,-0.045,0.275],[-0.346,-0.059,-0.597,0.721]])
    position(left,right,False)
    rospy.sleep(1.0)
    
    #Gripper start position
    left_gripper.calibrated()
    left_gripper.close()
    right_gripper.calibrated()
    right_gripper.open()
   
    #Position 2: getting the sword  
    left = np.array([[0.947,0.26,-0.11],[0.13,0.74,-0.148,0.642]])
    right = np.array([[0.0,-0.5,-0.6],[0.0,1.0,0.0,0.0]])
    position(left,right)
    rospy.sleep(1.0) 
    left_gripper.open()

    while True:
        #When thin button is pressed, move to position 3
        if (left_button_thin.state==True):

            #Position 3: start position of pulling a sword
            left = np.array([[0.479,0.327,-0.211],[0.1635,0.8409,-0.04805,0.51366]])
            right = np.array([[0.0,-0.5,-0.6],[0.0,1.0,0.0,0.0]])
            position(left,right)
            rospy.sleep(1.0)
            
            #Catersian path plan 1: try to catch the sword with right arm
            a = [0.552,0.014,-0.12]
            b = [0.552,0.114,-0.12]          
            c = [0.552,0.214,-0.12]
            o = [0.28048,-0.68855,-0.52949,-0.4085]
            points = np.array([a,b,c])
            quaternions = np.array([o,o,o]) 
            execute_trajectory_from_points(points, quaternions)
            
            #Switching the sword from left arm to right arm
            right_gripper.close()
            rospy.sleep(1.0)
            left_gripper.open()
            rospy.sleep(1.0)

            #Catersian path plan 2: Pulling out the sword
            a = [0.562,0.264,-0.12]
            b = [0.562 + 0.01218693434*2,0.264,-0.12 + 0.09925461516*2]
            c = [0.562 + 0.01218693434*4,0.264,-0.12 + 0.09925461516*4]
            d = [0.562 + 0.01218693434*6,0.264,-0.12 + 0.09925461516*6]
            o = [0.28048,-0.68855,-0.52949,-0.4085]
            points = np.array([a,b,c,d])
            quaternions = np.array([o,o,o,o]) 
            execute_trajectory_from_points(points, quaternions)
            
            #Position 4: Waving the sword  
            left = np.array([[0.0,0.5,-0.6],[0.0,1.0,0.0,0.0]])
            right = np.array([[1.08,-0.33,0.437],[0.56151,0.01578,0.80809,-0.17733]])
            position(left,right)
            
            #Position 5: Getting to start position 
            left = np.array([[0.0,0.5,-0.6],[0.0,1.0,0.0,0.0]])
            right = np.array([[0.5,-0.1,0.0],[0.5,0.5,0.5,-0.5]])
            position(left,right)
            break
        else:
            pass

def position(left,right,order=True):

    ### Left arm setting

    #Left arm starting position
    goal_left = PoseStamped()
    goal_left.header.frame_id = "base"
    
    #x, y, and z position
    goal_left.pose.position.x = left[0][0]
    goal_left.pose.position.y = left[0][1]
    goal_left.pose.position.z = left[0][2]
    
    #Orientation as a quaternion
    goal_left.pose.orientation.x = left[1][0]
    goal_left.pose.orientation.y = left[1][1]
    goal_left.pose.orientation.z = left[1][2]
    goal_left.pose.orientation.w = left[1][3]
    
    #Set the goal state
    left_arm.set_pose_target(goal_left)
    
    #Set the start state
    left_arm.set_start_state_to_current_state()

    #Plan a path
    left_plan = left_arm.plan()



    ### Right arm setting

    #Right arm starting position
    goal_right = PoseStamped()
    goal_right.header.frame_id = "base"

    #x, y, and z position
    goal_right.pose.position.x = right[0][0]
    goal_right.pose.position.y = right[0][1]
    goal_right.pose.position.z = right[0][2]
    
    #Orientation as a quaternion
    goal_right.pose.orientation.x = right[1][0]
    goal_right.pose.orientation.y = right[1][1]
    goal_right.pose.orientation.z = right[1][2]
    goal_right.pose.orientation.w = right[1][3]

    #Set the goal state
    right_arm.set_pose_target(goal_right)

    #Set the start state
    right_arm.set_start_state_to_current_state()

    #Plan a path
    right_plan = right_arm.plan()

    #Execution: if order is true, left arm goes first, else right arm goes first
    if order:
        left_arm.execute(left_plan)
        right_arm.execute(right_plan)
    else:
        right_arm.execute(right_plan)
        left_arm.execute(left_plan)

def execute_trajectory(waypoints):
  
    # ....
    jump_threshold = 10000  
    # Calculate a point of trajectory for each eef_step   
    eef_step = 0.01 
    # The minimum distance between each waypoint
    dist_threshold = 0.03 

    (plan, fraction) = right_arm.compute_cartesian_path(waypoints, eef_step, jump_threshold)
    #print "plan: ",plan
    print "fraction: ",fraction

    #Execution
    right_arm.execute(plan)


def execute_trajectory_from_points(points, quaternions):

    min_dist = 0.03    
    old_point = []    
    waypoints = []
    point = []
    wpose = geometry_msgs.msg.Pose() 
   
    for i in range(len(points)):
        wpose.orientation.x = quaternions[i, 0]
        wpose.orientation.y = quaternions[i, 1]
        wpose.orientation.z = quaternions[i, 2]
        wpose.orientation.w = quaternions[i, 3]
        wpose.position.x = points[i, 0]
        wpose.position.y = points[i, 1]
        wpose.position.z = points[i, 2]
        
        if len(old_point) != 0:
            if distance(old_point, point) >= min_dist:
                waypoints.append(copy.deepcopy(wpose))
                old_point = point   
        else:
            waypoints.append(copy.deepcopy(wpose))
            old_point = point

    #print "waypoints: ", waypoints
    execute_trajectory(waypoints)

def distance(point1, point2):
    dist_squared = 0
    for dimension in range(len(point1)):
        dist_squared += (point1[dimension] - point2[dimension])**2
    return math.sqrt(dist_squared)


if __name__ == '__main__':
    main()
