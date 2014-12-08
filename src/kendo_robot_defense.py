#!/usr/bin/env python
import sys
import rospy
import roslib
import moveit_commander
import baxter_interface
import tf
from moveit_msgs.msg import OrientationConstraint, Constraints
from geometry_msgs.msg import PoseStamped
from baxter_interface import gripper as baxter_gripper
from std_msgs.msg import String
from tf.msg import tfMessage

#Initialize moveit_commander
moveit_commander.roscpp_initialize(sys.argv)

#Start a node
rospy.init_node('kendo_node')

#Initialize both arms
robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
left_arm = moveit_commander.MoveGroupCommander('left_arm')
right_arm = moveit_commander.MoveGroupCommander('right_arm')
left_arm.set_planner_id('RRTConnectkConfigDefault')
left_arm.set_planning_time(1)
right_arm.set_planner_id('RRTConnectkConfigDefault')
right_arm.set_planning_time(1)
listener = tf.TransformListener()

#Initialize the "last position" variables
#This is used to eliminating the tremble of the arm
last_x = 99.0
last_y = 99.0
last_z = 99.0
threshold = 0.05

#Callback Function
def callback(message):
    global last_x 
    global last_y 
    global last_z 
    global error   
    try:
        if message.transforms[0].child_frame_id == 'ar_marker_23':
            #Read the position of artag
            (trans,rot) = listener.lookupTransform('/base', '/ar_marker_23', rospy.Time(0)) 
            #Execute only when the difference of the current position and the last position exceed the threshold
            if ((abs(trans[0]-last_x) < threshold) and (abs(trans[1]-last_y) < threshold) and (abs(trans[2]-last_z) < threshold)):
                pass
            else:
                last_x = trans[0]
                last_y = trans[1]
                last_z = trans[2]
                print trans
                
                #Goal position
                goal = PoseStamped()
                goal.header.frame_id = "base"
                
                #x, y, and z position
                goal.pose.position.x = trans[0]
                goal.pose.position.y = trans[1]
                goal.pose.position.z = trans[2]-0.1
                
                #Orientation as a quaternion: default orientation
                goal.pose.orientation.x = 0.5
                goal.pose.orientation.y = 0.5
                goal.pose.orientation.z = 0.5
                goal.pose.orientation.w = -0.5
              
                #Set the goal state
                right_arm.set_pose_target(goal)

                #Set the start state
                right_arm.set_start_state_to_current_state()
                
                #Create a orientation constraint for the arm
                orien_const = OrientationConstraint()
                orien_const.link_name = "right_gripper";
                orien_const.header.frame_id = "base";
                orien_const.orientation.x = 0.5;
                orien_const.orientation.y = 0.5;
                orien_const.orientation.z = 0.5;
                orien_const.orientation.w = -0.5;
                orien_const.absolute_x_axis_tolerance = 0.1;
                orien_const.absolute_y_axis_tolerance = 0.1;
                orien_const.absolute_z_axis_tolerance = 0.1;
                orien_const.weight = 1.0;
                consts = Constraints()
                consts.orientation_constraints = [orien_const]
                right_arm.set_path_constraints(consts)
                
                #Plan a path
                right_plan = right_arm.plan()

                #Execute the plan
                right_arm.execute(right_plan)      
        else:
            pass   
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        print 'exception'

#Start position function
#Left arm is straight down by the side of the body
#Right arm holding the sword right in front of the body
#Use button to control the gripper (Round Button: open, Dash Button: close)
def start_position():
    #Left arm starting position
    goal_left = PoseStamped()
    goal_left.header.frame_id = "base"
    
    #x, y, and z position
    goal_left.pose.position.x = 0.0
    goal_left.pose.position.y = 0.5
    goal_left.pose.position.z = -0.6
    
    #Orientation as a quaternion: pointing straight down
    goal_left.pose.orientation.x = 0.0
    goal_left.pose.orientation.y = 1.0
    goal_left.pose.orientation.z = 0.0
    goal_left.pose.orientation.w = 0.0
    
    #Set the goal state
    left_arm.set_pose_target(goal_left)
    
    #Set the start state
    left_arm.set_start_state_to_current_state()

    #Plan a path
    left_plan = left_arm.plan()

    #Right arm starting position
    goal_right = PoseStamped()
    goal_right.header.frame_id = "base"

    #x, y, and z position
    goal_right.pose.position.x = 0.5
    goal_right.pose.position.y = -0.1
    goal_right.pose.position.z = 0.0
    
    #Orientation as a quaternion: pointing straight left (default)
    goal_right.pose.orientation.x = 0.5
    goal_right.pose.orientation.y = 0.5
    goal_right.pose.orientation.z = 0.5
    goal_right.pose.orientation.w = -0.5

    #Set the goal state
    right_arm.set_pose_target(goal_right)

    #Set the start state
    right_arm.set_start_state_to_current_state()

    #Plan a path
    right_plan = right_arm.plan()

    #Execute both plan
    left_arm.execute(left_plan)
    right_arm.execute(right_plan)
    
def main():
    raw_input('Press <Enter> to start program: ')
    start_position()
    rospy.Subscriber("tf", tfMessage, callback)
    rospy.spin()
   
if __name__ == '__main__':
    main()
