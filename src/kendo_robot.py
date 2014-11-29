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

# initialize last position
last_x = 99.0
last_y = 99.0
last_z = 99.0
error = 0.02

def callback(message):

    global last_x 
    global last_y 
    global last_z 
    global error 
        
    try:
       if message.transforms[0].child_frame_id == 'ar_marker_23':
        #print(rospy.get_name() + " Message: %s" % (message.transforms[0].transform.translation) )

        #read position of artag
            (trans,rot) = listener.lookupTransform('/base', '/ar_marker_23', rospy.Time(0))
            # may be useful in the future
            #(trans_head_camera,rot_head_camera) = listener.lookupTransform('/corrected_head', '/ar_marker_23', rospy.Time(0))
              
              
            if ((abs(trans[0]-last_x)<error) and (abs(trans[1]-last_y)<error)):
                pass
            else:
                last_x = trans[0]
                last_y = trans[1]
                last_z = trans[2]
                
                x = trans[0]
                # to avoid blocking the ar_Tag
                if trans[1] > 0:
                   y = trans[1] + 0.04
                else:
                   y = trans[1] - 0.04
                z = trans[2]
                
                print trans
                
                #x = float(message.transforms[0].transform.translation.x)
                #y = float(message.transforms[0].transform.translation.y) 
                #z = float(message.transforms[0].transform.translation.z)
                #print x
                #print y
                #print z
                
                goal = PoseStamped()
                goal.header.frame_id = "base"
                
                #goal position of gripper
                goal.pose.position.x = x
                goal.pose.position.y = y
                goal.pose.position.z = 0.0
                
                #Orientation as a quaternion
                goal.pose.orientation.x = 0.5
                goal.pose.orientation.y = 0.5
                goal.pose.orientation.z = 0.5
                goal.pose.orientation.w = -0.5
                
                '''goal.pose.orientation.x = 0.70711
                goal.pose.orientation.y = 0
                goal.pose.orientation.z = 0.70711
                goal.pose.orientation.w = 0'''

                #Set the goal state to the pose you just defined
                right_arm.set_pose_target(goal)

                #Set the start state for the left arm
                right_arm.set_start_state_to_current_state()

                #Plan a path
                right_plan = right_arm.plan()

                #Execute the plan
                right_arm.execute(right_plan) 

    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        print 'exception'
        
            
         

def start_position():
    #Left arm starting position
    goal_left = PoseStamped()
    goal_left.header.frame_id = "base"

    #x, y, and z position
    goal_left.pose.position.x = 0.0
    goal_left.pose.position.y = 0.5
    goal_left.pose.position.z = -0.6
    
    #Orientation as a quaternion
    goal_left.pose.orientation.x = 0.0
    goal_left.pose.orientation.y = -1.0
    goal_left.pose.orientation.z = 0.0
    goal_left.pose.orientation.w = 0.0
    
    #Set the goal state to the pose you just defined
    left_arm.set_pose_target(goal_left)

    #Set the start state for the arm
    left_arm.set_start_state_to_current_state()

    #Plan a path
    left_plan = left_arm.plan()

    #Right arm starting position
    goal_right = PoseStamped()
    goal_right.header.frame_id = "base"

    #x, y, and z position
    goal_right.pose.position.x = 0.5
    goal_right.pose.position.y = -0.2
    goal_right.pose.position.z = 0.0
    
    #Orientation as a quaternion
    goal_right.pose.orientation.x = 0.5
    goal_right.pose.orientation.y = 0.5
    goal_right.pose.orientation.z = 0.5
    goal_right.pose.orientation.w = -0.5

    #Set the goal state to the pose you just defined
    right_arm.set_pose_target(goal_right)

    #Set the start state for the arm
    right_arm.set_start_state_to_current_state()

    #Plan a path
    right_plan = right_arm.plan()

    #Execute both plan
    left_arm.execute(left_plan)
    right_arm.execute(right_plan)
    
def end_position():
    #Left arm starting position
    goal_left = PoseStamped()
    goal_left.header.frame_id = "base"

    #x, y, and z position
    goal_left.pose.position.x = 0.0
    goal_left.pose.position.y = 0.5
    goal_left.pose.position.z = -0.6
    
    #Orientation as a quaternion
    goal_left.pose.orientation.x = 0.0
    goal_left.pose.orientation.y = -1.0
    goal_left.pose.orientation.z = 0.0
    goal_left.pose.orientation.w = 0.0
    
    #Set the goal state to the pose you just defined
    left_arm.set_pose_target(goal_left)

    #Set the start state for the arm
    left_arm.set_start_state_to_current_state()

    #Plan a path
    left_plan = left_arm.plan()

    #Right arm starting position
    goal_right = PoseStamped()
    goal_right.header.frame_id = "base"

    #x, y, and z position
    goal_right.pose.position.x = 0.0
    goal_right.pose.position.y = -0.5
    goal_right.pose.position.z = -0.6
    
    #Orientation as a quaternion
    goal_right.pose.orientation.x = 0.0
    goal_right.pose.orientation.y = -1.0
    goal_right.pose.orientation.z = 0.0
    goal_right.pose.orientation.w = 0.0

    #Set the goal state to the pose you just defined
    right_arm.set_pose_target(goal_right)

    #Set the start state for the arm
    right_arm.set_start_state_to_current_state()

    #Plan a path
    right_plan = right_arm.plan()

    #Execute both plan
    left_arm.execute(left_plan)
    right_arm.execute(right_plan)  
    
def main():


    raw_input('Press <Enter> to move to start program: ')
    start_position()
    
    rospy.Subscriber("tf", tfMessage, callback)
    rospy.spin()

       
       
if __name__ == '__main__':
    try:
       main()
    # keyboardinterrupt doesnt work here
    except KeyboardInterrupt:
       print "end"
       #end_position()
       pass
