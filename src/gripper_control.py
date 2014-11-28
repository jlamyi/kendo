#!/usr/bin/env python
# Class limb is defined in 'baxter_interface/src/baxter_interface/limb.py'
import sys
import rospy
import baxter_interface
from baxter_interface import CHECK_VERSION

def main():
	rospy.init_node("gripper_control")
	rs = baxter_interface.RobotEnable(CHECK_VERSION)
	if rs.state().enabled:
		setup_buttons()
		print("Gripper Publisher Enabled.")
		rospy.spin()
	else:
		print("Active the subscriber first.")

def setup_buttons():
	gripper_left = baxter_interface.Gripper("left")
	gripper_right = baxter_interface.Gripper("right")
	button_left_thin = baxter_interface.DigitalIO('left_upper_button')
	button_right_thin = baxter_interface.DigitalIO('right_upper_button')
	button_left_thin.state_changed.connect(left_pressed)
	button_right_thin.state_changed.connect(right_pressed)
	if gripper_left.error():
		gripper_left.reset()
	if gripper_right.error():
		gripper_right.reset()
	if (not gripper_left.calibrated() and
		gripper_left.type() != 'custom'):
		gripper_left.calibrate()
	if (not gripper_right.calibrated() and
		gripper_right.type() != 'custom'):
		gripper_right.calibrate()
	
def left_pressed(value):
	if value == False:
		gripper = baxter_interface.Gripper("left")
		reverse_gripper(gripper)

def right_pressed(value):
	if value == False:
		gripper = baxter_interface.Gripper("right")
		reverse_gripper(gripper)

def reverse_gripper(gripper):
	if gripper.position() > 90:
		gripper.close()
	else:
		gripper.open()

# Main
if __name__ == '__main__':
	main()
