#!/usr/bin/env python

import rospy

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

def test():
    pubJoint = rospy.Publisher('/typea/joint_trajectory_controller/command', JointTrajectory, queue_size=10)
    rospy.init_node('bioloid_command_test', anonymous=True)

    rate = rospy.Rate(20) # 2hz
    rospy.loginfo("\33[96mBioloid Command Test\33[0m")
    joint = JointTrajectory()
    points = JointTrajectoryPoint()
    nameJoints = ['l_shoulder_swing_joint', 'l_shoulder_lateral_joint', 'l_elbow_joint', 
                  'r_shoulder_swing_joint','r_shoulder_lateral_joint', 'r_elbow_joint', 
                  'l_hip_twist_joint', 'l_hip_lateral_joint','l_hip_swing_joint', 
                  'l_knee_joint', 'l_ankle_swing_joint', 'l_ankle_lateral_joint', 
                  'r_hip_twist_joint','r_hip_lateral_joint', 'r_hip_swing_joint', 
                  'r_knee_joint', 'r_ankle_swing_joint', 'r_ankle_lateral_joint']
    joint.joint_names = nameJoints
    
    _actionArmDown = [2,-2,1,2,2,1,0,-0.4,0,0,0,0,0,0,0,0,0,0]
    _actionArmUp = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
    arm_state = "Up"
    rospy.loginfo("\33[96mStarting Arm Up\33[0m")
    switch_timer = rospy.Time.now()
    while not rospy.is_shutdown():
        if rospy.Time.now() - switch_timer > rospy.Duration(2):
            switch_timer = rospy.Time.now()
            if arm_state == "Up":
                arm_state = "Down"
                rospy.loginfo("\33[96mArm Down\33[0m")
            elif arm_state == "Down":
                arm_state = "Up"
                rospy.loginfo("\33[96mArm Up\33[0m")
        if arm_state == "Down":
            points.positions = _actionArmDown
        elif arm_state == "Up":
            points.positions = _actionArmUp
        points.time_from_start = rospy.Duration(0.01, 0)
        joint.points = [points]
        joint.header.stamp = rospy.Time.now()
        #print joint
        pubJoint.publish(joint)
        # Time Sync
        rate.sleep()

if __name__ == '__main__':
    try:
        test()
    except rospy.ROSInterruptException:
        pass