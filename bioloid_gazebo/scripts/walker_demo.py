#!/usr/bin/env python

import rospy
from typea_gazebo.typea import TypeA


if __name__=="__main__":
    rospy.init_node("walker_demo")
    
    rospy.loginfo("Instantiating TypeA Client")
    typea=TypeA()
    rospy.sleep(1)
 
    rospy.loginfo("TypeA Walker Demo Starting")

    typea.set_walk_velocity(0.2,0,0)
    rospy.sleep(30)
    typea.set_walk_velocity(1,0,0)
    rospy.sleep(30)
    #typea.set_walk_velocity(0,1,0)
    #rospy.sleep(3)
    #typea.set_walk_velocity(-1,0,0)
    #rospy.sleep(3)
    #typea.set_walk_velocity(0,-1,0)
    #rospy.sleep(3)
    #typea.set_walk_velocity(1,1,0)
    #rospy.sleep(3)
    #typea.set_walk_velocity(-1,-1,0)
    #rospy.sleep(3)
    typea.set_walk_velocity(0,0,0)

    rospy.loginfo("TypeA Walker Demo Finished")
