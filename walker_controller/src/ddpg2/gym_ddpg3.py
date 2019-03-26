# ddpg that works.

import filter_env
from ddpg import *
import gc
gc.enable()


import roslib
import rospy
import rostopic
import random
import time
import math
import csv
from std_srvs.srv import Empty
from gazebo_msgs.srv import SetModelConfiguration

from control_msgs.msg import JointControllerState
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from gazebo_msgs.msg import LinkStates
from gazebo_msgs.msg import ContactsState
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64
from std_msgs.msg import String
from sensor_msgs.msg import Joy

import threading
from scipy.interpolate import interp1d

ENV_NAME = 'BipedalWalker-v2'
EPISODES = 500000
TEST = 10

# src_file = 'walking-data-preprocessed-3.csv'
# src_file = "along-z-5-preprocessed.csv"
src_file = "navneet-walks-for-biped.csv"
reward_file = "reward_file.csv"
trajectory_file = "trajectory_file.csv"
# src_file = "walking-data-march.csv"
# src_file = "walking-data-ga-2.csv"

# pubHipR = rospy.Publisher('/waist_thighR_position_controller/command', Float64, queue_size=10)
# pubHipL = rospy.Publisher('/waist_thighL_position_controller/command', Float64, queue_size=10)
# pubKneeR = rospy.Publisher('/thighR_shankR_position_controller/command', Float64, queue_size=10)
# pubKneeL = rospy.Publisher('/thighL_shankL_position_controller/command', Float64, queue_size=10)
pubJoint = rospy.Publisher('/typea/joint_trajectory_controller/command', JointTrajectory, queue_size=10)

reset_simulation = rospy.ServiceProxy('/gazebo/reset_world', Empty)
reset_joints = rospy.ServiceProxy('/gazebo/set_model_configuration', SetModelConfiguration)
unpause = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
pause = rospy.ServiceProxy('/gazebo/pause_physics', Empty)

fall = 0


rospy.init_node('walker_control_script')
rate = rospy.Rate(50)

class RobotState(object):
    def __init__(self):
        # self.waist_z = 0.0
        # self.waist_y = 0.0
        # self.outer_ring_inner_ring_theta = 0.0
        # self.hipr_theta = 0.0
        # self.hipr_theta_dot = 0.0
        # self.hipl_theta = 0.0
        # self.hipl_theta_dot = 0.0
        # self.kneer_theta = 0.0
        # self.kneer_theta_dot = 0.0
        # self.kneel_theta = 0.0
        # self.kneel_theta_dot = 0.0
        # self.vel_y = 0.0
        # self.vel_z = 0.0
        # self.footr_contact = 0
        # self.footl_contact = 0
        # self.robot_state = [self.vel_y, self.vel_z, self.hipr_theta, self.hipr_theta_dot, self.hipl_theta, self.hipl_theta_dot, \
        # self.kneer_theta, self.kneer_theta_dot, self.kneel_theta, self.kneel_theta_dot, self.footr_contact, self.footl_contact]

        self.latest_reward = 0.0
        self.best_reward = -100000000000000.0
        self.episode = 0
        self.last_outer_ring_inner_ring_theta = 0.0
        self.last_time = 0.0

        self.fall = 0
        self.done = False
        self.count_of_1 = 0
        self.avg_reward = 0.0
        

        #Bruna
        self.l_ankle_swing_joint_pos =0
        self.l_hip_swing_joint_pos =0
        self.l_knee_joint_pos =0
        self.r_ankle_swing_joint_pos =0
        self.r_hip_swing_joint_pos =0
        self.r_knee_joint_pos =0
        self.l_ankle_swing_joint_vel =0
        self.l_hip_swing_joint_vel =0
        self.l_knee_joint_vel =0
        self.r_ankle_swing_joint_vel =0
        self.r_hip_swing_joint_vel =0
        self.r_knee_joint_vel =0

        self.l_shoulder_swing_joint_pos=0
        self.l_shoulder_lateral_joint_pos =0
        self.l_elbow_joint_pos=0
        self.r_shoulder_swing_joint_pos=0
        self.r_shoulder_lateral_joint_pos=0 
        self.r_elbow_joint_pos =0
        self.l_hip_twist_joint_pos =0
        self.l_hip_lateral_joint_pos =0
        self.l_ankle_lateral_joint_pos=0 
        self.r_hip_twist_joint_pos=0
        self.r_hip_lateral_joint_pos =0
        self.r_ankle_lateral_joint_pos=0
        self.l_shoulder_swing_joint_vel=0
        self.l_shoulder_lateral_joint_vel =0
        self.l_elbow_joint_vel=0
        self.r_shoulder_swing_joint_vel=0
        self.r_shoulder_lateral_joint_vel=0 
        self.r_elbow_joint_vel =0
        self.l_hip_twist_joint_vel =0
        self.l_hip_lateral_joint_vel =0
        self.l_ankle_lateral_joint_vel=0 
        self.r_hip_twist_joint_vel=0
        self.r_hip_lateral_joint_vel =0
        self.r_ankle_lateral_joint_vel=0
        self.data=None

        self.footr_contact = 0
        self.footl_contact = 0
        self.vel_y_imu = 0.0
        self.vel_z_imu = 0.0
        self.pos_x_odom = 0.0
        self.pos_z_odom = 0.0
        self.outer_ring_inner_ring_theta = 0.0
        #self.robot_state = [self.vel_y_imu, self.vel_z_imu, self.l_ankle_swing_joint_pos, self.l_hip_swing_joint_pos, self.l_knee_joint_pos, self.r_ankle_swing_joint_pos, \
        #   self.r_hip_swing_joint_pos, self.r_knee_joint_pos, self.l_ankle_swing_joint_vel, self.l_hip_swing_joint_vel,self.l_knee_joint_vel,self.r_ankle_swing_joint_vel,self.r_hip_swing_joint_vel,self.r_knee_joint_vel, self.footr_contact, self.footl_contact,self.pos_x_odom,self.pos_z_odom]
        self.robot_state = [self.vel_y_imu, self.vel_z_imu, self.l_shoulder_swing_joint_pos, self.l_shoulder_lateral_joint_pos, self.l_elbow_joint_pos, self.r_shoulder_swing_joint_pos,
  self.r_shoulder_lateral_joint_pos, self.r_elbow_joint_pos, self.l_hip_twist_joint_pos, self.l_hip_lateral_joint_pos,
  self.l_hip_swing_joint_pos, self.l_knee_joint_pos, self.l_ankle_swing_joint_pos, self.l_ankle_lateral_joint_pos, self.r_hip_twist_joint_pos,
  self.r_hip_lateral_joint_pos, self.r_hip_swing_joint_pos, self.r_knee_joint_pos, self.r_ankle_swing_joint_pos, self.r_ankle_lateral_joint_pos,self.l_shoulder_swing_joint_vel, self.l_shoulder_lateral_joint_vel, self.l_elbow_joint_vel, self.r_shoulder_swing_joint_vel,
  self.r_shoulder_lateral_joint_vel, self.r_elbow_joint_vel, self.l_hip_twist_joint_vel, self.l_hip_lateral_joint_vel,
  self.l_hip_swing_joint_vel, self.l_knee_joint_vel, self.l_ankle_swing_joint_vel, self.l_ankle_lateral_joint_vel, self.r_hip_twist_joint_vel,
  self.r_hip_lateral_joint_vel, self.r_hip_swing_joint_vel, self.r_knee_joint_vel, self.r_ankle_swing_joint_vel, self.r_ankle_lateral_joint_vel, self.footr_contact, self.footl_contact,self.pos_x_odom,self.pos_z_odom]

class Publisher(threading.Thread):
    #def __init__(self, pubHipR, pubHipL, pubKneeR, pubKneeL, rate):
    def __init__(self, pubJoint, rate):
        threading.Thread.__init__(self)
        self.counter = 0
        # self.pubHipR = pubHipR
        # self.pubHipL = pubHipL
        # self.pubKneeR = pubKneeR
        # self.pubKneeL = pubKneeL
        self.pubJoint = pubJoint
        self.rate = rate


    def run(self):
        publisher(self.pubJoint, self.rate, self.counter)

robot_state = RobotState()


def reset():
    # ['waist_thighR', 'waist_thighL', 'thighR_shankR', 'thighL_shankL', 'outer_ring_inner_ring', 'inner_ring_boom', 'boom_waist']
    rospy.wait_for_service('gazebo/reset_world')
    try:
        reset_simulation()
    except(rospy.ServiceException) as e:
        print "reset_world failed!"


    rospy.wait_for_service('gazebo/set_model_configuration')

    try:
        reset_joints("typea", "robot_description", ['r_shoulder_swing_joint', 'l_shoulder_swing_joint', 'r_shoulder_lateral_joint', 'l_shoulder_lateral_joint',  'r_elbow_joint', 'l_elbow_joint', 'r_hip_twist_joint', 'l_hip_twist_joint', 'r_hip_lateral_joint',  'l_hip_lateral_joint', 'r_hip_swing_joint', 'l_hip_swing_joint', 'r_knee_joint', 'l_knee_joint',  'r_ankle_swing_joint', 'l_ankle_swing_joint', 'r_ankle_lateral_joint', 'l_ankle_lateral_joint'], [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        robot_state.last_outer_ring_inner_ring_theta = 0.0
    except (rospy.ServiceException) as e:
        print "reset_joints failed!"

    rospy.wait_for_service('/gazebo/pause_physics')
    try:
        pause()
    except (rospy.ServiceException) as e:
        print "rospause failed!'"

    set_robot_state()

    # print "called reset()"



def set_robot_state():
    #robot_state.robot_state = [robot_state.vel_y_imu, robot_state.vel_z_imu, robot_state.l_ankle_swing_joint_pos, robot_state.l_hip_swing_joint_pos, robot_state.l_knee_joint_pos, robot_state.r_ankle_swing_joint_pos, robot_state.r_hip_swing_joint_pos, robot_state.r_knee_joint_pos, robot_state.l_ankle_swing_joint_vel,	robot_state.l_hip_swing_joint_vel,robot_state.l_knee_joint_vel,robot_state.r_ankle_swing_joint_vel,robot_state.r_hip_swing_joint_vel,robot_state.r_knee_joint_vel, robot_state.footr_contact,robot_state.footl_contact,robot_state.pos_x_odom,robot_state.pos_z_odom]
    robot_state.robot_state = [robot_state.vel_y_imu, robot_state.vel_z_imu, robot_state.l_shoulder_swing_joint_pos, robot_state.l_shoulder_lateral_joint_pos, robot_state.l_elbow_joint_pos, robot_state.r_shoulder_swing_joint_pos,
        robot_state.r_shoulder_lateral_joint_pos, robot_state.r_elbow_joint_pos, robot_state.l_hip_twist_joint_pos, robot_state.l_hip_lateral_joint_pos,
        robot_state.l_hip_swing_joint_pos, robot_state.l_knee_joint_pos, robot_state.l_ankle_swing_joint_pos, robot_state.l_ankle_lateral_joint_pos, robot_state.r_hip_twist_joint_pos,
        robot_state.r_hip_lateral_joint_pos, robot_state.r_hip_swing_joint_pos, robot_state.r_knee_joint_pos, robot_state.r_ankle_swing_joint_pos, robot_state.r_ankle_lateral_joint_pos,robot_state.l_shoulder_swing_joint_vel, robot_state.l_shoulder_lateral_joint_vel, robot_state.l_elbow_joint_vel, robot_state.r_shoulder_swing_joint_vel,
        robot_state.r_shoulder_lateral_joint_vel, robot_state.r_elbow_joint_vel, robot_state.l_hip_twist_joint_vel, robot_state.l_hip_lateral_joint_vel,
        robot_state.l_hip_swing_joint_vel, robot_state.l_knee_joint_vel, robot_state.l_ankle_swing_joint_vel, robot_state.l_ankle_lateral_joint_vel, robot_state.r_hip_twist_joint_vel,
        robot_state.r_hip_lateral_joint_vel, robot_state.r_hip_swing_joint_vel, robot_state.r_knee_joint_vel, robot_state.r_ankle_swing_joint_vel, robot_state.r_ankle_lateral_joint_vel, robot_state.footr_contact, robot_state.footl_contact,robot_state.pos_x_odom,robot_state.pos_z_odom]
def take_action(action):
    rospy.wait_for_service('/gazebo/unpause_physics')

    try:
        unpause()
    except (rospy.ServiceException) as e:
        print "/gazebo/pause_physics service call failed"

    # pubHipR.publish(action[0])
    # pubKneeR.publish(action[1])
    # pubHipL.publish(action[2])
    # pubKneeL.publish(action[3])
    joint = JointTrajectory()
    points = JointTrajectoryPoint()
    nameJoints = ['l_shoulder_swing_joint', 'l_shoulder_lateral_joint', 'l_elbow_joint', 
                  'r_shoulder_swing_joint','r_shoulder_lateral_joint', 'r_elbow_joint', 
                  'l_hip_twist_joint', 'l_hip_lateral_joint','l_hip_swing_joint', 
                  'l_knee_joint', 'l_ankle_swing_joint', 'l_ankle_lateral_joint', 
                  'r_hip_twist_joint','r_hip_lateral_joint', 'r_hip_swing_joint', 
                  'r_knee_joint', 'r_ankle_swing_joint', 'r_ankle_lateral_joint']
    newAction = action
    #newAction = [0,0,0,0,0,0,0,0,0,action[0],action[1],action[2],0,0,0,action[3],action[4],action[5]]
    points.positions = newAction
    points.time_from_start = rospy.Duration(0.01, 0)
    joint.points = [points]
    joint.joint_names = nameJoints
    joint.header.stamp = rospy.Time.now()
    #print joint
    pubJoint.publish(joint)

    if(robot_state.data!= None):
        while(np.max(np.abs(np.array(newAction)-np.array(robot_state.data.position)))>0.1):
            print(np.max(np.abs(np.array(newAction)-np.array(robot_state.data.position))))

    reward = -0.1  # when it used to run, used to be -0.1
    current_time = time.time()
    if (robot_state.outer_ring_inner_ring_theta - robot_state.last_outer_ring_inner_ring_theta) <= 0.01: #-0.001forward     
        delta_time = current_time - robot_state.last_time
        reward += ((robot_state.outer_ring_inner_ring_theta - robot_state.last_outer_ring_inner_ring_theta))*10
    robot_state.last_time = current_time
    robot_state.last_outer_ring_inner_ring_theta = robot_state.outer_ring_inner_ring_theta


    if (robot_state.pos_z_odom) <= 0.18: #-0.001forward        
        reward += -2.0
        #print("odometria")
    # if robot_state.waist_z < -0.10:
    #     reward += -100
    #     robot_state.done = True
    #     robot_state.fall = 1
    if (robot_state.footr_contact > 60 and robot_state.footl_contact > 60):
        reward += -100
        robot_state.done = True
        robot_state.fall = 1
        #print("\33[91mDROPPED DOWN!\33[0m")

    if robot_state.outer_ring_inner_ring_theta >9.0:
        reward += 100
        robot_state.done = True
        robot_state.fall = 1
        print("\33[92mREACHED TO THE END!\33[0m")
    rate.sleep()
    return reward, robot_state.done


def callbackJointStates(data):
    # ['boom_waist', 'outer_ring_inner_ring', 'thighL_shankL', 'thighR_shankR', 'waist_thighL', 'waist_thighR']
    # if vel == 0 ['waist_thighR', 'waist_thighL', 'thighR_shankR', 'thighL_shankL', 'outer_ring_inner_ring', 'boom_waist']
    robot_state.data = data

    if len(data.velocity)!=0:

        # robot_state.vel_z = data.velocity[0]
        # robot_state.vel_y = data.velocity[1]
        # robot_state.kneel_theta_dot = data.velocity[2]
        # robot_state.kneer_theta_dot = data.velocity[3]
        # robot_state.hipl_theta_dot = data.velocity[4]
        # robot_state.hipr_theta_dot = data.velocity[5]

        # robot_state.waist_z = data.position[0]
        # robot_state.waist_y = data.position[1]
        # robot_state.outer_ring_inner_ring_theta = data.position[1]
        # robot_state.kneel_theta = data.position[2]
        # robot_state.kneer_theta = data.position[3]
        # robot_state.hipl_theta = data.position[4]
        # robot_state.hipr_theta = data.position[5]

        #perna
        robot_state.l_ankle_swing_joint_pos =data.position[10]
        robot_state.l_hip_swing_joint_pos =data.position[8]
        robot_state.l_knee_joint_pos =data.position[9]
        robot_state.r_ankle_swing_joint_pos =data.position[16]
        robot_state.r_hip_swing_joint_pos =data.position[14]
        robot_state.r_knee_joint_pos =data.position[15]
        robot_state.l_ankle_swing_joint_vel =data.velocity[10]
        robot_state.l_hip_swing_joint_vel =data.velocity[8]
        robot_state.l_knee_joint_vel =data.velocity[9]
        robot_state.r_ankle_swing_joint_vel =data.velocity[16]
        robot_state.r_hip_swing_joint_vel =data.velocity[14]
        robot_state.r_knee_joint_vel =data.velocity[15]


        #restantepubJoint
        robot_state.l_shoulder_swing_joint_pos=data.position[0]
        robot_state.l_shoulder_lateral_joint_pos =data.position[1]
        robot_state.l_elbow_joint_pos=data.position[2]
        robot_state.r_shoulder_swing_joint_pos=data.position[3]
        robot_state.r_shoulder_lateral_joint_pos=data.position[4] 
        robot_state.r_elbow_joint_pos =data.position[5]
        robot_state.l_hip_twist_joint_pos =data.position[6]
        robot_state.l_hip_lateral_joint_pos =data.position[7]
        robot_state.l_ankle_lateral_joint_pos=data.position[11] 
        robot_state.r_hip_twist_joint_pos=data.position[12]
        robot_state.r_hip_lateral_joint_pos =data.position[13]
        robot_state.r_ankle_lateral_joint_pos=data.position[17]
        robot_state.l_shoulder_swing_joint_vel=data.velocity[0]
        robot_state.l_shoulder_lateral_joint_vel =data.velocity[1]
        robot_state.l_elbow_joint_vel=data.velocity[2]
        robot_state.r_shoulder_swing_joint_vel=data.velocity[3]
        robot_state.r_shoulder_lateral_joint_vel=data.velocity[4]
        robot_state.r_elbow_joint_vel =data.velocity[5]
        robot_state.l_hip_twist_joint_vel =data.velocity[6]
        robot_state.l_hip_lateral_joint_vel =data.velocity[7]
        robot_state.l_ankle_lateral_joint_vel=data.velocity[11]
        robot_state.r_hip_twist_joint_vel=data.velocity[12]
        robot_state.r_hip_lateral_joint_vel =data.velocity[13]
        robot_state.r_ankle_lateral_joint_vel=data.velocity[17]

    else:
        # robot_state.vel_z = 0
        # robot_state.vel_y = 0
        # robot_state.kneel_theta_dot = 0
        # robot_state.kneer_theta_dot = 0
        # robot_state.hipl_theta_dot = 0
        # robot_state.hipr_theta_dot = 0

        # robot_state.waist_z = 0
        # robot_state.waist_y = 0
        # robot_state.outer_ring_inner_ring_theta = 0
        # robot_state.kneel_theta = 0
        # robot_state.kneer_theta = 0
        # robot_state.hipl_theta = 0
        # robot_state.hipr_theta = 0
        #pernas
        robot_state.l_ankle_swing_joint_pos =0
        robot_state.l_hip_swing_joint_pos =0
        robot_state.l_knee_joint_pos =0
        robot_state.r_ankle_swing_joint_pos =0
        robot_state.r_hip_swing_joint_pos =0
        robot_state.r_knee_joint_pos =0
        robot_state.l_ankle_swing_joint_vel =0
        robot_state.l_hip_swing_joint_vel =0
        robot_state.l_knee_joint_vel =0
        robot_state.r_ankle_swing_joint_vel =0
        robot_state.r_hip_swing_joint_vel =0
        robot_state.r_knee_joint_vel =0


        #restante
        robot_state.l_shoulder_swing_joint_pos=0
        robot_state.l_shoulder_lateral_joint_pos =0
        robot_state.l_elbow_joint_pos=0
        robot_state.r_shoulder_swing_joint_pos=0
        robot_state.r_shoulder_lateral_joint_pos=0 
        robot_state.r_elbow_joint_pos =0
        robot_state.l_hip_twist_joint_pos =0
        robot_state.l_hip_lateral_joint_pos =0
        robot_state.l_ankle_lateral_joint_pos=0 
        robot_state.r_hip_twist_joint_pos=0
        robot_state.r_hip_lateral_joint_pos =0
        robot_state.r_ankle_lateral_joint_pos=0
        robot_state.l_shoulder_swing_joint_vel=0
        robot_state.l_shoulder_lateral_joint_vel =0
        robot_state.l_elbow_joint_vel=0
        robot_state.r_shoulder_swing_joint_vel=0
        robot_state.r_shoulder_lateral_joint_vel=0 
        robot_state.r_elbow_joint_vel =0
        robot_state.l_hip_twist_joint_vel =0
        robot_state.l_hip_lateral_joint_vel =0
        robot_state.l_ankle_lateral_joint_vel=0 
        robot_state.r_hip_twist_joint_vel=0
        robot_state.r_hip_lateral_joint_vel =0
        robot_state.r_ankle_lateral_joint_vel=0


    set_robot_state()
    # rate.sleep()


def callbackSub(data):
    set_robot_state()

def callbackContactShankR(data):
    if not data.states:
        #robot_state.footr_contact = 0
        robot_state.footr_contact += 1
    else:
        robot_state.footr_contact = 0

def callbackVelImu(data):
    robot_state.vel_y_imu = data.orientation.y
    robot_state.vel_z_imu = data.orientation.z
 
def callbackVelOdom(data):
    robot_state.pos_x_odom = data.pose.pose.position.x
    robot_state.outer_ring_inner_ring_theta=data.pose.pose.position.x
    robot_state.pos_z_odom = data.pose.pose.position.z

def callbackContactShankL(data):
    if not data.states:
        robot_state.footl_contact += 1
    else:
        robot_state.footl_contact = 0

def listener():
    print "listener"

    rospy.Subscriber("/typea/joint_states", JointState, callbackJointStates)
    rospy.Subscriber("/typea/footr_contact_sensor_state", ContactsState, callbackContactShankR)
    rospy.Subscriber("/typea/footl_contact_sensor_state", ContactsState, callbackContactShankL)
    rospy.Subscriber("/typea/imu", Imu, callbackVelImu)
    rospy.Subscriber("/typea/odom", Odometry, callbackVelOdom)


def publisher(pubHipR, rate, counter):

    while not rospy.is_shutdown():
        # writing rewards in the csv file
        file = open(reward_file, 'wt')
        writer = csv.writer(file)
        writer.writerow(['avg_reward'])

        #env = [18, 6] # [state_dim 12, action_dim 4]
        env = [42, 18] 
        agent = DDPG(env)
        for episode in range(EPISODES):
            reset()
            state = robot_state.robot_state
            # Train
   
            for steps in range(1600):
                action = agent.noise_action(state)
                reward,done = take_action(action)
                next_state = robot_state.robot_state
                agent.perceive(state,action,reward,next_state,done)
                state = next_state
                if done:
                    robot_state.done = False
                    break
            robot_state.episode = episode
            # Testing:
            if episode % 100 == 0 and episode > 100:
                traj_file = open(trajectory_file, 'wt')
                traj_writer = csv.writer(traj_file, delimiter='\t')
                traj_writer.writerow(['r_shoulder_swing_joint', 'l_shoulder_swing_joint', 'r_shoulder_lateral_joint', 'l_shoulder_lateral_joint',  'r_elbow_joint', 'l_elbow_joint', 'r_hip_twist_joint', 'l_hip_twist_joint', 'r_hip_lateral_joint',  'l_hip_lateral_joint', 'r_hip_swing_joint', 'l_hip_swing_joint', 'r_knee_joint', 'l_knee_joint',  'r_ankle_swing_joint', 'l_ankle_swing_joint', 'r_ankle_lateral_joint', 'l_ankle_lateral_joint'])

                print "testing"
                total_reward = 0
                count_of_1 = 0
                for i in range(TEST):
                    reset()
                    print "\33[96mTesting", i, "\33[0m"
                    state = robot_state.robot_state
                    for steps in range(1600):
                        action = agent.action(state) # direct action for test
                        reward,done = take_action(action)
                        state = robot_state.robot_state
                        traj_writer.writerow([robot_state.vel_y_imu, robot_state.vel_z_imu, robot_state.l_ankle_swing_joint_pos, robot_state.l_hip_swing_joint_pos, robot_state.l_knee_joint_pos, robot_state.r_ankle_swing_joint_pos, \
robot_state.r_hip_swing_joint_pos, robot_state.r_knee_joint_pos, robot_state.l_ankle_swing_joint_vel,robot_state.l_hip_swing_joint_vel,robot_state.l_knee_joint_vel,robot_state.r_ankle_swing_joint_vel,robot_state.r_hip_swing_joint_vel,robot_state.r_knee_joint_vel, robot_state.footr_contact,robot_state.footl_contact,robot_state.pos_x_odom,robot_state.pos_z_odom])
                        traj_file.flush()
                        if reward == 1:
                            count_of_1 += 1
                        total_reward += reward
                        if done:
                            robot_state.done = False
                            break
                ave_reward = total_reward/TEST
                robot_state.latest_reward = ave_reward
                if ave_reward > robot_state.best_reward:
                    robot_state.best_reward = ave_reward
                robot_state.avg_reward = ave_reward
                writer.writerow([ave_reward])
                file.flush()

                print "episode: ",episode,"Evaluation Average Reward: ",ave_reward
                # print "best_reward: ", robot_state.best_reward

def main():

    # Create new threads
    thread = Publisher(pubJoint, rate)

    # Start new Threads
    thread.start()
    listener()

if __name__ == '__main__':
    main()
