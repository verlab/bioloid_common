# import some modules
import filter_env
from ddpg import *
import gc
gc.enable()

#import math and ros modules
import roslib
import rospy
import rostopic
import random
import time
import math
import csv
from std_srvs.srv import Empty
from gazebo_msgs.srv import SetModelConfiguration

#import some msgs types
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
from geometry_msgs.msg import Point

#import interpolate
from scipy.interpolate import interp1d

#define some macros
ENV_NAME = 'BipedalWalker-v2'
EPISODES = 500000
TEST = 10
RANGE = 1600

class walkerControl():
    def __init__(self):
        rospy.init_node('walkerControl')
        self.srcFile = "navneet-walks-for-biped.csv"
        self.rewardFile = "reward_file.csv"
        self.trajectoryFile = "trajectory_file.csv"
        self.fall = 0
        self.robotState = RobotState()

        #Publish and service
        self.pubJoint = rospy.Publisher('/typea/joint_trajectory_controller/command', JointTrajectory, queue_size=10)
        self.resetSimulation = rospy.ServiceProxy('/gazebo/reset_world', Empty)
        self.resetJoints = rospy.ServiceProxy('/gazebo/set_model_configuration', SetModelConfiguration)
        self.unpause = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
        self.pause = rospy.ServiceProxy('/gazebo/pause_physics', Empty)

        #Listener
        rospy.Subscriber("/typea/joint_states", JointState, self.callbackJointStates)
        rospy.Subscriber("/typea/footr_contact_sensor_state", ContactsState, self.callbackContactShankR)
        rospy.Subscriber("/typea/footl_contact_sensor_state", ContactsState, self.callbackContactShankL)
        rospy.Subscriber("/typea/imu", Imu, self.callbackVelImu)
        rospy.Subscriber("/typea/odom", Odometry, self.callbackVelOdom)

    def callbackJointStates(self, data):
        self.robotState.data = data
        if (len(data.velocity) != 0):

            #Position -> Right
            self.robotState.jointPosition['right']['ankleSwing'] = data.position[16]
            self.robotState.jointPosition['right']['hipSwing'] = data.position[14]
            self.robotState.jointPosition['right']['knee'] = data.position[15]
            self.robotState.jointPosition['right']['shoulderSwing'] = data.position[3]
            self.robotState.jointPosition['right']['shoulderLateral'] = data.position[4]
            self.robotState.jointPosition['right']['elbow'] = data.position[5]
            self.robotState.jointPosition['right']['hipTwist'] = data.position[12]
            self.robotState.jointPosition['right']['hipLateral'] = data.position[13]
            self.robotState.jointPosition['right']['ankleLateral'] = data.position[17]

            #Position -> Left
            self.robotState.jointPosition['left']['ankleSwing'] = data.position[10]
            self.robotState.jointPosition['left']['hipSwing'] = data.position[8]
            self.robotState.jointPosition['left']['knee'] = data.position[9]
            self.robotState.jointPosition['left']['shoulderSwing'] = data.position[0]
            self.robotState.jointPosition['left']['shoulderLateral'] = data.position[1]
            self.robotState.jointPosition['left']['elbow'] = data.position[2]
            self.robotState.jointPosition['left']['hipTwist'] = data.position[6]
            self.robotState.jointPosition['left']['hipLateral'] = data.position[7]
            self.robotState.jointPosition['left']['ankleLateral'] = data.position[11]

            #Velocity -> Right
            self.robotState.jointVelocity['right']['ankleSwing'] = data.velocity[16]
            self.robotState.jointVelocity['right']['hipSwing'] = data.velocity[14]
            self.robotState.jointVelocity['right']['knee'] = data.velocity[15]
            self.robotState.jointVelocity['right']['shoulderSwing'] = data.velocity[3]
            self.robotState.jointVelocity['right']['shoulderLateral'] = data.velocity[4]
            self.robotState.jointVelocity['right']['elbow'] = data.velocity[5]
            self.robotState.jointVelocity['right']['hipTwist'] = data.velocity[12]
            self.robotState.jointVelocity['right']['hipLateral'] = data.velocity[13]
            self.robotState.jointVelocity['right']['ankleLateral'] = data.velocity[17]

            #Velocity -> Left
            self.robotState.jointVelocity['left']['ankleSwing'] = data.velocity[10]
            self.robotState.jointVelocity['left']['hipSwing'] = data.velocity[8]
            self.robotState.jointVelocity['left']['knee'] = data.velocity[9]
            self.robotState.jointVelocity['left']['shoulderSwing'] = data.velocity[0]
            self.robotState.jointVelocity['left']['shoulderLateral'] = data.velocity[1]
            self.robotState.jointVelocity['left']['elbow'] = data.velocity[2]
            self.robotState.jointVelocity['left']['hipTwist'] = data.velocity[6]
            self.robotState.jointVelocity['left']['hipLateral'] = data.velocity[7]
            self.robotState.jointVelocity['left']['ankleLateral'] = data.velocity[11]

        else:
            for i in self.robotState.jointPosition:
                for j in self.robotState.jointPosition[i]:
                    self.robotState.jointPosition[i][j] = 0
            for i in self.robotState.jointVelocity:
                for j in self.robotState.jointVelocity[i]:
                    self.robotState.jointVelocity[i][j] = 0

        self.robotState.robotStateUpdate()

    def callbackContactShankR(self, data):
        if not data.states:
            self.robotState.contactFootR += 1
        else:
            self.robotState.contactFootR = 0

    def callbackContactShankL(self, data):
        if not data.states:
            self.robotState.contactFootL += 1
        else:
            self.robotState.contactFootL = 0

    def callbackVelImu(self, data):
        self.robotState.imuVelY = data.orientation.y
        self.robotState.imuVelZ = data.orientation.z

    def callbackVelOdom(self, data):
        self.robotState.odomPosX = data.pose.pose.position.x
        self.robotState.odometryX = data.pose.pose.position.x
        self.robotState.odomPosZ = data.pose.pose.position.z
        self.robotState.odomRotation = data.pose.pose.orientation

    def reset(self):
        rospy.wait_for_service('/gazebo/reset_world')
        try:
            self.resetSimulation()
        except(rospy.ServiceException) as e:
            rospy.loginfo("[Walker reset]: Reset simulation failed!")

        rospy.wait_for_service('/gazebo/set_model_configuration')
        try:
            self.resetJoints("typea", "robot_description", ['r_shoulder_swing_joint', 'l_shoulder_swing_joint', 'r_shoulder_lateral_joint', 'l_shoulder_lateral_joint',  'r_elbow_joint', 'l_elbow_joint', 'r_hip_twist_joint', 'l_hip_twist_joint', 'r_hip_lateral_joint',  'l_hip_lateral_joint', 'r_hip_swing_joint', 'l_hip_swing_joint', 'r_knee_joint', 'l_knee_joint',  'r_ankle_swing_joint', 'l_ankle_swing_joint', 'r_ankle_lateral_joint', 'l_ankle_lateral_joint'], [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
            self.robotState.lastOdometryX = 0.0
        except (rospy.ServiceException) as e:
            rospy.loginfo("[Walker reset]: Reset model failed!")

        rospy.wait_for_service('/gazebo/pause_physics')
        try:
            self.pause()
        except (rospy.ServiceException) as e:
            rospy.loginfo("[Walker reset]: ROS pause failed!")

        self.robotState.robotStateUpdate()

    def takeAction(self, action):
    
        rospy.wait_for_service('/gazebo/unpause_physics')
        try:
            self.unpause()
            print ("debugX")
        except (rospy.ServiceException) as e:
            rospy.loginfo("[Walker unpause]: ROS unpause failed!")

        joint = JointTrajectory()
        points = JointTrajectoryPoint()
        nameJoints = ['l_shoulder_swing_joint', 'l_shoulder_lateral_joint', 'l_elbow_joint',
                    'r_shoulder_swing_joint','r_shoulder_lateral_joint', 'r_elbow_joint',
                    'l_hip_twist_joint', 'l_hip_lateral_joint','l_hip_swing_joint',
                    'l_knee_joint', 'l_ankle_swing_joint', 'l_ankle_lateral_joint',
                    'r_hip_twist_joint','r_hip_lateral_joint', 'r_hip_swing_joint',
                    'r_knee_joint', 'r_ankle_swing_joint', 'r_ankle_lateral_joint']
        newAction = [0,0,0,0,0,0,0,0,action[0],action[1],action[2],0,0,0,action[2],action[3],action[4],0]
        points.positions = newAction
        points.time_from_start = rospy.Duration(0.01, 0)
        joint.points = [points]
        joint.joint_names = nameJoints
        joint.header.stamp = rospy.Time.now()
        self.pubJoint.publish(joint)
        print ("debugY")

        reward = -0.1 
        currentTime = time.time()

        if ((self.robotState.odometryX - self.robotState.lastOdometryX) >= 0.1) or ((self.robotState.odometryX - self.robotState.lastOdometryX) < 0): #-0.001forward
            deltaTime = currentTime - self.robotState.lastTime
            reward += ((self.robotState.odometryX - self.robotState.lastOdometryX))*30
            print("[Ganho]:", reward)
            print("[Distancia]:", self.robotState.odometryX - self.robotState.lastOdometryX)
       
        self.robotState.lastTime = currentTime
        self.robotState.lastOdometryX = self.robotState.odometryX

        if (self.robotState.contactFootR > 20 and self.robotState.contactFootL > 20 and self.robotState.odomPosZ <= 0.18):
            reward += -100
            self.robotState.done = True
            self.robotState.fall = 1

        if (np.abs(self.robotState.odomRotation.x)>0.2 or np.abs(self.robotState.odomRotation.y)>0.2 or np.abs(self.robotState.odomRotation.z)>0.2):
            reward += -80
            self.robotState.done = True
            self.robotState.fall = 1
        print ("debugB")
        if self.robotState.odometryX > 9.0:
            reward += 100
            self.robotState.done = True
            self.robotState.fall = 1
            print("\33[92mREACHED TO THE END!\33[0m")
        
        self.rate.sleep()
        return reward, self.robotState.done

    def run(self):
        self.rate = rospy.Rate(50)
        while not rospy.is_shutdown():       
           
            # writing rewards in the csv file
            file = open(self.rewardFile, 'wt')
            writer = csv.writer(file)
            writer.writerow(['avg_reward'])
            env = [42, 18]
            agent = DDPG(env)
            for episode in range(EPISODES):
                self.reset()
                state = self.robotState.robot_state
                
                # Train 
                for steps in range(RANGE):
                    action = agent.noise_action(state)
                    reward, done = self.takeAction(action)
                    next_state = self.robotState.robot_state
                    agent.perceive(state, action ,reward ,next_state, done)
                    state = next_state
                    if done:
                        self.robotState.done = False
                        break
                
                self.robotState.episode = episode
                
                # Testing:
                if episode % 100 == 0 and episode > 100:
                    trajFile = open(self.trajectoryFile, 'wt')
                    trajWriter = csv.writer(trajFile, delimiter='\t')
                    trajWriter.writerow(['r_shoulder_swing_joint', 'l_shoulder_swing_joint', 'r_shoulder_lateral_joint', 'l_shoulder_lateral_joint',  'r_elbow_joint', 'l_elbow_joint', 'r_hip_twist_joint', 'l_hip_twist_joint', 'r_hip_lateral_joint',  'l_hip_lateral_joint', 'r_hip_swing_joint', 'l_hip_swing_joint', 'r_knee_joint', 'l_knee_joint',  'r_ankle_swing_joint', 'l_ankle_swing_joint', 'r_ankle_lateral_joint', 'l_ankle_lateral_joint'])
                    totalReward = 0
                    count = 0
                    for i in range(TEST):
                        self.reset()
                        print ("\33[96mTesting", i, "\33[0m")
                        state = self.robotState.robot_state
                        for steps in range(RANGE):
                            action = agent.action(state) # direct action for test
                            reward, done = self.takeAction(action)
                            state = self.robotState.robot_state
                            trajWriter.writerow([self.robotState.imuVelY, self.robotState.imuVelZ, self.robotState.jointPosition['left']['ankleSwing'], self.robotState.jointPosition['left']['hipSwing'], self.robotState.jointPosition['left']['knee'], self.robotState.jointPosition['right']['ankleSwing'], \
                            self.robotState.jointPosition['right']['hipSwing'], self.robotState.jointPosition['right']['knee'], self.robotState.jointVelocity['left']['ankleSwing'], self.robotState.jointVelocity['left']['hipSwing'], self.robotState.jointVelocity['left']['knee'], self.robotState.jointVelocity['right']['ankleSwing'], \
                            self.robotState.jointVelocity['right']['hipSwing'], self.robotState.jointVelocity['right']['knee'], self.robotState.contactFootR, self.robotState.contactFootL, self.robotState.odomPosX, self.robotState.odomPosZ])
                            trajFile.flush()
                            if reward == 1:
                                count += 1
                            totalReward += reward
                            if done:
                                self.robotState.done = False
                                break
                    
                    aveReward = totalReward/TEST
                    self.robotState.latestReward = aveReward
                    if aveReward > self.robotState.bestReward:
                        self.robotState.bestReward = aveReward
                    self.robotState.avgReward = aveReward
                    writer.writerow([aveReward])
                    file.flush()

                    print ("episode: ", episode, "Evaluation Average Reward: ", aveReward)

class RobotState(object):
    def __init__(self):
        self.latestReward = 0.0
        self.bestReward = -100000000000000.0
        self.episode = 0
        self.lastTime = 0.0
        self.fall = 0
        self.done = False
        self.count = 0
        self.avgReward = 0.0
        self.data = None
        self.contactFootR = 0
        self.contactFootL = 0
        self.imuVelY = 0.0
        self.imuVelZ = 0.0
        self.odomPosX = 0.0
        self.odomPosZ = 0.0
        self.odomRotation = Point(0,0,0)
        self.odometryX = 0.0
        self.lastOdometryX = 0.0

        self.jointPosition = {"right" : {"ankleSwing" : 0, "hipSwing" : 0, "knee" : 0, "shoulderSwing" : 0, "shoulderLateral" : 0, "elbow" : 0, "hipTwist" : 0, "hipLateral" : 0, "ankleLateral" : 0},
                               "left" : {"ankleSwing" : 0, "hipSwing" : 0, "knee" : 0, "shoulderSwing" : 0, "shoulderLateral" : 0, "elbow" : 0, "hipTwist" : 0, "hipLateral" : 0, "ankleLateral" : 0}}
        self.jointVelocity = {"right" : {"ankleSwing" : 0, "hipSwing" : 0, "knee" : 0, "shoulderSwing" : 0, "shoulderLateral" : 0, "elbow" : 0, "hipTwist" : 0, "hipLateral" : 0, "ankleLateral" : 0},
                               "left" : {"ankleSwing" : 0, "hipSwing" : 0, "knee" : 0, "shoulderSwing" : 0, "shoulderLateral" : 0, "elbow" : 0, "hipTwist" : 0, "hipLateral" : 0, "ankleLateral" : 0}}
        self.robot_state = []
        self.robotStateUpdate()

    def robotStateUpdate(self):
            self.robot_state = [self.imuVelY, self.imuVelZ, self.jointPosition['left']['shoulderSwing'], self.jointPosition['left']['shoulderLateral'], self.jointPosition['left']['elbow'], self.jointPosition['right']['shoulderSwing'],
            self.jointPosition['right']['shoulderLateral'], self.jointPosition['right']['elbow'], self.jointPosition['left']['hipTwist'], self.jointPosition['left']['hipLateral'],
            self.jointPosition['left']['hipSwing'], self.jointPosition['left']['knee'], self.jointPosition['left']['ankleSwing'], self.jointPosition['left']['ankleLateral'], self.jointPosition['right']['hipTwist'],
            self.jointPosition['right']['hipLateral'], self.jointPosition['right']['hipSwing'], self.jointPosition['right']['knee'], self.jointPosition['right']['ankleSwing'], self.jointPosition['right']['ankleLateral'],
            self.jointVelocity['left']['shoulderSwing'], self.jointVelocity['left']['shoulderLateral'], self.jointVelocity['left']['elbow'], self.jointVelocity['right']['shoulderSwing'],
            self.jointVelocity['right']['shoulderLateral'], self.jointVelocity['right']['elbow'], self.jointVelocity['left']['hipTwist'], self.jointVelocity['left']['hipLateral'],
            self.jointVelocity['left']['hipSwing'], self.jointVelocity['left']['knee'], self.jointVelocity['left']['ankleSwing'], self.jointVelocity['left']['ankleLateral'], self.jointVelocity['right']['hipTwist'],
            self.jointVelocity['right']['hipLateral'], self.jointVelocity['right']['hipSwing'], self.jointVelocity['right']['knee'], self.jointVelocity['right']['ankleSwing'], self.jointVelocity['right']['ankleLateral'],
            self.contactFootR, self.contactFootL,self.odomPosX,self.odomPosZ]

if __name__ == '__main__':
    try:
        control = walkerControl()
        control.run()
    except:
        rospy.loginfo("[Walker Control]: Some error occured!")

