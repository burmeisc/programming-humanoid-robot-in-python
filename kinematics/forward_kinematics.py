'''In this exercise you need to implement forward kinematics for NAO robot

* Tasks:
    1. complete the kinematics chain definition (self.chains in class ForwardKinematicsAgent)
       The documentation from Aldebaran is here:
       http://doc.aldebaran.com/2-1/family/robots/bodyparts.html#effector-chain
    2. implement the calculation of local transformation for one joint in function
       ForwardKinematicsAgent.local_trans. The necessary documentation are:
       http://doc.aldebaran.com/2-1/family/nao_h21/joints_h21.html
       http://doc.aldebaran.com/2-1/family/nao_h21/links_h21.html
    3. complete function ForwardKinematicsAgent.forward_kinematics, save the transforms of all body parts in torso
       coordinate into self.transforms of class ForwardKinematicsAgent

* Hints:
    the local_trans has to consider different joint axes and link parameters for different joints
'''

# add PYTHONPATH
import os
import sys
sys.path.append(os.path.join(os.path.abspath(os.path.dirname(__file__)), '..', 'joint_control'))

from numpy.matlib import matrix, identity
import numpy as np

from angle_interpolation import AngleInterpolationAgent


class ForwardKinematicsAgent(AngleInterpolationAgent):
    def __init__(self, simspark_ip='localhost',
                 simspark_port=3100,
                 teamname='DAInamite',
                 player_id=0,
                 sync_mode=True):
        super(ForwardKinematicsAgent, self).__init__(simspark_ip, simspark_port, teamname, player_id, sync_mode)
        self.transforms = {n: identity(4) for n in self.joint_names}
        pi = np.pi
        # chains defines the name of chain and joints of the chain
        self.chains = {'Head': ['HeadYaw', 'HeadPitch'],
                       # YOUR CODE HERE
                       'LArm': ['LShoulderPitch', 'LShoulderRoll', 'LElbowYaw','LElbowRoll','LWristYaw'],
                       'LLeg': ['LHipYawPitch','LHipRoll', 'LHipPitch','LKneePitch','LAnklePitch','LAnkleRoll'],
                       'RLeg': ['RHipYawPitch','RHipRoll', 'RHipPitch','RKneePitch','RAnklePitch','RAnkleRoll'],
                       'RArm':['RShoulderPitch', 'RShoulderRoll', 'RElbowYaw','RElbowRoll','RWristYaw']
                       }
        #parameter for the DH-matrix [a,alpha,d,angle_offset,[dx,dy,dz]]
        self.dh_params = {'HeadYaw':[0,0,0,0,[0,0,126.5]],
                          'HeadPitch': [0, -pi/2,0, -pi/2,[0,0,0]],

                          'LShoulderPitch':[0,-pi/2,0,0,[0,98+15,100.0]],
                          'LShoulderRoll':[0,pi/2,0,-pi/2,[0,0,0]],
                          'LElbowYaw': [0,-pi/2,105.00,0,[0,0,0]],
                          'LElbowRoll': [0,pi/2,0,0,[0,0,0]],

                          'RShoulderPitch':[0,-pi/2,0,0,[0,-98.00-15.0,100.0]],
                          'RShoulderRoll': [0, pi/2,0,pi/2,[0,0,0]],
                          'RElbowYaw':[0,pi/2,0,0,[0,0,0]],
                          'RElbowRoll': [0,pi/2,0,0,[0,0,0]]

                        }

    def think(self, perception):
        self.forward_kinematics(perception.joint)
        return super(ForwardKinematicsAgent, self).think(perception)

    def local_trans(self, joint_name, joint_angle):
        '''calculate local transformation of one joint

        :param str joint_name: the name of joint
        :param float joint_angle: the angle of joint in radians
        :return: transformation
        :rtype: 4x4 matrix
        '''
        T = identity(4)
        # YOUR CODE HERE
        t_param = self.dh_params[joint_name]
        a = t_param[0]
        alpha = t_param[1]
        d = t_param[2]
        angle_offset = t_param[3]
        dx = t_param[4][0]
        dy= t_param[4][1]
        dz = t_param[4][2]
        joint_angle = joint_angle + angle_offset

        if joint_name == "LElbowYaw":
            joint_angle = -joint_angle

        T[0] = [np.cos(joint_angle), -np.sin(joint_angle),0,alpha+dx]
        T[1] = [np.sin(joint_angle)*np.cos(alpha),np.cos(joint_angle)*np.cos(alpha), -np.sin(alpha),-d*np.sin(alpha)+dy]
        T[2] = [np.sin(joint_angle)*np.cos(alpha), np.cos(joint_angle)*np.sin(alpha), np.cos(alpha), d*np.cos(alpha)+dz]
        T[3] = [0,0,0,1]

        return T

    def forward_kinematics(self, joints):
        '''forward kinematics

        :param joints: {joint_name: joint_angle}
        '''
        #iterates over joint chains from chains e.g. ['HeadYaw','HeadPitch']
        for chain_joints in self.chains.values():
            T = identity(4)
            joint_before = 0
            for joint in chain_joints:
                angle = joints[joint]
                Tl = self.local_trans(joint, angle)
                # YOUR CODE HERE
                #T is product of transformation matrixes of all joints before
                if not(joint_before == 0):
                    T = self.transforms[joint_before]*Tl
                else: T = Tl

                self.transforms[joint] = T

                joint_before = joint

if __name__ == '__main__':
    agent = ForwardKinematicsAgent()
    agent.run()
