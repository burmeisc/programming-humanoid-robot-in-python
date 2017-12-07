'''In this exercise you need to implement inverse kinematics for NAO's legs

* Tasks:
    1. solve inverse kinematics for NAO's legs by using analytical or numerical method.
       You may need documentation of NAO's leg:
       http://doc.aldebaran.com/2-1/family/nao_h21/joints_h21.html
       http://doc.aldebaran.com/2-1/family/nao_h21/links_h21.html
    2. use the results of inverse kinematics to control NAO's legs (in InverseKinematicsAgent.set_transforms)
       and test your inverse kinematics implementation.
'''
from forward_kinematics import ForwardKinematicsAgent
from numpy.matlib import identity, arccos, sqrt, arcsin
#import numpy as np
from numpy.linalg import inv
from numpy import sin,cos,arctan,pi,transpose,asarray,eye,dot



class InverseKinematicsAgent(ForwardKinematicsAgent):

    def inverse_kinematics(self, effector_name, transform):
        '''solve the inverse kinematics

        :param str effector_name: name of end effector, e.g. LLeg, RLeg
        :param transform: 4x4 transform matrix
        :return: list of joint angles
        '''
        joint_angles = []
        # YOUR CODE HERE
        # change position of translation vector in matrix
        transform = asarray(transform)

        transform[:, -1] = transpose(transform[-1, :])
        transform[-1, :] = [0, 0, 0, 1]

        # check for endeffector
        if effector_name == 'LLeg':
            joint_values = self.inverse_LLeg(transform, joint_angles)

        elif effector_name == 'RLeg':
            joint_values = self.inverse_RLeg(transform, joint_angles)

        else:
            print('Endeffector unkown')

        return joint_values

    def set_transforms(self, effector_name, transform):
        '''solve the inverse kinematics and control joints use the results
        '''
        # YOUR CODE HERE
        joint_values = self.inverse_kinematics(effector_name, transform)
        joint_angles = joint_values[0]
        joint_names = joint_values[1]
        time = [[1]] * len(joint_names)
        angles = []
        names = []
        for angle, name in zip(joint_angles, joint_names):
            angles.append([[angle,[3, 0.00000, 0.00000], [3, 0.00000, 0.00000]]])
            names.append(name)

        self.keyframes = (names, time, angles)  # the result joint angles have to fill in

    # BIIIG TO DO: acutally there are multiple solution for each joint, which should be validated through FK.
    # for now only one (positive) solution is taken
    def inverse_LLeg(self, transform, joint_angles):
        tigh = 100
        tibia = 102.9

        joint_name = ['LKneePitch', 'LAnkleRoll', 'LAnklePitch', 'LHipRoll', 'LHipPitch', 'LHipYawPitch']

        # calculate T1
        rot_x = [[1. , 0, 0,0], [0, cos(pi / 4.), -sin(pi / 4.),0], [0, sin(pi / 4.), cos(pi / 4.),0],[0,0,0,1]]

        A_base = eye(4)
        A_end = eye(4)
        A_base[:,-1] = [0,50.0,-85.0,1]
        A_end[:,-1] = [0,0, -45.19,1]
        A_base = inv(A_base)
        A_end = inv(A_end)
        T_hut = dot(dot(A_base,transform), A_end)
        #print(T_hut)
        T_tilde = dot(rot_x, T_hut)
        #print(T_tilde)
        T1 = inv(T_tilde)

        # calculate knee_pitch
        print(T1)
        print()
        root = T1[0, 3]**2 + T1[1, 3]**2 + T1[2, 3]**2
        print("root {}".format(root))
        print((tigh**2 + tibia**2 - root) / 2. * tigh * tibia)
        knee_pitch = pi - arccos((tigh**2 * tibia**2 - root) / 2. * tigh * tibia)
        print("knee_pitch {}".format(knee_pitch))
        joint_angles.append(knee_pitch)

        # ankle roll
        ankle_roll = arctan(T1[0, 2] / T1[1, 2])

        joint_angles.append(ankle_roll)

        # T2
        rot_y = [[cos(-pi / 2), 0, sin(-pi / 2), 0], [0, 1, 0, 0],
                 [-sin(-pi / 2), 0, cos(-pi / 2), 0], [0, 0, 0, 1]]
        rot_z = [[cos(pi), -sin(pi), 0, 0], [sin(pi), cos(pi), 0, 0], [0, 0, 1, 0],
                 [0, 0, 0, 1]]
        # Tranformation matrix T(5-->6) calculated by FK
        t_5_6 = self.local_trans("LAnkleRoll", ankle_roll)
        temp = inv(dot(dot(t_5_6, rot_z), rot_y))
        T_tilde_2 = dot(T_tilde, temp)
        T2 = inv(T_tilde_2)

        # ankle pitch
        zaehler = T2[1, 3] * (tibia + tigh * cos(knee_pitch)) + tigh * T2[0, 3] * sin(knee_pitch)
        nenner = tigh**2 * sin(knee_pitch)**2 + (tibia + tigh * cos(knee_pitch))
        ankle_pitch = arcsin(zaehler / nenner)
        joint_angles.append(ankle_pitch)

        # T3
        t_4_5 = self.local_trans("LAnklePitch", ankle_pitch)
        t_3_4 = self.local_trans("LKneePitch", knee_pitch)
        trans = inv(dot(t_3_4, t_4_5))
        T3 = dot(inv(T2), inv(trans))

        hip_roll = arccos(T3[1, 2]) - pi / 4
        joint_angles.append(hip_roll)

        hip_pitch = arcsin(T3[1, 1] / sin(hip_roll + pi / 4))
        joint_angles.append(hip_pitch)

        hip_yaw_pitch = arccos(T3[0, 2] / sin(hip_roll + pi / 4))
        joint_angles.append(hip_yaw_pitch)
        return joint_angles, joint_name

    def inverse_RLeg(self, transform, joint_angles):
        tigh = 100
        tibia = 102.9

        joint_name = ['RKneePitch', 'RAnkleRoll', 'RAnklePitch', 'RHipRoll', 'RHipPitch', 'RHipYawPitch']

        # calculate T1
        rot_x = [[1, 0, 0], [0, np.cos(np.pi / 4), -np.sin(np.pi / 4)], [0, np.sin(np.pi / 4), np.cos(np.pi / 4)]]
        T_tilde = np.dot(rot_x, transform)
        T1 = np.invert(T_tilde)

        # calculate knee_pitch
        root = np.sqrt(np.square(T1[1, 4]) + np.square(T1[2, 4]) + np.square(T1[3, 4]))
        knee_pitch = np.pi - np.arccos((np.square(tigh) * np.square(tibia) - root) / 2 * tigh * tibia)

        joint_angles[0] = knee_pitch

        # ankle roll
        ankle_roll = np.arctan(T1[2, 4] / T1[3, 4])

        joint_angles[1] = ankle_roll

        # T2
        rot_y = [[np.cos(-np.pi / 2), 0, np.sin(-np.pi / 2)], [0, 1, 0], [-np.sin(-np.pi / 2), 0, np.cos(-np.pi / 2)]]
        rot_z = [[np.cos(np.pi), -np.sin(np.pi), 0], [np.sin(np.pi), np.cos(np.pi), 0], [0, 0, 1]]
        # Tranformation matrix T(5-->6) calculated by FK
        t_5_6 = self.local_trans("RAnkleRoll", ankle_roll)
        temp = np.invert(np.dot(np.dot(t_5_6, rot_z), rot_y))
        T_tilde_2 = np.dot(T_tilde, temp)
        T2 = np.invert(T_tilde_2)

        # ankle pitch
        zaehler = T2[2, 4] * (tibia + tigh * np.cos(knee_pitch)) + tigh * T2[1, 4] * np.sin(knee_pitch)
        nenner = np.square(tigh) * np.square(np.sin(knee_pitch)) + (tibia + tigh * np.cos(knee_pitch))
        ankle_pitch = np.arcsin(zaehler / nenner)
        joint_angles[2] = ankle_pitch

        # T3
        t_4_5 = self.local_trans("RAnklePitch", ankle_pitch)
        t_3_4 = self.local_trans("RKneePitch", knee_pitch)
        trans = np.invert(np.dot(t_3_4, t_4_5))
        T3 = np.dot(np.invert(T2), np.invert(trans))

        hip_roll = np.arccos(T3[2, 3]) - np.pi / 4
        joint_angles[3] = hip_roll

        hip_pitch = np.arcsin(T3[2, 2] / np.sin(hip_roll + np.pi / 4))
        joint_angles[4] = hip_pitch

        hip_yaw_pitch = np.arccos(T3[1, 3] / np.sin(hip_roll - np.pi / 4))  # only difference to left leg
        joint_angles[5] = hip_yaw_pitch

        return joint_angles, joint_name


if __name__ == '__main__':
    agent = InverseKinematicsAgent()
    # test inverse kinematics
    T = identity(4)
    T[-1, 1] = 0.05
    T[-1, 2] = 0.26
    agent.set_transforms('LLeg', T)
    agent.run()
