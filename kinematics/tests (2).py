
import numpy as np
from math import pi, sqrt, atan2, cos, sin
import unittest
from forward_kinematics import ForwardKinematicsAgent


class Tests(unittest.TestCase):

    def all_chains(self, angles, expected_values):
        agent = ForwardKinematicsAgent()
        agent.forward_kinematics(angles)
        for key in expected_values:
            M = agent.transforms[key]
            R = self.rotationMatrixToEulerAngles(M[0:3, 0:3])
            values = [M[0, 3], M[1, 3], M[2, 3], R[0], R[1], R[2]]
            self.assertTrue(np.allclose(expected_values[key], values),
                            str(expected_values[key]) + ' expected not equals calculated ' + str(values))

    def single_matrix(self, agent, joint_name, angle_in_radians, result_matrix):
        self.assertTrue(np.allclose(agent.local_trans(joint_name, angle_in_radians), result_matrix),
                        str(agent.local_trans(joint_name, angle_in_radians)) + ' calculated not equals expected ' + str(result_matrix))

    def test_local_trans(self):
        agent = ForwardKinematicsAgent()

        self.single_matrix(agent, 'HeadYaw', 0, [[1, 0, 0, 0],
                                                 [0, 1, 0, 0],
                                                 [0, 0, 1, 126.5],
                                                 [0, 0, 0, 1]])
        self.single_matrix(agent, 'HeadYaw', pi/2, [[0, 1, 0, 0],
                                                   [-1, 0, 0, 0],
                                                   [0, 0, 1, 126.5],
                                                   [0, 0, 0, 1]])
        self.single_matrix(agent, 'HeadPitch', -pi/2, [[0, 0, -1, 0],
                                                       [0, 1, 0, 0],
                                                       [1, 0, 0, 0],
                                                       [0, 0, 0, 1]])
        self.single_matrix(agent, 'RElbowRoll', pi, [[1, 0, 0, 0],
                                                     [0, -1, 0, 0],
                                                     [0, 0, -1, 0],
                                                     [0, 0, 0, 1]])

    def test_rotationMatrixToEulerAngles(self):
        R = np.identity(3)
        angles = self.rotationMatrixToEulerAngles(R)
        self.assertTrue(np.allclose(np.array([0, 0, 0]), angles),
                        str(angles) + ' not equals 0, 0, 0: ' + str(R))

        R = np.matrix([[0, 1, 0], [-1, 0, 0], [0, 0, 1]])
        angles = self.rotationMatrixToEulerAngles(R)
        self.assertTrue(np.allclose(np.array([0, 0, pi/2]), angles),
                        str(angles) + ' not equals 0, 0, pi/2: ' + str(R))

        R = np.matrix([[0, 0, -1], [0, 1, 0], [1, 0, 0]])
        angles = self.rotationMatrixToEulerAngles(R)
        self.assertTrue(np.allclose(np.array([0, -pi/2, 0]), angles),
                        str(angles) + ' not equals 0, -pi/2, 0: ' + str(R))

        R = np.matrix([[1, 0, 0], [0, -1, 0], [0, 0, -1]])
        angles = self.rotationMatrixToEulerAngles(R)
        self.assertTrue(np.allclose(np.array([pi, 0, 0]), angles),
                        str(angles) + ' not equals pi, 0, 0: ' + str(R))

        R = np.matrix([[cos(pi/4), 0, sin(pi/4)], [0, 1, 0], [-sin(pi/4), 0, cos(pi/4)]])
        angles = self.rotationMatrixToEulerAngles(R)
        self.assertTrue(np.allclose(np.array([0, pi/4, 0]), angles),
                        str(angles) + ' not equals 0, pi/4, 0: ' + str(R))

        R = np.matrix([[1, 0, 0], [0, cos(pi/4), -sin(pi/4)], [0, sin(pi/4), cos(pi/4)]])
        angles = self.rotationMatrixToEulerAngles(R)
        self.assertTrue(np.allclose(np.array([pi/4, 0, 0]), angles),
                        str(angles) + ' not equals pi/4, 0, 0: ' + str(R))



    def test_default_values(self):
        angles = {'HeadYaw' : 0,
                  'HeadPitch' : 0,
                  'LShoulderPitch' : 0,
                  'LShoulderRoll' : 0,
                  'LElbowYaw' : 0,
                  'LElbowRoll' : 0,
                  'LWristYaw' : 0,
                  'LHipYawPitch' : 0,
                  'LHipRoll' : 0,
                  'LHipPitch' : 0,
                  'LKneePitch' : 0,
                  'LAnklePitch' : 0,
                  'LAnkleRoll' : 0,
                  'RShoulderPitch' : 0,
                  'RShoulderRoll' : 0,
                  'RElbowYaw' : 0,
                  'RElbowRoll' : 0,
                  'RWristYaw' : 0,
                  'RHipYawPitch' : 0,
                  'RHipRoll' : 0,
                  'RHipPitch' : 0,
                  'RKneePitch' : 0,
                  'RAnklePitch' : 0,
                  'RAnkleRoll' : 0}
        expected_values = {'HeadPitch' : [0, 0, 126.5, 0, 0, 0],
                           'LWristYaw':  [160.95, 113, 100, 0, 0, 0],
                           'RWristYaw':  [160.95, -113, 100, 0, 0, 0],
                           'LAnkleRoll': [0, 50, -287.9, 0, 0, 0],
                           'RAnkleRoll': [0, -50, -287.9, 0, 0, 0]
                           }
        self.all_chains(angles, expected_values)
        pass

    def test_head_values(self):
        angles = {'HeadYaw' : pi/2,
                  'HeadPitch' : pi/4,
                  'LShoulderPitch' : 0,
                  'LShoulderRoll' : 0,
                  'LElbowYaw' : 0,
                  'LElbowRoll' : 0,
                  'LWristYaw' : 0,
                  'LHipYawPitch' : 0,
                  'LHipRoll' : 0,
                  'LHipPitch' : 0,
                  'LKneePitch' : 0,
                  'LAnklePitch' : 0,
                  'LAnkleRoll' : 0,
                  'RShoulderPitch' : 0,
                  'RShoulderRoll' : 0,
                  'RElbowYaw' : 0,
                  'RElbowRoll' : 0,
                  'RWristYaw' : 0,
                  'RHipYawPitch' : 0,
                  'RHipRoll' : 0,
                  'RHipPitch' : 0,
                  'RKneePitch' : 0,
                  'RAnklePitch' : 0,
                  'RAnkleRoll' : 0}
        expected_values = {'HeadPitch' : [0, 0, 126.5, 0, pi/4, pi/2],
                           'LWristYaw':  [160.95, 113, 100, 0, 0, 0],
                           'RWristYaw':  [160.95, -113, 100, 0, 0, 0],
                           'LAnkleRoll': [0, 50, -287.9, 0, 0, 0],
                           'RAnkleRoll': [0, -50, -287.9, 0, 0, 0]
                           }
        self.all_chains(angles, expected_values)
        pass

    def test_arm_values(self):
        angles = {'HeadYaw' : 0,
                  'HeadPitch' : 0,
                  'LShoulderPitch' : 0,
                  'LShoulderRoll' : 0,
                  'LElbowYaw' : 0,
                  'LElbowRoll' : 0,
                  'LWristYaw' : 0,
                  'LHipYawPitch' : 0,
                  'LHipRoll' : 0,
                  'LHipPitch' : 0,
                  'LKneePitch' : 0,
                  'LAnklePitch' : 0,
                  'LAnkleRoll' : 0,
                  'RShoulderPitch' : pi/2,
                  'RShoulderRoll' : -pi/4,
                  'RElbowYaw' : pi/2,
                  'RElbowRoll' : pi/2,
                  'RWristYaw' : 0,
                  'RHipYawPitch' : 0,
                  'RHipRoll' : 0,
                  'RHipPitch' : 0,
                  'RKneePitch' : 0,
                  'RAnklePitch' : 0,
                  'RAnkleRoll' : 0}
        expected_values = {'HeadPitch' : [0, 0, 126.5, 0, 0, 0],
                           'LWristYaw':  [160.95, 113, 100, 0, 0, 0],
                           'RWristYaw':  [55.95, -98 -15*cos(pi/4) -105*sin(pi/4), 100 +15*sin(pi/4) -105*cos(pi/4) , pi/4, 0, 0],
                           'LAnkleRoll': [0, 50, -287.9, 0, 0, 0],
                           'RAnkleRoll': [0, -50, -287.9, 0, 0, 0]
                           }
        self.all_chains(angles, expected_values)
        pass

    def rotationMatrixToEulerAngles(self, R):
        sy = sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])

        if not sy < 1e-6:
            x = atan2(R[2, 1], R[2, 2])
            y = atan2(-R[2, 0], sy)
            z = atan2(R[1, 0], R[0, 0])
        else:
            x = atan2(-R[1, 2], R[1, 1])
            y = atan2(-R[2, 0], sy)
            z = 0

        return np.array([x, y, -z])

if __name__ == '__main__':
    unittest.main()

