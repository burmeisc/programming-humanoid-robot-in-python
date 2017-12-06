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
from numpy.matlib import identity


class InverseKinematicsAgent(ForwardKinematicsAgent):

    def inverse_kinematics(self, effector_name, transform):
        '''solve the inverse kinematics

        :param str effector_name: name of end effector, e.g. LLeg, RLeg
        :param transform: 4x4 transform matrix
        :return: list of joint angles
        '''
        joint_angles = []
        # YOUR CODE HERE
        # check for endeffector
        if effector_name == 'LLeg':
            joint_angles = self.inverse_LLeg(transform)

        if effector_name == 'RLeg':
            joint_angles = self.inverse_RLeg(transform)

        if effector_name =="LArm":
            joint_angles = self.inverse_LArm(transform)

        if effector_name == 'RArm':
            joint_angles = self.inverse_RArm(transform)

        if effector_name == 'Head':
            joint_angles = self.inverse_Head(transform)
        else: print('Endeffector unkown')

        return joint_angles

    def set_transforms(self, effector_name, transform):
        '''solve the inverse kinematics and control joints use the results
        '''
        # YOUR CODE HERE
        self.keyframes = ([], [], [])  # the result joint angles have to fill in

    def inverse_LLeg (self):

    def inverse_RLeg(self):

    def inverse_LArm(self):

    def inverse_RArm(self):

    def inverse_Head(self,transform):
        #target_postition = [px,py,pz]
        target_position = transform[-1][:]

        return joint_angles

if __name__ == '__main__':
    agent = InverseKinematicsAgent()
    # test inverse kinematics
    T = identity(4)
    T[-1, 1] = 0.05
    T[-1, 2] = 0.26
    agent.set_transforms('LLeg', T)
    agent.run()
