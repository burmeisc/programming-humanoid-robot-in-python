'''In this file you need to implement remote procedure call (RPC) server

* There are different RPC libraries for python, such as xmlrpclib, json-rpc. You are free to choose.
* The following functions have to be implemented and exported:
 * get_angle
 * set_angle
 * get_posture
 * execute_keyframes
 * get_transform
 * set_transform
* You can test RPC server with ipython before implementing agent_client.py
'''

# add PYTHONPATH
import os
import sys
sys.path.append(os.path.join(os.path.abspath(os.path.dirname(__file__)), '..', 'kinematics'))

from inverse_kinematics import InverseKinematicsAgent
from SimpleXMLRPCServer import SimpleXMLRPCServer
from SimpleXMLRPCServer import SimpleXMLRPCRequestHandler

class ServerAgent(InverseKinematicsAgent):
    '''ServerAgent provides RPC service
    '''
    # YOUR CODE HERE
    def get_angle(self, joint_name):
        '''get sensor value of given joint'''
        # YOUR CODE HERE
        #print("get angle called")
        angle = self.perception.joint[joint_name]
        #print(type(angle))
        return angle


    def set_angle(self, joint_name, angle):
        '''set target angle of joint for PID controller
        '''
        # YOUR CODE HERE
        self.perception.joint[joint_name] = angle

    def get_posture(self):
        '''return current posture of robot'''
        # YOUR CODE HERE
        return self.recognize_posture()

    def execute_keyframes(self, keyframes):
        '''excute keyframes, note this function is blocking call,
        e.g. return until keyframes are executed
        '''

        # YOUR CODE HERE

        self.angle_interpolation(keyframes,self.perception)

    def get_transform(self, name):
        '''get transform with given name
        '''
        # YOUR CODE HERE
        return self.forward_kinematics(name)

    def set_transform(self, effector_name, transform):
        '''solve the inverse kinematics and control joints use the results
        '''
        # YOUR CODE HERE
        transform = self.inverse_kinematics(effector_name,transform)



# Create server
server = SimpleXMLRPCServer(("localhost", 8000), allow_none= True)
print(server)
server.register_instance(ServerAgent())
server.register_introspection_functions()

    # Run the server's main loop

server.serve_forever()

if __name__ == '__main__':
    agent = ServerAgent()
    agent.run()

