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
import numpy as np


class InverseKinematicsAgent(ForwardKinematicsAgent):
    def inverse_kinematics(self, effector_name, transform):
        '''solve the inverse kinematics

        :param str effector_name: name of end effector, e.g. LLeg, RLeg
        :param transform: 4x4 transform matrix (target)
        :return: list of joint angles
        '''
        joint_angles = []
        # YOUR CODE HERE

        """
        INIT
        """

        error = 0.01

        chain = {}

        for jointName in self.chains[effector_name]:
            chain[jointName] = self.perception.joint[jointName]

        M_pos = [0] * len(self.chains[effector_name])
        for i, jointName in enumerate(self.chains[effector_name]):
            M_pos[i] = self.transforms[jointName]

        print(transform)
        print(M_pos[-1])

        while(np.linalg.norm(transform - M_pos[-1]) > error):
            dO = self.jacobianTransformation(effector_name, transform, M_pos)
            O = dO * 2



            #last step before next loop
            for i, jointName in enumerate(self.chains[effector_name]):
                M_pos[i] = self.transforms[jointName]



        return joint_angles

    def jacobianTransformation(self, effector_name, transform, M_pos):

        Jt = self.jacobianTranspose(effector_name, transform, M_pos)
        V = transform - M_pos[-1]
        dO = np.dot(Jt, V)

        return dO

    def jacobianTranspose(self, effector_name, peter, M_pos):
        Jt = [0] * len(M_pos)
        i = 0
        for matrix in M_pos:
            dec = self.decompose(matrix)
            vec = [dec['Ox'], dec['Oy'], dec['Oz']]

            vec = np.cross(vec, np.subtract([peter[0,3], peter[1,3], peter[2,3]],[matrix[0,3], matrix[1,3], matrix[2,3]]))
            Jt[i] = vec
            i += 1


        print(Jt)
        return Jt


    def decompose(self, M):
        # decompose a transformationmatrix

        Ox = np.arctan2(M[2, 1], M[2, 2])
        Oy = np.arctan2(-M[0, 2], np.sqrt(pow(M[2, 1], 2) + pow(M[2, 2], 2)))
        Oz = np.arctan2(M[1, 0], M[0, 0])

        return {'Ox': Ox, 'Oy': Oy, 'Oz': Oz}


    def set_transforms(self, effector_name, transform):
        '''solve the inverse kinematics and control joints use the results
        '''
        # YOUR CODE HERE



        InverseKinematicsAgent.inverse_kinematics(self, effector_name, transform)
        self.keyframes = ([], [], [])  # the result joint angles have to fill in

if __name__ == '__main__':
    agent = InverseKinematicsAgent()
    # test inverse kinematics
    T = identity(4)
    T[-1, 1] = 0.05
    T[-1, 2] = 0.26
    agent.set_transforms('LLeg', T)
    agent.run()
