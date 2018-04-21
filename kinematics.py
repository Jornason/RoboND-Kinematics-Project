

import rospy
import tf
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose
from mpmath import *
from sympy import *
import csv


class Kinematics:

    #Compute and save in memory the transformation matrices when the object is created
    #to speed-up run-time computations
    def __init__(self):

        # Create symbols
        q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8')
        d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8')
        a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7')
        alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7')

        # Create Modified DH parameters
        self.s = {alpha0:        0, a0:      0, d1:  0.75, q1: q1,
                 alpha1: rad(-90), a1:   0.35, d2:     0, q2: q2-rad(90),
                 alpha2:        0, a2:   1.25, d3:     0, q3: q3,
                 alpha3: rad(-90), a3: -0.054, d4:  1.50, q4: q4,
                 alpha4:  rad(90), a4:      0, d5:     0, q5: q5,
                 alpha5: rad(-90), a5:      0, d6:     0, q6: q6,
                 alpha6:        0, a6:      0, d7: 0.303, q7: 0
            }

        # Create individual transformation matrices
        self.T0_1 = self.transformation_matrix(alpha0, a0, d1, q1).subs(self.s)
        self.T1_2 = self.transformation_matrix(alpha1, a1, d2, q2).subs(self.s)
        self.T2_3 = self.transformation_matrix(alpha2, a2, d3, q3).subs(self.s)
        self.T3_4 = self.transformation_matrix(alpha3, a3, d4, q4).subs(self.s)
        self.T4_5 = self.transformation_matrix(alpha4, a4, d5, q5).subs(self.s)
        self.T5_6 = self.transformation_matrix(alpha5, a5, d6, q6).subs(self.s)
        self.T6_EE = self.transformation_matrix(alpha6, a6, d7, q7).subs(self.s)

        #Create complete transformation matrix
        self.T0_EE = self.T0_1 * self.T1_2 * self.T2_3 * self.T3_4 * self.T4_5 * self.T5_6 * self.T6_EE

        # Compensate for rotation discrepancy between DH parameters and Gazebo
        self.Rot_err = self.rot_z(rad(180)) * self.rot_y(rad(-90))

        #Generic computation part of R0_3 (only needs to be done once)
        self.R0_3_gen = self.T0_1[0:3,0:3] * self.T1_2[0:3,0:3] * self.T2_3[0:3,0:3]

    #Compute inverse kinematics
    def compute_IK(self, px, py, pz, roll, pitch, yaw):

        # Create symbols
        q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8')
        d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8')
        a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7')
        alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7')
        r, p, y = symbols('r p y')

        #Create rotation matrices for x,y,z
        R_x = self.rot_x(r)
        R_y = self.rot_y(p)
        R_z = self.rot_z(y)

        # Create rotation matrix of the end effector
        R_EE = R_z * R_y * R_x

        # Compensate for rotation discrepancy between DH parameters and Gazebo
        R_EE = R_EE * self.Rot_err
        R_EE = R_EE.subs({'r': roll, 'p': pitch, 'y': yaw})

        #Position of the end effector
        EE = Matrix([[px], [py], [pz]])

        #Position of the wrist center
        WC = EE - (0.303) * R_EE[:, 2]

        #Computation of joint angles using geometric inverse kinematics method
        theta1 = atan2(WC[1], WC[0])

        a = 1.501
        b = sqrt(pow((sqrt(WC[0] * WC[0] + WC[1] * WC[1]) - 0.35), 2) + \
            pow((WC[2] - 0.75), 2))
        c = 1.25

        angle_a = acos((b*b + c*c - a*a) / (2*b*c))
        angle_b = acos((a*a + c*c - b*b) / (2*a*c))
        delta = atan2(WC[2] - 0.75, sqrt(WC[0]*WC[0] + WC[1]*WC[1]) - 0.35)
        theta2 = pi/2 - angle_a - delta
        theta3 = pi/2 - (angle_b + 0.036)

        R0_3 = self.R0_3_gen.evalf(subs={q1:theta1, q2:theta2, q3:theta3})
        R3_6 = R0_3.inv("LU") * R_EE

        theta6 = atan2(-R3_6[1,1], R3_6[1,0])
        theta5 = atan2(-R3_6[1,1]/sin(theta6), R3_6[1,2])
        theta4 = atan2(R3_6[2,2]/sin(theta5), -R3_6[0,2]/sin(theta5))

        print("Theta1: %04.8f"% theta1)
        print("Theta2: %04.8f"% theta2)
        print("Theta3: %04.8f"% theta3)
        print("Theta4: %04.8f"% theta4)
        print("Theta5: %04.8f"% theta5)
        print("Theta6: %04.8f"% theta6)
        print("\n")

        #simplify angles of theta4 and theta6, to try to prevent large rotations
        theta4 = self.simplify_angle(theta4)
        theta6 = self.simplify_angle(theta6)

        return theta1, theta2, theta3, theta4, theta5, theta6

    #Substitute the joint angles to return the transformation matrix
    def compute_FK(self, theta1, theta2, theta3, theta4, theta5, theta6):

        q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8')
        d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8')
        a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7')
        alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7')

        return self.T0_EE.evalf(subs={q1: theta1, q2: theta2, q3: theta3, q4: theta4, q5: theta5, q6: theta6})

    #Prevent large rotations by clipping between -pi and pi
    def simplify_angle(self,angle):

        angle = abs(angle) % (2*pi) * sign(angle)
        if angle > pi:
            angle = angle - 2*pi

        return angle

    #Rotation matrix for x
    def rot_x(self,q):
        R_x = Matrix([[ 1,              0,        0],
                      [ 0,         cos(q),  -sin(q)],
                      [ 0,         sin(q),  cos(q)]])

        return R_x

    #Rotation matrix for y
    def rot_y(self,q):
        R_y = Matrix([[ cos(q),        0,  sin(q)],
                      [      0,        1,       0],
                      [-sin(q),        0, cos(q)]])

        return R_y

    #Rotation matrix for z
    def rot_z(self,q):
        R_z = Matrix([[ cos(q),  -sin(q),       0],
                      [ sin(q),   cos(q),       0],
                      [      0,        0,       1]])

        return R_z

    #Generic transformation matrix
    def transformation_matrix(self,alpha, a, d, q):
        T = Matrix([[            cos(q),           -sin(q),           0,             a],
                    [ sin(q)*cos(alpha), cos(q)*cos(alpha), -sin(alpha), -sin(alpha)*d],
                    [ sin(q)*sin(alpha), cos(q)*sin(alpha),  cos(alpha),  cos(alpha)*d],
                    [                 0,                 0,           0,             1]])
        return T
