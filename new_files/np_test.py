import numpy as np
from filterpy.kalman import ExtendedKalmanFilter

class Testing:
    def __init__(self):
        self.ekf = ExtendedKalmanFilter(dim_x=12, dim_z=12, dim_u=6)
        self.dt = 0.01 # sampling rate of the EKF, edit?
        self.input = np.array([0, 0, 0, 0, 0, 0])
        self.ekf.x = np.array([1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0])  # initial state
        # model parameters
        self.m = 13.5 #kg #mass
        self.I_xx = 0.26 #inertia
        self.I_yy = 0.23
        self.I_zz = 0.37
        self.X_udot = 6.36 #added mass
        self.Y_vdot = 7.12
        self.Z_wdot = 18.68
        self.K_pdot = 0.189
        self.M_qdot = 0.135
        self.N_rdot = 0.222
        self.X_u = 13.7 #linear and quadratic damping
        self.X_uu = 141
        self.Y_v = 0
        self.Y_vv = 217
        self.Z_w = 33
        self.Z_ww = 190
        self.K_p = 0.0
        self.K_pp = 1.19
        self.M_q = 0.8
        self.M_qq = 0.47
        self.N_r = 0
        self.N_rr = 1.5

        self._propagate_state()
        self._get_F_Jacobian()

    def _propagate_state(self): # nonlinear state transition, use model and discretize
        #innvoation is the CT model, further it is discretized using forward euler
        #state vector is x=[x,y,z,phi,theta,psi,u,v,w,p,q,r]
        innovation = np.zeros((12)) #clear the array
        for i in range(6): # innovation in CT is equal to the velocity which is contained in other states
            innovation[i] = self.ekf.x[i + 6]
        #the actual model part
        M_rb = np.diag([self.m, self.m, self.m, self.I_xx, self.I_yy, self.I_zz])
        M_a = np.diag([self.X_udot, self.Y_vdot, self.Z_wdot, self.K_pdot, self.M_qdot, self.N_rdot])
        M = M_rb + M_a # constant mass/inertia matrix, combined rigid body and added mass
        # noinspection PyTypeChecker
        C_rb = np.array([[0, 0, 0, 0, self.m*self.ekf.x[8], -self.m*self.ekf.x[7]],
                        [0, 0, 0, -self.m*self.ekf.x[8], 0, self.m*self.ekf.x[6]],
                        [0, 0, 0, self.m*self.ekf.x[7], -self.m*self.ekf.x[6], 0],
                        [0, self.m*self.ekf.x[8], -self.m*self.ekf.x[7], 0, -self.I_zz*self.ekf.x[11], -self.I_yy*self.ekf.x[10]],
                        [-self.m*self.ekf.x[8], 0, self.m*self.ekf.x[6], self.I_zz*self.ekf.x[11], 0, self.I_xx*self.ekf.x[9]],
                        [self.m*self.ekf.x[7], -self.m*self.ekf.x[6], 0, self.I_yy*self.ekf.x[10], -self.I_xx*self.ekf.x[9], 0]])
        # noinspection PyTypeChecker
        C_a =  np.array([[0, 0, 0, 0, -self.Z_wdot*self.ekf.x[8], self.Y_vdot*self.ekf.x[7]],
                        [0, 0, 0, self.Z_wdot*self.ekf.x[8], 0, -self.X_udot*self.ekf.x[6]],
                        [0, 0, 0, -self.Y_vdot*self.ekf.x[7], self.X_udot*self.ekf.x[6], 0],
                        [0, -self.Z_wdot*self.ekf.x[8], self.Y_vdot*self.ekf.x[7], 0, self.N_rdot*self.ekf.x[11], self.M_qdot*self.ekf.x[10]],
                        [self.Z_wdot*self.ekf.x[8], 0, -self.X_udot*self.ekf.x[6], -self.N_rdot*self.ekf.x[11], 0, -self.K_pdot*self.ekf.x[9]],
                        [-self.Y_vdot*self.ekf.x[7], self.X_udot*self.ekf.x[6], 0, -self.M_qdot*self.ekf.x[10], self.K_pdot*self.ekf.x[9], 0]])
        D = np.diag([self.X_u+(self.X_uu*np.abs(self.ekf.x[6])), self.Y_v+(self.Y_vv*np.abs(self.ekf.x[7])), self.Z_w+(self.Z_ww*np.abs(self.ekf.x[8])), self.K_p+(self.K_pp*np.abs(self.ekf.x[9])), self.M_q+(self.M_qq*np.abs(self.ekf.x[10])), self.N_r+(self.N_rr*np.abs(self.ekf.x[11]))])
        C = C_rb + C_a + D # combined coriolis and damping matrices, all are multiplied by velocity

        innovation[6:12] = np.dot(np.linalg.inv(M), (self.input - np.dot(C, self.ekf.x[6:12]))) #adds the model in the bottom right corner of the matrix
        #CT model is complete, discretization
        self.ekf.x = self.ekf.x + innovation*self.dt #forward euler discretization, does it work like that?

    def _get_F_Jacobian(self): # linearization to get F matrix and compute covariance
        Jacobian_cont = np.zeros((12, 12))
        for i in range(6):  # add diagonal part in top right corner
            # ex: innovation[0,6] = 1 # Set the diagonal offset by 6
            Jacobian_cont[i, i + 6] = 1


            Jacobian_cont[6:12, 6:12] = np.array([[(-self.X_u - 2*self.X_uu*np.abs(self.ekf.x[6]))/(self.X_udot + self.m), -self.ekf.x[11]*(self.Y_vdot - self.m)/(self.X_udot + self.m), -self.ekf.x[10]*(-self.Z_wdot + self.m)/(self.X_udot + self.m), 0, (self.Z_wdot*self.ekf.x[8] - self.ekf.x[8]*self.m)/(self.X_udot + self.m), (-self.Y_vdot*self.ekf.x[7] + self.ekf.x[7]*self.m)/(self.X_udot + self.m)],
                                              [-self.ekf.x[11]*(-self.X_udot + self.m)/(self.Y_vdot + self.m), (-self.Y_v - 2*self.Y_vv*np.abs(self.ekf.x[7]))/(self.Y_vdot + self.m), -self.ekf.x[9]*(self.Z_wdot - self.m)/(self.Y_vdot + self.m), (-self.Z_wdot*self.ekf.x[8] + self.ekf.x[8]*self.m)/(self.Y_vdot + self.m), 0, (self.X_udot*self.ekf.x[6] - self.ekf.x[6]*self.m)/(self.Y_vdot + self.m)],
                                              [-self.ekf.x[10]*(self.X_udot - self.m)/(self.Z_wdot + self.m), -self.ekf.x[9]*(-self.Y_vdot + self.m)/(self.Z_wdot + self.m), (-self.Z_w - 2*self.Z_ww*np.abs(self.ekf.x[8]))/(self.Z_wdot + self.m), (self.Y_vdot*self.ekf.x[7] - self.ekf.x[7]*self.m)/(self.Z_wdot + self.m), (-self.X_udot*self.ekf.x[6] + self.ekf.x[6]*self.m)/(self.Z_wdot + self.m), 0],
                                              [0, (self.Z_wdot*self.ekf.x[8] - self.ekf.x[8]*self.m - self.ekf.x[8]*(self.Y_vdot - self.m))/(self.I_xx + self.K_pdot), (-self.Y_vdot*self.ekf.x[7] + self.ekf.x[7]*self.m - self.ekf.x[7]*(-self.Z_wdot + self.m))/(self.I_xx + self.K_pdot), (-self.K_p - 2*self.K_pp*np.abs(self.ekf.x[9]))/(self.I_xx + self.K_pdot), (self.I_zz*self.ekf.x[11] - self.N_rdot*self.ekf.x[11] - self.ekf.x[11]*(-self.I_yy + self.M_qdot))/(self.I_xx + self.K_pdot), (self.I_yy*self.ekf.x[10] - self.M_qdot*self.ekf.x[10] - self.ekf.x[10]*(-self.I_zz + self.N_rdot))/(self.I_xx + self.K_pdot)],
                                              [(-self.Z_wdot*self.ekf.x[8] + self.ekf.x[8]*self.m - self.ekf.x[8]*(-self.X_udot + self.m))/(self.I_yy + self.M_qdot), 0, (self.X_udot*self.ekf.x[6] - self.ekf.x[6]*self.m - self.ekf.x[6]*(self.Z_wdot - self.m))/(self.I_yy + self.M_qdot), (-self.I_zz*self.ekf.x[11] + self.N_rdot*self.ekf.x[11] - self.ekf.x[11]*(self.I_xx - self.K_pdot))/(self.I_yy + self.M_qdot), (-self.M_q - 2*self.M_qq*np.abs(self.ekf.x[10]))/(self.I_yy + self.M_qdot), (-self.I_xx*self.ekf.x[9] + self.K_pdot*self.ekf.x[9] - self.ekf.x[9]*(self.I_zz - self.N_rdot))/(self.I_yy + self.M_qdot)],
                                              [(self.Y_vdot*self.ekf.x[7] - self.ekf.x[7]*self.m - self.ekf.x[7]*(self.X_udot - self.m))/(self.I_zz + self.N_rdot), (-self.X_udot*self.ekf.x[6] + self.ekf.x[6]*self.m - self.ekf.x[6]*(-self.Y_vdot + self.m))/(self.I_zz + self.N_rdot), 0, (-self.I_yy*self.ekf.x[10] + self.M_qdot*self.ekf.x[10] - self.ekf.x[10]*(-self.I_xx + self.K_pdot))/(self.I_zz + self.N_rdot), (self.I_xx*self.ekf.x[9] - self.K_pdot*self.ekf.x[9] - self.ekf.x[9]*(self.I_yy - self.M_qdot))/(self.I_zz + self.N_rdot), (-self.N_r - 2*self.N_rr*np.abs(self.ekf.x[11]))/(self.I_zz + self.N_rdot)]])
        self.ekf.F = np.identity(12) + Jacobian_cont*self.dt #discretize jacobian of continuous model

if __name__ == '__main__':
    ekf_node = Testing()

    # remaining part of the jacobian matrix, calculated using separate script on the non linear model
    # noinspection PyTypeChecker
    # test1 = np.array([0, (self.Z_wdot*self.ekf.x[8] - self.ekf.x[8]*self.m - self.ekf.x[8]*(self.Y_vdot - self.m))/(self.I_xx + self.K_pdot), (-self.Y_vdot*self.ekf.x[7] + self.ekf.x[7]*self.m - self.ekf.x[7]*(-self.Z_wdot + self.m))/(self.I_xx + self.K_pdot), (-self.K_p - 2*self.K_pp*np.abs(self.ekf.x[9]))/(self.I_xx + self.K_pdot), (self.I_zz*self.ekf.x[11] - self.N_rdot*self.ekf.x[11] - self.ekf.x[11]*(-self.I_yy + self.M_qdot))/(self.I_xx + self.K_pdot), (self.I_yy*self.ekf.x[10] - self.M_qdot*self.ekf.x[10] - self.ekf.x[10]*(-self.I_zz + self.N_rdot))/(self.I_xx + self.K_pdot)])
    # test1 = np.array([0])
    # test2 = np.array([(self.Z_wdot*self.ekf.x[8] - self.ekf.x[8]*self.m - self.ekf.x[8]*(self.Y_vdot - self.m))/(self.I_xx + self.K_pdot)])
    # test3 = np.array([(-self.Y_vdot*self.ekf.x[7] + self.ekf.x[7]*self.m - self.ekf.x[7]*(-self.Z_wdot + self.m))/(self.I_xx + self.K_pdot)])
    # test4 = np.array([(-self.K_p - 2*self.K_pp*np.abs(self.ekf.x[9]))/(self.I_xx + self.K_pdot)])
    # test5 = np.array([(self.I_zz*self.ekf.x[11] - self.N_rdot*self.ekf.x[11] - self.ekf.x[11]*(-self.I_yy + self.M_qdot))/(self.I_xx + self.K_pdot)])
    # test6 = np.array([(self.I_yy*self.ekf.x[10] - self.M_qdot*self.ekf.x[10] - self.ekf.x[10]*(-self.I_zz + self.N_rdot))/(self.I_xx + self.K_pdot)])
    # rospy.loginfo(test1)
    # rospy.loginfo(test2)
    # rospy.loginfo(test3)
    # rospy.loginfo(test4)
    # rospy.loginfo(test5)
    # rospy.loginfo(test6)
    #
    # variables = {
    #     'self.Z_wdot': self.Z_wdot,
    #     'self.ekf.x[8]': self.ekf.x[8][0],
    #     'self.m': self.m,
    #     'self.Y_vdot': self.Y_vdot,
    #     'self.I_xx': self.I_xx,
    #     'self.K_pdot': self.K_pdot,
    #     'self.ekf.x[7]': self.ekf.x[7].item(),
    #     'self.Z_wdot': self.Z_wdot,
    #     'self.ekf.x[9]': self.ekf.x[9],
    #     'self.I_zz': self.I_zz,
    #     'self.N_rdot': self.N_rdot,
    #     'self.ekf.x[11]': self.ekf.x[11],
    #     'self.I_yy': self.I_yy,
    #     'self.M_qdot': self.M_qdot,
    #     'self.ekf.x[10]': self.ekf.x[10],
    #     'self.K_p': self.K_p,
    #     'self.K_pp': self.K_pp,
    #     'self.N_r': self.N_r,
    #     'self.N_rr': self.N_rr
    # }
    #
    # # Log the shape of each variable
    # for var_name, variable in variables.iteritems():  # Use iteritems() for Python 2.7
    #     if isinstance(variable, np.ndarray):
    #         rospy.loginfo("%s is an array with shape: %s", var_name, variable.shape)
    #     else:
    #         rospy.loginfo("%s is not an array. Type: %s", var_name, type(variable))