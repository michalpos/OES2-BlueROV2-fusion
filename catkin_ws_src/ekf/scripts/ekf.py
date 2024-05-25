#!/usr/bin/env python
from filterpy.kalman import ExtendedKalmanFilter
from filterpy.common import Q_discrete_white_noise
import numpy as np

import rospy
from uuvlab_msgs.msg import Imu, DVL, EKF, ThrusterCmd
from sensor_msgs.msg import FluidPressure
from geometry_msgs.msg import PointStamped, Wrench

class EKFFusionNode:
    def __init__(self):
        rospy.init_node('ekf_fusion_node', anonymous=True)
        # Timestamps for the latest sensor data
        self.last_dvl_time = rospy.Time.now()
        self.last_depth_time = rospy.Time.now()
        self.last_imu_time = rospy.Time.now()
        self.last_ugps1_time = rospy.Time.now()
        self.last_ugps2_time = rospy.Time.now()

        # Initialize sensor data variables
        self.dvl_valid = False #extra parameter we get from dvl
        self.dvl_data = None
        self.depth_data = None
        self.imu_data = None
        self.ugps1_data = None
        self.ugps2_data = None
        self.input = np.zeros(6)

        # Publisher for the EKF state
        self.ekf_pub = rospy.Publisher("/bluerov2/ekf", EKF, queue_size=10)

        # Subscribe to sensor topics
        rospy.Subscriber('/bluerov2/dvl/data', DVL, self.dvl_callback)
        rospy.Subscriber('/bluerov2/pressure', FluidPressure, self.pressure_callback)
        rospy.Subscriber('/bluerov2/imu', Imu, self.imu_callback)
        rospy.Subscriber('/waterlinked94/acoustic_position/filtered', PointStamped, self.ugps1_callback)
        rospy.Subscriber('/waterlinked95/acoustic_position/filtered', PointStamped, self.ugps2_callback)
        rospy.Subscriber('/thruster_manager/input', Wrench, self.input_callback)

        self.dt = 0.0285 # sampling rate of the EKF

        # Initialize EKF state and covariance
        #state matrix is (x,y,z,roll,pitch,yaw,x_dot,y_dot,z_dot,roll_dot,pitch_dot,yaw_dot)
        self.ekf = ExtendedKalmanFilter(dim_x=12, dim_z=12, dim_u=6)
        self.ekf.x = np.zeros(12)  # init state
        self.ekf.P = np.eye(12) * 0.001

        # self.ekf.Q = Q_discrete_white_noise(2, dt=self.dt, var=0.05, block_size=6, order_by_dim=False) #variance based on nothing
        self.ekf.Q = np.diag([2.5e-7, 2.5e-6, 1.7e-4, 3e-4, 3e-4, 5e-4, 3.2e-7, 3.8e-7, 6.5e-8, 2.5e-5, 2.5e-5, 6e-4])

        #measurement covariance matrices, for now datasheet based; used as in self.ekf.R = self.R_imu for imu update etc.
        vlarge = 1000000 #large covariance means we ignore the value that is contained there (0)
        self.R_imu = np.diag([vlarge,vlarge,vlarge,7.34e-5, 2.33e-5, 1.8e-4,vlarge,vlarge,vlarge,vlarge,vlarge,vlarge]) #static test
        self.R_ugps1 = np.diag([2.77e-04,5.41e-04,vlarge,vlarge,vlarge,vlarge,vlarge,vlarge,vlarge,vlarge,vlarge,vlarge]) #based on static test
        self.R_ugps2 = np.diag([2.77e-04,5.41e-04,vlarge,vlarge,vlarge,vlarge,vlarge,vlarge,vlarge,vlarge,vlarge,vlarge]) #based on static test
        self.R_yaw_ugps = np.diag([vlarge,vlarge,vlarge,vlarge,vlarge,1.08e-3,vlarge,vlarge,vlarge,vlarge,vlarge,vlarge]) #sum of variances in the ugps
        self.R_dvl = np.diag([vlarge,vlarge,vlarge,vlarge,vlarge,vlarge,5.41e-7,2.41e-6,1.04e-5,vlarge,vlarge,vlarge]) # static test
        self.R_pressure = np.diag([vlarge,vlarge,1.13e-05 ,vlarge,vlarge,vlarge,vlarge,vlarge,vlarge,vlarge,vlarge,vlarge]) # static test

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
        self.K_p = 0
        self.K_pp = 1.19
        self.M_q = 0.8
        self.M_qq = 0.47
        self.N_r = 0
        self.N_rr = 1.5

        #for conversion of imu, not to the report
        self.initial_quat = np.array([0, 0, 0, 1])
        # Quaternion for -90 degrees rotation around Y-axis
        self.rotation_quat = np.array([0, -np.sqrt(2) / 2, 0, np.sqrt(2) / 2])

        # running run_ekf function at desired rate, our 'main loop'
        self.ekf_timer = rospy.Timer(rospy.Duration(self.dt), self.run_ekf)
        rospy.spin()


    #sensor callbacks, so functions that are called when new data arrives, parsing and assigning variables
    def dvl_callback(self, msg): #we need position and velocity in each axis
        #consider dynamic covariance, we don't know how it's calculated
        self.dvl_data = np.array([0,0,0,0,0,0,msg.velocity.x, msg.velocity.y, msg.velocity.z,0,0,0])
        self.dvl_valid = msg.velocity_valid
        self.last_dvl_time = rospy.Time.now()


    def pressure_callback(self, msg):
         depth = self._convert_to_depth(msg) #make sure sign is correct
         self.depth_data = np.array([0,0,depth,0,0,0,0,0,0,0,0,0])
         self.last_depth_time = rospy.Time.now()


    def imu_callback(self, msg):
        #self.imu_data = np.array([0,0,0,msg.roll,msg.pitch,msg.yaw,0,0,0,0,0,0]) #assuming everything was correct
        roll, pitch, yaw = self.process_quaternion(msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w)
        #extra adjustment
        roll_corrected = yaw
        pitch_corrected = pitch
        yaw_corrected = roll

        self.imu_data = np.array([0, 0, 0, roll_corrected, pitch_corrected, yaw_corrected, 0, 0, 0, 0, 0, 0])
        #we don't collect velocity bc it'd be correlated to position, unknown interaction
        self.last_imu_time = rospy.Time.now()

    # def imu_callback(self, msg): #function to put in the report
    #     self.imu_data = np.array([0,0,0,msg.roll,msg.pitch,msg.yaw,0,0,0,0,0,0]) #assuming everything was correct
    #     # we don't collect velocity bc it'd be correlated to position, unknown interaction
    #     self.last_imu_time = rospy.Time.now()


    def ugps1_callback(self, msg): #do we use z? does it make sense at all?
        # remember to move the starting point to where the rov is
        self.ugps1_data = np.array([msg.x,msg.y,0,0,0,0,0,0,0,0,0,0])
        self.last_ugps1_time = rospy.Time.now()


    def ugps2_callback(self, msg):
        self.ugps2_data = np.array([msg.x,msg.y,0,0,0,0,0,0,0,0,0,0])
        self.last_ugps2_time = rospy.Time.now()


    def input_callback(self, msg):
        self.input = np.array([msg.force.x, -msg.force.y, -msg.force.z, -msg.torque.x, -msg.torque.y, -msg.torque.z]) #signs flipped to match coord system
        self.last_input_time = rospy.Time.now()


    def run_ekf(self, event): #main method run in loop
        current_time = rospy.Time.now() #some datapoints could get lost but if kalman runs fast it shouldn't be an issue

        #update input to the thrusters, no code necessary

        # Process UGPS data for yaw measurement if new data is available from both UGPS sensors
        yaw_ugps = self.process_ugps_data_for_yaw(current_time)
        if self.last_yaw_ugps_time < current_time: #check to make sure we don't publish oudated data
            yaw_ugps = None

        #predict step of the kalman filter, own function. Runs everyiteration at specified frequency
        self.predict()

        #subsequent update steps, yaw drift correction
        if yaw_ugps is not None and self.last_yaw_ugps_time > current_time:
            self.ekf.update(z=yaw_ugps, HJacobian=self.HJ_yaw_ugps, Hx=self.Hx_yaw_ugps, R=self.R_yaw_ugps) # update step with yaw

        if self.dvl_data is not None and self.last_dvl_time > current_time and self.dvl_valid is True:         #same update for dvl
            self.ekf.update(z=self.dvl_data, HJacobian=self.HJ_dvl, Hx=self.Hx_dvl, R=self.R_dvl)

        if self.imu_data is not None and self.last_imu_time > current_time:         #same update for imu
            self.ekf.update(z=self.imu_data, HJacobian=self.HJ_imu, Hx=self.Hx_imu, R=self.R_imu)

        if self.depth_data is not None and self.last_depth_time > current_time:         #same update for pressure sensor
            self.ekf.update(z=self.depth_data, HJacobian=self.HJ_pressure, Hx=self.Hx_pressure, R=self.R_pressure)

        if self.ugps1_data is not None and self.last_ugps1_time > current_time:         #same update for ugps1
            self.ekf.update(z=self.ugps1_data, HJacobian=self.HJ_ugps, Hx=self.Hx_ugps, R=self.R_ugps1)

        if self.ugps2_data is not None and self.last_ugps2_time > current_time:         #same update for ugps2
            self.ekf.update(z=self.ugps2_data, HJacobian=self.HJ_ugps, Hx=self.Hx_ugps, R=self.R_ugps2)

        #if there's no new measurement data (unlikely), only prediction will be carried out

        #check for FPS + print on screen, use 'rostopic hz /topic1' in console

        # Publish the estimated state and covariance
        ekf_msg = EKF()
        ekf_msg.header.stamp = rospy.Time.now()
        #.header.frame_id = 'base_link' not necessary
        ekf_msg.priori_estimate = self.ekf.x_prior.tolist()
        ekf_msg.state_estimate = self.ekf.x.tolist()
        ekf_msg.P = self.ekf.P.flatten().tolist()
        ekf_msg.yaw_ugps = yaw_ugps if yaw_ugps is not None else -100.0 #-100.0 is 'false' value
        self.ekf_pub.publish(ekf_msg)


    # additional minor methods
    def _compute_yaw(self):
        yaw = np.arctan2((self.ugps2_data[1] - self.ugps1_data[1]), (self.ugps2_data[0] - self.ugps1_data[0]))
        self.last_yaw_ugps_time = rospy.Time.now()
        return yaw  # assumes 2 (old) is left, 1 (new) is right and yaw increases CW



    def _convert_to_depth(self, msg):
        density_water = 1000  # kg/m^3 for freshwater
        gravity = 9.81  # m/s^2
        atm_pressure = 101325  # Pa (average sea level atmospheric pressure), adjust?
        depth = (msg.fluid_pressure - atm_pressure) / (density_water * gravity)
        return depth


    def process_ugps_data_for_yaw(self, current_time):
        ugps_sample_time = 0.5  # sampling time of ugps system, compute yaw only if both have a recent measurement
        if self.ugps1_data and self.ugps2_data:
            if (self.last_yaw_ugps_time < self.last_ugps1_time and self.last_yaw_ugps_time < self.last_ugps2_time
                    and abs(self.last_ugps1_time - self.ugps2_data) < ugps_sample_time):
                # if there was measurements after last yaw calculation, won't use same twice; if ugps1 and 2 are 1 sample apart
                # Compute yaw from UGPS1 and UGPS2 data
                yaw_ugps = self._compute_yaw()
                return yaw_ugps
        return None


    def predict(self):  # own predict step of the ekf, one from the package couldn't be used
        self._propagate_state()
        self._get_F_Jacobian()
        self.ekf.P = np.dot(self.ekf.F, self.ekf.P).dot(self.ekf.F.T) + self.ekf.Q
        #save predicted values
        self.ekf.x_prior = np.copy(self.ekf.x)
        self.ekf.P_prior = np.copy(self.ekf.P)


    def _propagate_state(self):  # nonlinear state transition, use model and discretize
        # innvoation is the CT model, further it is discretized using forward euler
        # state vector is x=[x,y,z,phi,theta,psi,u,v,w,p,q,r]
        innovation = np.zeros(12)  # clear the array
        for i in range(6):  # innovation in CT is equal to the velocity which is contained in other states
            innovation[i] = self.ekf.x[i + 6]
        # the actual model part
        M_rb = np.diag([self.m, self.m, self.m, self.I_xx, self.I_yy, self.I_zz])
        M_a = np.diag([self.X_udot, self.Y_vdot, self.Z_wdot, self.K_pdot, self.M_qdot, self.N_rdot])
        M = M_rb + M_a  # constant mass/inertia matrix, combined rigid body and added mass
        # noinspection PyTypeChecker
        C_rb = np.array([[0, 0, 0, 0, self.m * self.ekf.x[8], -self.m * self.ekf.x[7]],
                         [0, 0, 0, -self.m * self.ekf.x[8], 0, self.m * self.ekf.x[6]],
                         [0, 0, 0, self.m * self.ekf.x[7], -self.m * self.ekf.x[6], 0],
                         [0, self.m * self.ekf.x[8], -self.m * self.ekf.x[7], 0, -self.I_zz * self.ekf.x[11],
                          -self.I_yy * self.ekf.x[10]],
                         [-self.m * self.ekf.x[8], 0, self.m * self.ekf.x[6], self.I_zz * self.ekf.x[11], 0,
                          self.I_xx * self.ekf.x[9]],
                         [self.m * self.ekf.x[7], -self.m * self.ekf.x[6], 0, self.I_yy * self.ekf.x[10],
                          -self.I_xx * self.ekf.x[9], 0]])
        # noinspection PyTypeChecker
        C_a = np.array([[0, 0, 0, 0, -self.Z_wdot * self.ekf.x[8], self.Y_vdot * self.ekf.x[7]],
                        [0, 0, 0, self.Z_wdot * self.ekf.x[8], 0, -self.X_udot * self.ekf.x[6]],
                        [0, 0, 0, -self.Y_vdot * self.ekf.x[7], self.X_udot * self.ekf.x[6], 0],
                        [0, -self.Z_wdot * self.ekf.x[8], self.Y_vdot * self.ekf.x[7], 0, self.N_rdot * self.ekf.x[11],
                         self.M_qdot * self.ekf.x[10]],
                        [self.Z_wdot * self.ekf.x[8], 0, -self.X_udot * self.ekf.x[6], -self.N_rdot * self.ekf.x[11], 0,
                         -self.K_pdot * self.ekf.x[9]],
                        [-self.Y_vdot * self.ekf.x[7], self.X_udot * self.ekf.x[6], 0, -self.M_qdot * self.ekf.x[10],
                         self.K_pdot * self.ekf.x[9], 0]])
        D = np.diag([self.X_u + (self.X_uu * np.abs(self.ekf.x[6])), self.Y_v + (self.Y_vv * np.abs(self.ekf.x[7])),
                     self.Z_w + (self.Z_ww * np.abs(self.ekf.x[8])), self.K_p + (self.K_pp * np.abs(self.ekf.x[9])),
                     self.M_q + (self.M_qq * np.abs(self.ekf.x[10])), self.N_r + (self.N_rr * np.abs(self.ekf.x[11]))])
        C = C_rb + C_a + D  # combined coriolis and damping matrices, all are multiplied by velocity

        innovation[6:12] = np.dot(np.linalg.inv(M), (self.input - np.dot(C, self.ekf.x[
                                                                            6:12])))  # adds the model in the bottom right corner of the matrix
        # CT model is complete, discretization
        print(innovation[6:12])
        self.ekf.x = self.ekf.x + innovation * self.dt  # forward euler discretization, does it work like that?


    def _get_F_Jacobian(self):  # linearization to get F matrix and compute covariance
        Jacobian_cont = np.zeros((12, 12))
        for i in range(6):  # add diagonal part in top right corner
            # ex: innovation[0,6] = 1 # Set the diagonal offset by 6
            Jacobian_cont[i, i + 6] = 1
        # remaining part of the jacobian matrix, calculated using separate script on the non linear model
        # noinspection PyTypeChecker
        Jacobian_cont[6:12, 6:12] = np.array([[(-self.X_u - 2 * self.X_uu * np.abs(self.ekf.x[6])) / (
                    self.X_udot + self.m), -self.ekf.x[11] * (self.Y_vdot - self.m) / (self.X_udot + self.m),
                                               -self.ekf.x[10] * (-self.Z_wdot + self.m) / (self.X_udot + self.m), 0,
                                               (self.Z_wdot * self.ekf.x[8] - self.ekf.x[8] * self.m) / (
                                                           self.X_udot + self.m),
                                               (-self.Y_vdot * self.ekf.x[7] + self.ekf.x[7] * self.m) / (
                                                           self.X_udot + self.m)],
                                              [-self.ekf.x[11] * (-self.X_udot + self.m) / (self.Y_vdot + self.m),
                                               (-self.Y_v - 2 * self.Y_vv * np.abs(self.ekf.x[7])) / (
                                                           self.Y_vdot + self.m),
                                               -self.ekf.x[9] * (self.Z_wdot - self.m) / (self.Y_vdot + self.m),
                                               (-self.Z_wdot * self.ekf.x[8] + self.ekf.x[8] * self.m) / (
                                                           self.Y_vdot + self.m), 0,
                                               (self.X_udot * self.ekf.x[6] - self.ekf.x[6] * self.m) / (
                                                           self.Y_vdot + self.m)],
                                              [-self.ekf.x[10] * (self.X_udot - self.m) / (self.Z_wdot + self.m),
                                               -self.ekf.x[9] * (-self.Y_vdot + self.m) / (self.Z_wdot + self.m),
                                               (-self.Z_w - 2 * self.Z_ww * np.abs(self.ekf.x[8])) / (
                                                           self.Z_wdot + self.m),
                                               (self.Y_vdot * self.ekf.x[7] - self.ekf.x[7] * self.m) / (
                                                           self.Z_wdot + self.m),
                                               (-self.X_udot * self.ekf.x[6] + self.ekf.x[6] * self.m) / (
                                                           self.Z_wdot + self.m), 0],
                                              [0, (self.Z_wdot * self.ekf.x[8] - self.ekf.x[8] * self.m - self.ekf.x[
                                                  8] * (self.Y_vdot - self.m)) / (self.I_xx + self.K_pdot), (
                                                           -self.Y_vdot * self.ekf.x[7] + self.ekf.x[7] * self.m -
                                                           self.ekf.x[7] * (-self.Z_wdot + self.m)) / (
                                                           self.I_xx + self.K_pdot),
                                               (-self.K_p - 2 * self.K_pp * np.abs(self.ekf.x[9])) / (
                                                           self.I_xx + self.K_pdot), (
                                                           self.I_zz * self.ekf.x[11] - self.N_rdot * self.ekf.x[11] -
                                                           self.ekf.x[11] * (-self.I_yy + self.M_qdot)) / (
                                                           self.I_xx + self.K_pdot), (
                                                           self.I_yy * self.ekf.x[10] - self.M_qdot * self.ekf.x[10] -
                                                           self.ekf.x[10] * (-self.I_zz + self.N_rdot)) / (
                                                           self.I_xx + self.K_pdot)],
                                              [(-self.Z_wdot * self.ekf.x[8] + self.ekf.x[8] * self.m - self.ekf.x[
                                                  8] * (-self.X_udot + self.m)) / (self.I_yy + self.M_qdot), 0, (
                                                           self.X_udot * self.ekf.x[6] - self.ekf.x[6] * self.m -
                                                           self.ekf.x[6] * (self.Z_wdot - self.m)) / (
                                                           self.I_yy + self.M_qdot), (
                                                           -self.I_zz * self.ekf.x[11] + self.N_rdot * self.ekf.x[11] -
                                                           self.ekf.x[11] * (self.I_xx - self.K_pdot)) / (
                                                           self.I_yy + self.M_qdot),
                                               (-self.M_q - 2 * self.M_qq * np.abs(self.ekf.x[10])) / (
                                                           self.I_yy + self.M_qdot), (
                                                           -self.I_xx * self.ekf.x[9] + self.K_pdot * self.ekf.x[9] -
                                                           self.ekf.x[9] * (self.I_zz - self.N_rdot)) / (
                                                           self.I_yy + self.M_qdot)],
                                              [(self.Y_vdot * self.ekf.x[7] - self.ekf.x[7] * self.m - self.ekf.x[7] * (
                                                          self.X_udot - self.m)) / (self.I_zz + self.N_rdot), (
                                                           -self.X_udot * self.ekf.x[6] + self.ekf.x[6] * self.m -
                                                           self.ekf.x[6] * (-self.Y_vdot + self.m)) / (
                                                           self.I_zz + self.N_rdot), 0, (
                                                           -self.I_yy * self.ekf.x[10] + self.M_qdot * self.ekf.x[10] -
                                                           self.ekf.x[10] * (-self.I_xx + self.K_pdot)) / (
                                                           self.I_zz + self.N_rdot), (
                                                           self.I_xx * self.ekf.x[9] - self.K_pdot * self.ekf.x[9] -
                                                           self.ekf.x[9] * (self.I_yy - self.M_qdot)) / (
                                                           self.I_zz + self.N_rdot),
                                               (-self.N_r - 2 * self.N_rr * np.abs(self.ekf.x[11])) / (
                                                           self.I_zz + self.N_rdot)]])
        self.ekf.F = np.identity(12) + Jacobian_cont * self.dt  # discretize jacobian of continuous model


    #define Hx function - that is the actual measurement function that takes states in
    #define H for each sensor - linearized Hx around an operating point (input), matrix
    def HJ_yaw_ugps(self, state): #used to get jacobian of measurement function(H matrix), allows to compute kalman gain
        HJacobian =  np.zeros((12, 12))
        HJacobian[5,5] = 1
        return HJacobian


    def Hx_yaw_ugps(self, state): #nonlinear measurement function, used to compute measurement based on state; projects state to measurement space
        Hx = np.array([0,0,0,0,0,state[5],0,0,0,0,0,0])
        return Hx


    def HJ_dvl(self, state):
        HJacobian =  np.zeros((12, 12))
        HJacobian[6, 6] = 1
        HJacobian[7, 7] = 1
        HJacobian[8, 8] = 1
        return HJacobian


    def Hx_dvl(self, state):
        Hx = np.array([0,0,0,0,0,0,state[6],state[7],state[8],0,0,0])
        return Hx


    def HJ_imu(self, state):
        HJacobian =  np.zeros((12, 12))
        HJacobian[3, 3] = 1
        HJacobian[4, 4] = 1
        HJacobian[5, 5] = 1
        return HJacobian


    def Hx_imu(self, state):
        Hx = np.array([0,0,0,state[3],state[4],state[5],0,0,0,0,0,0])
        return Hx


    def HJ_pressure(self, state):
        HJacobian =  np.zeros((12, 12))
        HJacobian[2, 2] = 1
        return HJacobian


    def Hx_pressure(self, state):
        Hx = np.array([0,0,state[2],0,0,0,0,0,0,0,0,0])
        return Hx


    def HJ_ugps(self, state):
        HJacobian =  np.zeros((12, 12))
        HJacobian[0, 0] = 1
        HJacobian[1, 1] = 1
        return HJacobian


    def Hx_ugps(self, state):
        Hx = np.array([state[0],state[1],0,0,0,0,0,0,0,0,0,0])
        return Hx


    #quaternion conversion because we had to, doesn't go in the report


    def quaternion_multiply(self, q1, q2):
        x1, y1, z1, w1 = q1
        x2, y2, z2, w2 = q2
        return np.array([
            w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
        ])


    def normalize_quaternion(self, q):
        norm_val = np.linalg.norm(q)
        return q / norm_val


    def quaternion_to_euler(self, q):
        x, y, z, w = q

        # roll (x-axis rotation)
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = np.arctan2(sinr_cosp, cosr_cosp)

        # pitch (y-axis rotation)
        sinp = 2 * (w * y - z * x)
        if np.abs(sinp) >= 1:
            pitch = np.sign(sinp) * np.pi / 2  # use 90 degrees if out of range
        else:
            pitch = np.arcsin(sinp)

        # yaw (z-axis rotation)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw


    def normalize_angle(self, angle):
        return (angle + np.pi) % (2 * np.pi) - np.pi


    def adjust_roll_angle(self, angle):
        if angle > 0:
            return angle - np.pi
        else:
            return angle + np.pi


    def correct_orientation(self, initial_quat, current_quat):
        # Inverse of the initial quaternion
        initial_quat_conjugate = np.array([-initial_quat[0], -initial_quat[1], -initial_quat[2], initial_quat[3]])

        # Combine the current quaternion with the inverse of the initial quaternion
        corrected_quat = self.quaternion_multiply(initial_quat_conjugate, current_quat)

        # Normalize the corrected quaternion
        corrected_quat = self.normalize_quaternion(corrected_quat)

        return corrected_quat


    def process_quaternion(self, orientationX, orientationY, orientationZ, orientationW):
        # Quaternion for IMU orientation
        imu_quat = np.array([orientationX, orientationY, orientationZ, orientationW])

        # Combine the rotations
        combined_quat = self.quaternion_multiply(self.rotation_quat, imu_quat)

        # Normalize the combined quaternion
        combined_quat = self.normalize_quaternion(combined_quat)

        # Convert the combined quaternion to Euler angles
        roll, pitch, yaw = self.quaternion_to_euler(combined_quat)

        # Adjust the roll angle
        roll = self.adjust_roll_angle(roll)

        # Normalize the pitch and yaw angles to the range -pi to pi
        pitch = self.normalize_angle(pitch)
        yaw = self.normalize_angle(yaw)

        return roll, pitch, yaw

if __name__ == '__main__':
    try:
        ekf_node = EKFFusionNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

        # legacy code, not used but maybe some methods will be useful
        # # innvoation is the CT model, further it is discretized using forward euler
        # # state vector is x=[x,y,z,phi,theta,psi,u,v,w,p,q,r]
        # innovation = np.zeros((12, 12))  # clear the array
        # for i in range(6):  # innovation in CT is equal to the velocity which is contained in other states
        #     # ex: innovation[0,6] = self.ekf.x[6] # Set the diagonal offset by 6
        #     innovation[i, i + 6] = self.ekf.x[i + 6]
        # # the actual model part
        # M_rb = np.diag([self.m, self.m, self.m, self.I_xx, self.I_yy, self.I_zz])
        # M_a = np.diag([self.X_udot, self.Y_vdot, self.Z_wdot, self.K_pdot, self.M_qdot, self.N_rdot])
        # M = M_rb + M_a  # constant mass/inertia matrix, combined rigid body and added mass
        # # noinspection PyTypeChecker
        # C_rb = np.array([0, 0, 0, 0, self.m * self.ekf.x[8], -self.m * self.ekf.x[7]],
        #                 [0, 0, 0, -self.m * self.ekf.x[8], 0, self.m * self.ekf.x[6]],
        #                 [0, 0, 0, self.m * self.ekf.x[7], -self.m * self.ekf.x[6], 0],
        #                 [0, self.m * self.ekf.x[8], -self.m * self.ekf.x[7], 0, -self.I_zz * self.ekf.x[11],
        #                  -self.I_yy * self.ekf.x[10]],
        #                 [-self.m * self.ekf.x[8], 0, self.m * self.ekf.x[6], self.I_zz * self.ekf.x[11], 0,
        #                  self.I_xx * self.ekf.x[9]],
        #                 [self.m * self.ekf.x[7], -self.m * self.ekf.x[6], 0, self.I_yy * self.ekf.x[10],
        #                  -self.I_xx * self.ekf.x[9], 0])
        # # noinspection PyTypeChecker
        # C_a = np.array([0, 0, 0, 0, -self.Z_wdot * self.ekf.x[8], self.Y_vdot * self.ekf.x[7]],
        #                [0, 0, 0, self.Z_wdot * self.ekf.x[8], 0, -self.X_udot * self.ekf.x[6]],
        #                [0, 0, 0, -self.Y_vdot * self.ekf.x[7], self.X_udot * self.ekf.x[6], 0],
        #                [0, -self.Z_wdot * self.ekf.x[8], self.Y_vdot * self.ekf.x[7], 0, self.N_rdot * self.ekf.x[11],
        #                 self.M_qdot * self.ekf.x[10]],
        #                [self.Z_wdot * self.ekf.x[8], 0, -self.X_udot * self.ekf.x[6], -self.N_rdot * self.ekf.x[11], 0,
        #                 -self.K_pdot * self.ekf.x[9]],
        #                [-self.Y_vdot * self.ekf.x[7], self.X_udot * self.ekf.x[6], 0, -self.M_qdot * self.ekf.x[10],
        #                 self.K_pdot * self.ekf.x[9], 0])
        # D = np.diag([self.X_u + (self.X_uu * np.abs(self.ekf.x[6])), self.Y_v + (self.Y_vv * np.abs(self.ekf.x[7])),
        #              self.Z_w + (self.Z_ww * np.abs(self.ekf.x[8])), self.K_p + (self.K_pp * np.abs(self.ekf.x[9])),
        #              self.M_q + (self.M_qq * np.abs(self.ekf.x[10])), self.N_r + (self.N_rr * np.abs(self.ekf.x[11]))])
        # C = C_rb + C_a + D  # combined coriolis and damping matrices, all are multiplied by velocity
        # innovation[6:12, 6:12] = np.linalg.inv(M) * (
        #             self.input - C * self.ekf.x[6:12])  # adds the model in the bottom right corner of the matrix
        # dt_vector = np.full((12, 1), self.dt)
        # # CT model is complete, discretization
        # self.ekf.x += innovation * dt_vector  # forward euler discretization, does it work like that?