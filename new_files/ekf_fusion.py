import rospy
import geometry_msgs.msg import #type of message we need
import filterpy
#... other libraries

class data_collect: #make it one class

    def __init__(self):
        rospy.init_node('ekf_fusion_node', anonymous=True)

        # Publisher for the EKF state
        self.state_pub = rospy.Publisher("/ekf_state_topic", StateType, queue_size=10)

        sub_imu = "" #name all nodes to which sensors publish
        #pressure, imu, ugps x2, dvl

        self.imu_subscriber = rospy.Subscriber(sub_imu, #type of meesage, #self.imu_callback)
        self.ugps_subscriber #same for all the rest

        # Timer to run the EKF at 100 Hz
        self.ekf_timer = rospy.Timer(rospy.Duration(1.0 / 100.0), self.run_ekf) #1/100s for run_ekf funciton

    def imu_callback(self,msg):
        # save new data in varaibles
        # check received time to mark if the data is new

    #same function for others

    #in pressure_callback convert pressure to depth

    #in DVL directly

    #ugps directly from both (use both for pose)

    def get_yaw_from_gps(self): # include this in run ekf, together with check
        #check if both have a current measurement
        #geometry to find yaw

    def run_ekf(self):
        #check ugps, if yes get_yaw
        #run ekf
        # each update uses if and flag for new data
        state_msg = StateType() #make a state message out of ekf estiamte
        self.state_pub.publish(state_msg) # publish state to the topic

