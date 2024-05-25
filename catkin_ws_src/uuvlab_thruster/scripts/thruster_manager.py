import rospy
import yaml
import numpy as np #import interp
from numpy.linalg import pinv
from geometry_msgs.msg import Wrench
from uuvlab_msgs.msg import ThrusterPWMs

class ThrusterManager:
	def __init__(self):
		self.pub_thrusterpwm = rospy.Publisher('thrusters/pwm', ThrusterPWMs, queue_size=1)
		config_file = rospy.get_param("~config_file","None") #"../config/bluerov2_heavy/thruster_manager.yaml"
		tam_file = rospy.get_param("~tam_file","None") #"../config/bluerov2_heavy/TAM.yaml"
		if(config_file != "None"):
		    print("config_file found: " + config_file)
		    with open(config_file, "r") as yaml_file:
			doc = yaml.load(yaml_file,Loader=yaml.SafeLoader)
			self.thrust_input = doc["thruster_manager"]["conversion_fcn_params"]["input"]
			self.thrust_output = doc["thruster_manager"]["conversion_fcn_params"]["output"]
			#print(thrust_pwm)
		else:
		    print("config_file: " + config_file)

		if(tam_file != "None"):
		    print("tam_file found: " + tam_file)
		    with open(tam_file, "r") as yaml_file:
			doc = yaml.load(yaml_file,Loader=yaml.SafeLoader)
			self.tam = doc["tam"]
			self.itam = np.linalg.pinv(self.tam)
		else:
		    print("tam_file: " + tam_file)


	def getInput(self,forces):
		force_input = np.array([forces.force.x, forces.force.y, forces.force.z, forces.torque.x, forces.torque.y, forces.torque.z])
		thrust_force = np.matmul(self.itam,force_input)
		thrust_pwm = np.interp(thrust_force,self.thrust_output,self.thrust_input)
		return thrust_pwm

	def sendThrustInput(self,forces):
		thrust_pwm = self.getInput(forces)
		ThrusterPWMsMsg = ThrusterPWMs()
		ThrusterPWMsMsg.header.stamp = rospy.Time.now()
		ThrusterPWMsMsg.t0 = thrust_pwm[0]
		ThrusterPWMsMsg.t1 = thrust_pwm[1]
		ThrusterPWMsMsg.t2 = thrust_pwm[2]
		ThrusterPWMsMsg.t3 = thrust_pwm[3]
		ThrusterPWMsMsg.t4 = thrust_pwm[4]
		ThrusterPWMsMsg.t5 = thrust_pwm[5]
		ThrusterPWMsMsg.t6 = thrust_pwm[6]
		ThrusterPWMsMsg.t7 = thrust_pwm[7]
		self.pub_thrusterpwm.publish(ThrusterPWMsMsg)
		return thrust_pwm


