import rospy
import numpy as np
from geometry_msgs.msg import Wrench

# For help contact Fredrik Fogh Soerensen (ffso@energy.aau.dk)

# Values to be changed:
# - rospy.Rate(value) :: Sample time in Hz.
# - amplitude   :: Amplitude of the sine function.
# - period      :: Period of the sine function.
# - offset      :: Offset for the mean value of the sine function.
# - dur         :: Duration of the function in seconds.

if __name__ == '__main__':
    try:
        pub = rospy.Publisher('/bluerov2/thruster_manager/input', Wrench, queue_size=10) # Start Publisher publishing Wrench msgs on topic named: /bluerov2/thruster_manager/input.
        rospy.init_node('input_generator', anonymous=True) # Initialize the ROS node to allow it to communicate with the ROS Master (node name = input_generator).
        # anonymous = True ensures that your node has a unique name by adding random numbers to the end of NAME.
        ## Parameters to be changed:
        rate = rospy.Rate(50) #Sample time in Hz, here 50 Hz.
        amplitude=1.0 # Amplitude of the sine function.
        period=2.0 # Period of the sine function., min 0.5s (bc of sampling)
        offset=0.0 # Offset for the mean value of the sine function.
        dur = 10 # Duration in seconds.
        start_time = rospy.get_time() # Get inital time.
        while not rospy.is_shutdown(): # Do while ROS is running.
            elapsed_time = rospy.get_time() - start_time # Calculate the elapsed time in seconds.
            if elapsed_time > dur: #stop producing values after duration.
           	msg = Wrench() # Call an empty Wrench msgs.
            	msg.force.z = float(thrust) # Fill in the value at the desired DOF (Here heave (msg.force.z)).
            	pub.publish(msg) # Publish the msg to the ROS Master.
                break # Break the program and kills the node.

            thrust = amplitude * np.sin(2.0 * np.pi * elapsed_time / period) + offset
            msg = Wrench() # Call an empty Wrench msgs.
            msg.force.z = float(thrust) # Fill in the value at the desired DOF (Here heave (msg.force.z)).
            pub.publish(msg) # Publish the msg to the ROS Master.
	    rate.sleep()
    except rospy.ROSInterruptException:
        pass
# ;'''''''''''''''''''''''''''''''''''''''/'













'' \
''
