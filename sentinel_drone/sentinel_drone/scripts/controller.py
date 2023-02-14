#!/usr/bin/env python3

"""
Controller for the drone
"""


# standard imports
import copy
import time

# third-party imports
import scipy.signal
import numpy as np
import rospy
from geometry_msgs.msg import PoseArray
from pid_tune.msg import PidTune
from sentinel_drone_driver.msg import PIDError, RCMessage
from sentinel_drone_driver.srv import CommandBool, CommandBoolResponse


MIN_THROTTLE = 1200             # Minimum throttle value, so that the drone does not hit the ground 
MIN_ROLL = 0
MIN_PITCH = 0
BASE_THROTTLE = 0               # Base value of throttle for hovering. NOTE: Unlike simulator, the drone does not hover at 1500 value (HINT: Hovering value in hardware will range somewhere between 1280 and 1350). Also, the hovering thrust changes with battery % . Hence this will varry with time and the correct value will have to be generated using integral term to remove steady state error 
MAX_THROTTLE = 1400             # Maximum throttle value, so that the drone does not accelerate in uncontrollable manner and is in control. NOTE: DO NOT change this value, if changing, be very careful operating the drone 
MAX_ROLL = 1000
MAX_PITCH = 1000
SUM_ERROR_THROTTLE_LIMIT = 0    # Integral anti windup sum value. Decide this value by calcuating carefully
filter_val =0.05

# Similarly, create upper and lower limits, base value, and max sum error values for roll and pitch




PID_OUTPUT_VALUES = [[], [], []] # This will be used to store data for filtering purpose

class DroneController:
    def __init__(self):

        self.rc_message = RCMessage()
        self.drone_whycon_pose_array = PoseArray()
        self.last_whycon_pose_received_at = 0
        self.is_flying = False
        # [x,y,z]
        self.drone_position = [0.0,0.0,0.0] 
        self.setpoint = [0,0, 14]         # Setpoints for x, y, z respectively      
        self.error      = [0, 0, 0]         # Error for roll, pitch and throttle
        # self.Kp = [40.6,40.6,49.5]
        # self.Ki = [0,0,0.0] #197
        # self.Kd = [1200,1200,450] #1223
        self.Kp = [0,0,0]
        self.Ki = [0,0,0.8] #197
        self.Kd = [0,0,0] #1223
        self.prev_values = [0,0,0]
        self.min_values = [1000,1000,1200]
        self.max_values = [2000,2000,1400]
        self.sample_freq = 28                    #control loop frequency of 28hz
        self.prevI = [0,0,0]                     #roll,pitch,throttle

        # Initialize rosnode

        node_name = "controller"
        rospy.init_node(node_name)
        rospy.on_shutdown(self.shutdown_hook)

        # Create subscriber for WhyCon 

        rospy.Subscriber("/whycon/poses", PoseArray, self.whycon_poses_callback)


        # Similarly create subscribers for pid_tuning_altitude, pid_tuning_roll, pid_tuning_pitch and any other subscriber if required

        rospy.Subscriber('/pid_tuning_altitude',PidTune,self.altitude_set_pid)
        rospy.Subscriber('/pid_tuning_pitch',PidTune,self.pitch_set_pid)
        rospy.Subscriber('/pid_tuning_roll',PidTune,self.roll_set_pid)
        # Create publisher for sending commands to drone 

        self.rc_pub = rospy.Publisher(
            "/sentinel_drone/rc_command", RCMessage, queue_size=1
        )

        # Create publisher for publishing errors for plotting in plotjuggler 

        self.pid_error_pub = rospy.Publisher(
            "/sentinel_drone/pid_error", PIDError, queue_size=1
        )


    def whycon_poses_callback(self, msg):
        self.last_whycon_pose_received_at = rospy.get_rostime().secs
        self.drone_whycon_pose_array = msg
        inp = [msg.poses[0].position.x,msg.poses[0].position.y,msg.poses[0].position.z]
        self.drone_position[0] = inp[0]*filter_val + (1-filter_val)*self.drone_position[0]
        self.drone_position[1] = inp[1]*filter_val + (1-filter_val)*self.drone_position[1]
        self.drone_position[2] = inp[2]*filter_val + (1-filter_val)*self.drone_position[2]

        
        # self.drone_position[0] = msg.poses[0].position.x
        # self.drone_position[1] = msg.poses[0].position.y
        # self.drone_position[2] = msg.poses[0].position.z


    # Callback functions for PID Tuning package
    def altitude_set_pid(self,alt):
        pass
        self.Kp[2] = alt.Kp * 0.05
        self.Ki[2] = alt.Ki * 0.008
        self.Kd[2] = alt.Kd * 0.3


    def pitch_set_pid(self,pitch):
        pass
        self.Kp[1] = pitch.Kp * 0.06 
        self.Ki[1] = pitch.Ki * 0.008
        self.Kd[1] = pitch.Kd * 0.3

    def roll_set_pid(self, roll):
        pass
        self.Kp[0] = roll.Kp * 0.06 
        self.Ki[0] = roll.Ki * 0.008
        self.Kd[0] = roll.Kd * 0.3


    def pid(self):          # PID algorithm

        #calculating error
        error = [0,0,0]
        error[0] = self.drone_position[0]  - self.setpoint[0]
        error[1] = self.drone_position[1] - self.setpoint[1]
        error[2] = self.drone_position[2] - self.setpoint[2]
        print("pitch",error[1])
        #using clipping technique to control integral part of throttle as roll and pitch have no integral part
        I_throttle = (self.prevI[2] + error[2]) * self.Ki[2]
        if (error[2]>0 and I_throttle<0) or (error[2]<0 and I_throttle>0) or I_throttle<-30 or I_throttle>30 or error[2]<-3 or error[2]>3:
            I_throttle = 0; 
        I_roll = (self.prevI[0] + error[0]) * self.Ki[0]
        I_pitch = (self.prevI[1] + error[1]) * self.Ki[1]

        #PID formula
        out_throttle = (self.Kp[2]*error[2]) + (self.Kd[2]*(error[2]-self.prev_values[2])) + I_throttle
        out_roll = (self.Kp[0]*error[0]) + (self.Kd[0]*(error[0]-self.prev_values[0])) + I_roll
        out_pitch = (self.Kp[1]*error[1]) + (self.Kd[1]*(error[1]-self.prev_values[1])) + I_pitch

        #setting commands for drone from PID values
        self.rc_message.rc_throttle = 1100 + int(out_throttle)
        self.rc_message.rc_roll = 1500 - int(out_roll)
        self.rc_message.rc_pitch = 1500 + int(out_pitch)

        #updating previous error
        self.prev_values = error 

        #udating previous error sum for integral part
        self.prevI = [I_roll, I_pitch, I_throttle]
        

        #clipping output for drone requirements
        print(self.Kp[2])
        print("integral",I_throttle)
        print("error", error[2])
        print("out",self.rc_message.rc_throttle)
        if self.rc_message.rc_throttle > self.max_values[2]:
            self.rc_message.rc_throttle = self.max_values[2]
        if self.rc_message.rc_pitch > self.max_values[1]:
            self.rc_message.rc_pitch = self.max_values[1]                                           
        if self.rc_message.rc_roll > self.max_values[0]:
            self.rc_message.rc_roll = self.max_values[0]
        if self.rc_message.rc_throttle < self.min_values[2]:
            self.rc_message.rc_throttle = self.min_values[2]
        if self.rc_message.rc_pitch < self.min_values[1]:
            self.rc_message.rc_pitch = self.min_values[1]                                           
        if self.rc_message.rc_roll < self.min_values[0]:
            self.rc_message.rc_roll = self.min_values[0]
    
        
        # Send constant 1500 to rc_message.rc_yaw
        self.rc_message.rc_yaw = np.uint16(1500)
        self.publish_data_to_rpi(self.rc_message.rc_roll, self.rc_message.rc_pitch, self.rc_message.rc_throttle)
        
        # Publish error messages for plotjuggler debugging 

        self.pid_error_pub.publish(
            PIDError(
                roll_error=self.error[0],
                pitch_error=0,#pitch_error=self.error[1],
                throttle_error=0,#throttle_error=self.error[2],
                yaw_error=0,
                zero_error=0,
            )
        )




    def publish_data_to_rpi(self, roll, pitch, throttle):

        self.rc_message.rc_throttle = np.uint16(throttle)
        self.rc_message.rc_roll = np.uint16(roll)
        self.rc_message.rc_pitch = np.uint16(pitch)
        self.rc_message.rc_yaw = np.uint16(1500)
        self.rc_pub.publish(self.rc_message)

        # NOTE: There is noise in the WhyCon feedback and the noise gets amplified because of derivative term, this noise is multiplied by high Kd gain values and create spikes in the output. 
        #       Sending data with spikes to the drone makes the motors hot and drone vibrates a lot. To reduce the spikes in output, it is advised to pass the output generated from PID through a low pass filter.
        #       An example of a butterworth low pass filter is shown here, you can implement any filter you like. Before implementing the filter, look for the noise yourself and compare the output of unfiltered data and filtered data 
        #       Filter adds delay to the signal, so there is a tradeoff between the noise rejection and lag. More lag is not good for controller as it will react little later. 
        #       Alternatively, you can apply filter on the source of noisy data i.e. WhyCon position feedback instead of applying filter to the output of PID 
        #       The filter implemented here is not the best filter, tune this filter that has the best noise rejection and less delay. 

        # BUTTERWORTH FILTER
        # span = 15 
        # for index, val in enumerate([roll, pitch, throttle]):
        #     PID_OUTPUT_VALUES[index].append(val)
        #     if len(PID_OUTPUT_VALUES[index]) == span:
        #         PID_OUTPUT_VALUES[index].pop(0)
        #     if len(PID_OUTPUT_VALUES[index]) != span-1:
        #         return
        #     order = 3 
        #     fs = 60           # Sampling frequency (camera FPS)
        #     fc = 5            # Low pass cutoff frequency
        #     nyq = 0.5 * fs    # Nyquist frequency
        #     wc = fc / nyq
        #     b, a = scipy.signal.butter(N=order, Wn=wc, btype='lowpass', analog=False, output='ba')
        #     filtered_signal = scipy.signal.lfilter(b, a, PID_OUTPUT_VALUES[index])
        #     if index == 0:
        #         self.rc_message.rc_roll = np.uint16(filtered_signal[-1])
        #     elif index == 1:
        #         self.rc_message.rc_pitch = np.uint16(filtered_signal[-1])
        #     elif index == 2:
        #         self.rc_message.rc_throttle = np.uint16(filtered_signal[-1])


        # Check the bounds of self.rc_message.rc_throttle, self.rc_message.rc_roll and self.rc_message.rc_pitch aftre rfiltering 
        # Similarly add bounds for pitch yaw and throttle 

        

    # This function will be called as soon as this rosnode is terminated. So we disarm the drone as soon as we press CTRL + C. 
    # If anything goes wrong with the drone, immediately press CTRL + C so that the drone disamrs and motors stop 

    def shutdown_hook(self):
        rospy.loginfo("Calling shutdown hook")
        self.disarm()

    # Function to arm the drone 

    def arm(self):
        rospy.loginfo("Calling arm service")
        service_endpoint = "/sentinel_drone/cmd/arming"
        rospy.wait_for_service(service_endpoint, 2.0)
        try:
            arming_service = rospy.ServiceProxy(service_endpoint, CommandBool)
            resp = arming_service(True)
            return resp.success, resp.result
        except rospy.ServiceException as err:
            rospy.logerr(err)

    # Function to disarm the drone 
    def disarm(self):
        rospy.loginfo("Calling disarm service")
        service_endpoint = "/sentinel_drone/cmd/arming"
        rospy.wait_for_service(service_endpoint, 10.0)
        try:
            arming_service = rospy.ServiceProxy(service_endpoint, CommandBool)
            resp = arming_service(False)
            return resp.success, resp.result
        except rospy.ServiceException as err:
            rospy.logerr(err)
        self.is_flying = False


if __name__ == "__main__":

    controller = DroneController()
    controller.arm()
    rospy.sleep(1)

    rospy.loginfo("Entering PID controller loop")
    r = rospy.Rate(28)
    while not rospy.is_shutdown():

        controller.pid()

        if rospy.get_rostime().secs - controller.last_whycon_pose_received_at > 1:
            rospy.logerr("Unable to detect WHYCON poses")
            controller.disarm()
        r.sleep()
        # Add the sleep time to run the controller loop at desired rate
        #rospy.sleep()

