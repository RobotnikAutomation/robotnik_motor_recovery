#!/usr/bin/env python
# -*- coding: utf-8 -*-

from rcomponent.rcomponent import *

# Insert here general imports:
import math

# Insert here msg and srv imports:
from std_msgs.msg import String

from robotnik_msgs.msg import RobotnikMotorsStatus
from robotnik_msgs.srv import enable_disable, enable_disableRequest, enable_disableResponse

from robotnik_msgs.msg import StringStamped

from std_srvs.srv import Trigger, TriggerResponse

from geometry_msgs.msg import Twist

class MotorRecovery(RComponent):
    """
    Package to enable or disable the motors depending on the status of the base hardware
    """

    def __init__(self):

        RComponent.__init__(self)

    def ros_read_params(self):
        """Gets params from param server"""
        RComponent.ros_read_params(self)

        self.base_hw_topic = rospy.get_param('~base_hw_topic', 'robotnik_base_hw/status')
        self.enable_base_control_service = rospy.get_param('~enable_base_control_service', 'robotnik_base_control/enable')
        self.recovery_cmd_vel_topic = rospy.get_param('~recovery_cmd_vel_topic', 'motor_recovery/cmd_vel')
        self.protection_motor_time = rospy.get_param('~protection_motor_time', 15)
        self.recovery_motor_time = rospy.get_param('~recovery_motor_time', 2)

    def ros_setup(self):
        """Creates and inits ROS components"""

        RComponent.ros_setup(self)

        # Steps of the machine status of this node
        self.status = rospy.Publisher('~status', String, queue_size=10)

        # cmd_vel output
        self.recovery_cmd_vel = rospy.Publisher(self.recovery_cmd_vel_topic, Twist, queue_size=10)

        # Base hardware subscriber
        self.base_hw = rospy.Subscriber(self.base_hw_topic, RobotnikMotorsStatus, self.base_hw_cb)
        RComponent.add_topics_health(self, self.base_hw, topic_id='base_hw', timeout=1.0, required=True)
        
        # Enable base control service client
        self.enable_base_control = rospy.ServiceProxy(self.enable_base_control_service, enable_disable)

        self.recovery_status = 'DIAGNOSTIC_MOTORS'
        
        self.diagnostic_step = 0
        self.protection_step = 0
        self.recovery_step = 0

        self.last_motor_disabled_time =  rospy.Time.now()
        self.last_recovery_time =  rospy.Time.now()

        return 0

    def init_state(self):

        return RComponent.init_state(self)

    def ready_state(self):
        """Actions performed in ready state"""

        # Check topic health

        if(self.check_topics_health() == False):
            self.switch_to_state(State.EMERGENCY_STATE)
            return RComponent.ready_state(self)

        self.status_msg = String()
        self.status_msg.data = self.recovery_status     
        self.status.publish(self.status_msg)

        self.run()

        return RComponent.ready_state(self)

    def emergency_state(self):
        if(self.check_topics_health() == True):
            self.switch_to_state(State.READY_STATE)

    def shutdown(self):
        """Shutdowns device

        Return:
            0 : if it's performed successfully
            -1: if there's any problem or the component is running
        """

        return RComponent.shutdown(self)

    def switch_to_state(self, new_state):
        """Performs the change of state"""

        return RComponent.switch_to_state(self, new_state)

    def base_hw_cb(self, msg):

        self.num_motors = len(msg.motor_status)
        self.motor_status = msg.motor_status 

        self.tick_topics_health('base_hw')

    def enableMotors(self):
        req = enable_disableRequest()
        req.value = True
        self.enable_base_control(req)

    def disableMotors(self):
        req = enable_disableRequest()
        req.value = False
        self.enable_base_control(req)

    def motorDiagnostic(self):

        diagnostic = True

        velocity_following_error_flag = False
        continuous_current_flag = False
        current_limiting_flag = False

        if self.diagnostic_step == 0:

            index = 0
            for motor in self.motor_status:

                motor_flags = motor.activedriveflags

                # if 'ZERO_VELOCITY' in motor_flags:
                #     self.zero_velocitry_flag = True
                #     diagnostic = False

                if 'VELOCITY_FOLLOWING_ERROR' in motor_flags:
                    velocity_following_error_flag = True
                    rospy.logwarn("Detected VELOCITY_FOLLOWING_ERROR on motor [" + str(index) + "]")

                if 'CONTINUOUS_CURRENT' in motor_flags:
                    continuous_current_flag = True
                    rospy.logwarn("Detected CONTINUOUS_CURRENT on motor [" + str(index) + "]")

                if 'CURRENT_LIMITING' in motor_flags:
                    current_limiting_flag = True
                    rospy.logwarn("Detected CURRENT_LIMITING on motor [" + str(index) + "]")

                index = index + 1
            
            self.diagnostic_step = 1

        if self.diagnostic_step == 1:

            if velocity_following_error_flag == True and  current_limiting_flag == True:
                diagnostic = False
            
            self.diagnostic_step = 0


        return diagnostic

    def motorProtection(self):
        
        motors_protected = False

        if self.protection_step == 0:

            rospy.logwarn("Disabling motors...")
            self.disableMotors()
            rospy.logwarn("Motors disabled")
            self.last_motor_disabled_time =  rospy.Time.now()
            rospy.logwarn("Stop for " + str(self.protection_motor_time) + " seconds")
            self.protection_step = 1

        if self.protection_step == 1:

            if  rospy.Time.now() - self.last_motor_disabled_time > rospy.Duration().from_sec(self.protection_motor_time):
                rospy.logwarn("Motors are protected")
                rospy.logwarn("Enabling motors...")
                self.enableMotors()
                rospy.logwarn("Motors enabled")
                motors_protected = True
                self.protection_step = 0  

        return motors_protected


    def motorRecovery(self):
        
        motors_recovered = False

        if self.recovery_step == 0:

            rospy.logwarn("Initializing motor recovery")     
            self.last_recovery_time =  rospy.Time.now()
            self.recovery_step = 1
            rospy.logwarn("Moving motors forward for " + str(self.recovery_motor_time) + " seconds")  

        if self.recovery_step  == 1:
            
            twist_command = Twist()
            twist_command.linear.x = 0.3

            # Move forward for self.recovery_motor_time seconds
            if  rospy.Time.now() - self.last_recovery_time <= rospy.Duration().from_sec(self.recovery_motor_time):
                self.recovery_cmd_vel.publish(twist_command)
                motors_recovered = False
                self.recovery_step = 1
            else:
                rospy.logwarn("Motors recovered")
                motors_recovered = True
                self.recovery_step = 0

        return motors_recovered


    def run(self):
        
        # Diagnostic mode
        if self.recovery_status == 'DIAGNOSTIC_MOTORS':
        
            if not self.motorDiagnostic():

                self.recovery_status = 'PROTECT_MOTORS'
    
        # Protection mode
        if self.recovery_status == 'PROTECT_MOTORS':

            if self.motorProtection():

                self.recovery_status = 'RECOVERY_MOTORS'

        # Recovery mode
        if self.recovery_status == 'RECOVERY_MOTORS':

            if self.motorRecovery():

                self.recovery_status = 'DIAGNOSTIC_MOTORS'
