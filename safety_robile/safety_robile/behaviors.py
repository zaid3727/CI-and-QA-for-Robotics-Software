#!/usr/bin/env python3

import rclpy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import py_trees as pt
import py_trees_ros as ptr

class rotate(pt.behaviour.Behaviour):

    """
    Rotates the robot about z-axis 
    """

    def __init__(self, name="rotate platform", topic_name="/cmd_vel", ang_vel=1.0):
        """
        One time setup for the behavior which takes less time to execute, and inherited from the parent class
        """
        # inherit all the class variables from the parent class and make it a behavior
        super(rotate, self).__init__(name)

        # Set up topic name to publish rotation commands
        self.topic_name = topic_name

        # Set up Maximum allowable rotational velocity
        self.ang_vel = ang_vel  # units: rad/sec

        # Execution checker
        self.sent_goal = False

    def setup(self, **kwargs):
        """
        Setting up things which generally might require time to prevent delay in tree initialisation
        """
        self.logger.info("[ROTATE] setting up rotate behavior")
        
        self.node = kwargs['node']

        # Create publisher to publish rotation commands
        self.cmd_vel_pub = self.node.create_publisher(
            msg_type=Twist,
            topic=self.topic_name,
            qos_profile=ptr.utilities.qos_profile_latched()
        )

        return True

    def update(self):
        """
        Primary function of the behavior is implemented in this method

        Rotating the robot at maximum allowed angular velocity
        """
        self.logger.info("[ROTATE] update: updating rotate behavior")
        self.logger.debug("%s.update()" % self.__class__.__name__)

        # Send the rotation command to self.topic_name in this method using the message type Twist()
        
        cmd_vel_msg = Twist()
        cmd_vel_msg.angular.z = self.ang_vel
        self.cmd_vel_pub.publish(cmd_vel_msg)
        
        return pt.common.Status.RUNNING

    def terminate(self, new_status):
        """
        terminate() is trigerred once the execution of the behavior finishes, 
        i.e. when the status changes from RUNNING to SUCCESS or FAILURE
        """
        self.logger.info("[ROTATE] terminate: publishing zero angular velocity")
        twist_msg = Twist()
        twist_msg.linear.x = 0.
        twist_msg.linear.y = 0.
        twist_msg.angular.z = 0.
                    
        self.cmd_vel_pub.publish(twist_msg)
        self.sent_goal = False
        return super().terminate(new_status)

class stop_motion(pt.behaviour.Behaviour):

    """
    Stops the robot when it is controlled using joystick or by cmd_vel command
    """
    def __init__(self, name: str = "stop platform", topic_name: str = "/cmd_vel"):
        """
        One time setup for the behavior which takes less time to execute, and inherited from the parent class
        """        
        super(stop_motion, self).__init__(name)

        # Set up topic name to publish rotation commands
        self.cmd_vel_topic = topic_name

    def setup(self, **kwargs):
        """
        Setting up things which generally might require time to prevent delay in tree initialisation
        """
        self.logger.info("[STOP MOTION] setting up stop motion behavior")

        try:
            self.node = kwargs['node']
        except KeyError as e:
            error_message = f"didn't find 'node' in setup's kwargs [{self.qualified_name}]"
            raise KeyError(error_message) from e  # 'direct cause' traceability

        # Create publisher to publish rotation commands
        self.cmd_vel_pub = self.node.create_publisher(
            msg_type=Twist,
            topic=self.cmd_vel_topic,
            qos_profile=ptr.utilities.qos_profile_latched()
        )
        
        return True

    def update(self):
        """
        Primary function of the behavior is implemented in this method

        Stopping the robot by sending zero velocity command to self.cmd_vel_topic
        """

        self.logger.info("[STOP] update: updating stop behavior")
        self.logger.debug("%s.update()" % self.__class__.__name__)
        
        """
        Send the zero rotation command to self.cmd_vel_topic
        """
        
        cmd_vel_msg = Twist()
        cmd_vel_msg.linear.x = 0.0
        cmd_vel_msg.linear.y = 0.0
        cmd_vel_msg.angular.z = 0.0
        self.cmd_vel_pub.publish(cmd_vel_msg)      

        return pt.common.Status.SUCCESS


    def terminate(self, new_status):
        """
        terminate() is trigerred once the execution of the behavior finishes, 
        i.e. when the status changes from RUNNING to SUCCESS or FAILURE
        """
        self.logger.info("[ROTATE] terminate: publishing zero angular velocity")
        twist_msg = Twist()
        twist_msg.linear.x = 0.
        twist_msg.linear.y = 0.
        twist_msg.angular.z = 0.
                    
        self.cmd_vel_pub.publish(twist_msg)
        self.sent_goal = False
        return super().terminate(new_status)

class battery_status2bb(ptr.subscribers.ToBlackboard):

    """
    Checking battery status
    """
    def __init__(self, topic_name: str = "/battery_voltage", name: str = 'Battery2BB', threshold: float = 30.0):
        """
        One time setup for the behavior which takes less time to execute, and inherited from the parent class
        """    
        super().__init__(name=name,
                        topic_name=topic_name,
                        topic_type=Float32,
                        blackboard_variables={'battery': 'data'},
                        initialise_variables={'battery': 100.0},
                        clearing_policy=pt.common.ClearingPolicy.NEVER,  # to decide when data should be cleared/reset.
                        qos_profile=ptr.utilities.qos_profile_unlatched()
                        )
        self.blackboard.register_key(
            key='battery_low_warning',
            access=pt.common.Access.WRITE
        ) 
        self.blackboard.battery_low_warning = False   # decision making
        self.threshold = threshold

    def update(self):
        """
        Primary function of the behavior is implemented in this method

        Call the parent to write the raw data to the blackboard and then check against the
        threshold to determine if the low warning flag should also be updated.
        """
        self.logger.info('[BATTERY] update: running battery_status2bb update')
        self.logger.debug("%s.update()" % self.__class__.__name__)
        status = super(battery_status2bb, self).update()
        
        """
        check battery voltage level stored in self.blackboard.battery. By comparing with threshold value, update the value of 
        self.blackboad.battery_low_warning
        """
        if self.blackboard.exists('battery'):
            if self.blackboard.battery < self.threshold:
                self.blackboard.battery_low_warning = True 
            else:
                self.blackboard.battery_low_warning = False
            self.feedback_message = "Battery level is low" if self.blackboard.battery_low_warning else "Battery level is ok"        

        return pt.common.Status.SUCCESS

class laser_scan_2bb(ptr.subscribers.ToBlackboard):

    """
    Checking laser_scan to avoid possible collison
    """
    def __init__(self, topic_name: str = "/scan", name: str = 'Scan2BB', safe_range: float = 0.25):
        """
        One time setup for the behavior which takes less time to execute, and inherited from the parent class
        """
        super().__init__(name=name,
                        topic_name=topic_name,
                        topic_type=LaserScan,
                        blackboard_variables={'laser_scan':'ranges'},
                        clearing_policy=pt.common.ClearingPolicy.NEVER,  # to decide when data should be cleared/reset.
                        qos_profile=QoSProfile(
                                    reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
                                    history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
                                    depth=10
                                )
                        )
        self.blackboard.register_key(
            key='collison_warning',
            access=pt.common.Access.WRITE
        )
        self.blackboard.register_key(
            key='point_at_min_dist',
            access=pt.common.Access.WRITE
        )
        self.blackboard.collison_warning = False   # decision making
        self.safe_min_range = safe_range
        self.blackboard.point_at_min_dist = 0.0

    def update(self):
        """
        Primary function of the behavior is implemented in this method

        Call the parent to write the raw data to the blackboard and then check against the
        threshold to set the warning if the robot is close to any obstacle.
        """
        self.logger.info("[LASER SCAN] update: running laser_scan_2bb update")
        self.logger.debug("%s.update()" % self.__class__.__name__)
        status = super(laser_scan_2bb, self).update()

        """
        Based on the closeness of laser scan points (any of them) to robot, update the value of self.blackboard.collison_warning.
        Assign the minimum value of laser scan to self.blackboard.point_at_min_dist. 
        """

        if self.blackboard.exists('laser_scan'):
            if min(self.blackboard.laser_scan) < self.safe_min_range:
                self.blackboard.collison_warning = True
                self.blackboard.point_at_min_dist = min(self.blackboard.laser_scan)
            else:
                self.blackboard.collison_warning = False
        else:
            return pt.common.Status.RUNNING

        return pt.common.Status.SUCCESS
