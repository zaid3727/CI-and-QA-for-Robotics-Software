#!/usr/bin/env python3

import rclpy
import smach

from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

import time

class MonitorBatteryAndCollision(smach.State):
    """
    Class to monitor battery level and possible collision
    Args:
        node: ROS node
        battery_threshold (float): threshold for battery level
        collision_threshold_distance (float): threshold for distance to obstacle
        timeout (float): timeout in seconds
    """
    def __init__(self, node, battery_threshold, collision_threshold_distance, timeout=5):
        smach.State.__init__(self, outcomes=['low_battery_level', 'possible_collision', 'timeout'])
        self.timeout = timeout
        self.battery_threshold = battery_threshold
        self.collision_threshold_distance = collision_threshold_distance
        self.battery_subscriber = node.create_subscription(String, '/battery_voltage', self.battery_callback, 10)
        self.laser_scan_subscriber = node.create_subscription(LaserScan, '/scan', self.laser_scan_callback, 10)
        time.sleep(0.5)
        self.start_time = time.time()
        self.node = node
        self.logger = self.node.get_logger()
        self.battery_low = False
        self.collision_possible = False

    def battery_callback(self, msg):
        if float(msg.data) < self.battery_threshold:
            self.logger.warning('Battery level is low')
            self.battery_low = True
        else:
            self.battery_low = False

    def laser_scan_callback(self, msg):
        if min(msg.ranges) < self.collision_threshold_distance:
            self.logger.warning('Battery level is low')
            self.collision_possible = True
        else:
            self.collision_possible = False

    def execute(self, ud):
        self.logger.info('Executing state MONITOR_BATTERY_AND_COLLISION')
        while (time.time() - self.start_time) < self.timeout:
            time.sleep(0.1)
            rclpy.spin_once(self.node)

            # priority: collision > battery
            if self.collision_possible:
                return 'possible_collision'
            elif self.battery_low:
                return 'low_battery_level'
            
        return 'timeout' 
    
class RotateBase(smach.State):
    """
    Class to rotate the robot base
    Args:
        node: ROS node
    """
    def __init__(self, node):
        smach.State.__init__(self, outcomes=['succeeded'])
        self.velocity_publisher = node.create_publisher(Twist, '/cmd_vel', 10)
        time.sleep(0.2)
        self.logger = node.get_logger()

    def execute(self, ud):
        self.logger.info('Executing state ROTATE_BASE')
        msg = Twist()
        msg.angular.z = 0.5
        self.velocity_publisher.publish(msg)

        return 'succeeded'    

class StopBase(smach.State):
    """
    Class to stop the robot base
    Args:
        node: ROS node
    """
    def __init__(self, node):
        smach.State.__init__(self, outcomes=['succeeded'])
        self.velocity_publisher = node.create_publisher(Twist, '/cmd_vel', 10)
        time.sleep(0.2)
        self.logger = node.get_logger()

    def execute(self, ud):
        self.logger.info('Executing state STOP_BASE')
        msg = Twist()
        msg.linear.x = 0.0
        msg.linear.y = 0.0
        msg.angular.z = 0.0
        self.velocity_publisher.publish(msg)

        return 'succeeded'

def main(args=None):
    """
    main function to execute the state machine
    """
    rclpy.init(args=args)
    node = rclpy.create_node('safety_features_node')

    sm = smach.StateMachine(['overall_success'])
    battery_threshold = 20
    collision_threshold_distance = 0.3
    timeout = 1000

    with sm:
        smach.StateMachine.add('MONITOR_BATTERY_AND_COLLISION', 
                                MonitorBatteryAndCollision(node, battery_threshold, collision_threshold_distance, timeout),
                                transitions={'low_battery_level':'ROTATE_BASE', 'possible_collision':'STOP_BASE', 'timeout':'overall_success'})
        
        smach.StateMachine.add('ROTATE_BASE',
                                RotateBase(node),
                                transitions={'succeeded':'MONITOR_BATTERY_AND_COLLISION'})
        
        smach.StateMachine.add('STOP_BASE',
                                StopBase(node),
                                transitions={'succeeded':'MONITOR_BATTERY_AND_COLLISION'})

    outcome = sm.execute()

    if outcome == 'overall_success':
        node.get_logger().info('State machine executed successfully')

    rclpy.shutdown()

if __name__ == "__main__":
    main()
