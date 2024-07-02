
import time
import unittest
import rclpy

from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from safety_robile.safety_robile_SMACH  import MonitorBatteryAndCollision, RotateBase, StopBase

class TestStateMachine(unittest.TestCase):

    def setUp(self):
        rclpy.init()

    def tearDown(self):
        rclpy.shutdown()

    def test_monitor_battery_and_collision_low_battery(self):
        node = rclpy.create_node('test_node')
        state = MonitorBatteryAndCollision(node, battery_threshold=50, collision_threshold_distance=1, timeout=5)

        low_battery_msg = String()
        low_battery_msg.data = '20.0'  # Battery level is 20%
        state.battery_callback(low_battery_msg)

        # Run the state
        outcome = state.execute(None)
        
        self.assertEqual(outcome, 'low_battery_level')

    def test_monitor_battery_and_collision_possible_collision(self):
        node = rclpy.create_node('test_node')
        state = MonitorBatteryAndCollision(node, battery_threshold=50, collision_threshold_distance=1, timeout=5)

        collision_msg = LaserScan()
        collision_msg.ranges = [1.4, 1.6, 1.7]
        state.laser_scan_callback(collision_msg)

        outcome = state.execute(None)

        self.assertEqual(outcome, 'possible_collision')

    def test_rotate_base(self):
        node = rclpy.create_node('test_node')
        state = RotateBase(node)

        # Run the state
        outcome = state.execute(None)

        self.assertEqual(outcome, 'succeeded')

    def test_stop_base(self):
        node = rclpy.create_node('test_node')
        state = StopBase(node)

        # Run the state
        outcome = state.execute(None)

        self.assertEqual(outcome, 'succeeded')

if __name__ == '__main__':
    unittest.main()
