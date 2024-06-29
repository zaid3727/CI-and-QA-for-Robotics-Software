import unittest
import rclpy
import time
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from safety_monitoring_SMACH  import MonitorBatteryAndCollision, RotateBase, StopBase

class TestStateMachine(unittest.TestCase):
    # def __init__(self):
        # self.setUp()

    def setUp(self):
        rclpy.init()

    def tearDown(self):
        rclpy.shutdown()

    def test_monitor_battery_and_collision_low_battery(self):
        node = rclpy.create_node('test_node')
        state = MonitorBatteryAndCollision(node, battery_threshold=90, collision_threshold_distance=1, timeout=5)

        # Simulate low battery by publishing a message
        low_battery_msg = String()
        low_battery_msg.data = '15.0'  # Set a value below the threshold
        state.battery_callback(low_battery_msg)

        # Run the state
        outcome = state.execute(None)
        
        self.assertEqual(outcome, 'low_battery_level')

    def test_monitor_battery_and_collision_possible_collision(self):
        node = rclpy.create_node('test_node')
        state = MonitorBatteryAndCollision(node, battery_threshold=90, collision_threshold_distance=1, timeout=5)

        # Simulate a possible collision by publishing a LaserScan message
        collision_msg = LaserScan()
        collision_msg.ranges = [0.4, 0.6, 0.7]  # Set a value below the threshold
        state.laser_scan_callback(collision_msg)

        # Run the state
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
    # test_obj = TestStateMachine()
    # test_obj.test_monitor_battery_and_collision_low_battery()
