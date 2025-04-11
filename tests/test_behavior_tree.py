import unittest
import rclpy
import py_trees as pt
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
from sensor_msgs.msg import LaserScan
from safety_robile.safety_robile.behaviors import rotate, stop_motion, battery_status2bb, laser_scan_2bb

class TestBehaviors(unittest.TestCase):

    def setUp(self):
        rclpy.init()
        self.node = rclpy.create_node('test_node')

    def tearDown(self):
        self.node.destroy_node()
        rclpy.shutdown()

    def test_rotate_behavior(self):
        rotate_behavior = rotate(topic_name='/cmd_vel', ang_vel=1.0)
        rotate_behavior.setup(node=self.node)
        self.assertIsInstance(rotate_behavior, rotate)

    def test_stop_motion_behavior(self):
        stop_behavior = stop_motion(topic_name='/cmd_vel')
        stop_behavior.setup(node=self.node)
        self.assertIsInstance(stop_behavior, stop_motion)

    def test_battery_status2bb(self):
        battery_behavior = battery_status2bb(topic_name='/battery_voltage', threshold=20.0)
        battery_behavior.setup(node=self.node)
        bb = pt.blackboard.Blackboard()
        bb.set("battery_low_warning", False)

        # Simulate low battery voltage message
        battery_msg = Float32()
        battery_msg.data = 19.0
        self.node.get_logger().info("Simulating low battery voltage message")
        
        # Publish the simulated message
        battery_publisher = self.node.create_publisher(Float32, '/battery_voltage', 10)
        battery_publisher.publish(battery_msg)

        # Check if the battery_low_warning is set
        def check_low_battery_warning():
            self.assertTrue(bb.get('battery_low_warning'))
            self.assertEqual(bb.get('battery'), 19.0)

        self.node.create_timer(1.0, check_low_battery_warning)
        rclpy.spin_once(self.node, timeout_sec=2.0)

        # Simulate normal battery voltage message
        battery_msg.data = 21.0
        self.node.get_logger().info("Simulating normal battery voltage message")
        battery_publisher.publish(battery_msg)

        # Check if the battery_low_warning is cleared
        def check_normal_battery_warning():
            self.assertFalse(bb.get('battery_low_warning'))
            self.assertEqual(bb.get('battery'), 21.0)

        self.node.create_timer(1.0, check_normal_battery_warning)
        rclpy.spin_once(self.node, timeout_sec=2.0)


    def test_laser_scan_2bb(self):
        laser_behavior = laser_scan_2bb(topic_name='/scan', safe_range=0.5)
        laser_behavior.setup(node=self.node)
        bb = pt.blackboard.Blackboard()
        bb.set("collison_warning", False)

        # Simulate LaserScan message
        scan_msg = LaserScan()
        scan_msg.ranges = [0.4] * 360
        self.node.get_logger().info("Simulating LaserScan message")

        # Simulate publishing a message
        laser_publisher = self.node.create_publisher(LaserScan, '/scan', 10)
        laser_publisher.publish(scan_msg)

        def check_msg_received():
            self.assertTrue(bb.get('collison_warning'))

        self.node.create_timer(1.0, check_msg_received)
        rclpy.spin_once(self.node, timeout_sec=2.0)

if __name__ == '__main__':
    unittest.main()
