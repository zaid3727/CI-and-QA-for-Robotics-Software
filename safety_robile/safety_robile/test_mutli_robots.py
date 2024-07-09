import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32
import sys

class RobotController(Node):
    def __init__(self, robot_id):
        super().__init__(f'robot_controller_{robot_id}')
        self.robot_id = robot_id
        self.is_leader = (robot_id == 1)
        
        if self.is_leader:
            self.velocity_publish = self.create_publisher(Twist, '/robot1/new_cmd_vel', 10)

            self.velocity_subscribe = self.create_subscription(Twist, '/cmd_vel', self.leader_cmd_vel_callback, 10)


        else:
            self.pre_exist_cmd_vel_topic = self.declare_parameter('cmd_vel_topic', f'/robot{robot_id}/cmd_vel').value

            self.init_velocity_publish = self.create_publisher(Twist, self.pre_exist_cmd_vel_topic, 10)

            self.velocity_subscribe = self.create_subscription(Twist, '/robot1/new_cmd_vel', self.follower_cmd_vel_callback, 10)
        
        self.battery_subscribe = self.create_subscription(Float32, f'/robot{robot_id}/battery_voltage', self.battery_callback, 10)
        self.laser_subscribe = self.create_subscription(LaserScan, f'/robot{robot_id}/scan', self.laser_callback, 10)

        self.battery_level = 50.0
        self.obstacle_detected = False
        self.safe_range = 0.3

    def leader_cmd_vel_callback(self, msg):
        self.velocity_publish.publish(msg)
        self.get_logger().info(f'leading robile info: cmd_vel: linear={msg.linear.x}, angular={msg.angular.z}')

    def follower_cmd_vel_callback(self, msg):
        if not self.obstacle_detected and self.battery_level > 20.0:
            self.cmd_vel_pub.publish(msg)
            self.get_logger().info(f'following robile {self.robot_id} info: cmd_vel: linear={msg.linear.x}, angular={msg.angular.z}')
        elif self.obstacle_detected:
            self.get_logger().warn(f'Obstacle detected in following robile {self.robot_id}, cmd_vel not published')
        elif self.battery_level <= 20.0:
            self.get_logger().warn(f'Battery low in following robile {self.robot_id}, cmd_vel not published')

    def battery_callback(self, msg):
        self.battery_level = msg.data
        if self.battery_level <= 20.0:
            self.get_logger().warn(f'Battery low in following robile {self.robot_id}')

    def laser_callback(self, msg):
        self.obstacle_detected = any(distance < self.safe_range for distance in msg.ranges)
        if self.obstacle_detected:
            self.get_logger().warn(f'Obstacle detected in following robile {self.robot_id}')
            self.stop_robot()

    def stop_robot(self):
        stop_msg = Twist()
        if self.is_leader:
            self.velocity_publish.publish(stop_msg)
        else:
            self.init_velocity_publish.publish(stop_msg)
        self.get_logger().info(f'Stopping robot {self.robot_id}')

def main(args=None):
    print('Starting robot controller...')
    rclpy.init(args=args)
    
    robot_id_param_node = rclpy.create_node('robot_id_param_node')
    robot_id = robot_id_param_node.declare_parameter('robot_id', 1).value
    robot_id_param_node.destroy_node()
    
    print(f'Robot ID: {robot_id}')
    
    robot_controller = RobotController(robot_id)

    try:
        rclpy.spin(robot_controller)
    except KeyboardInterrupt:
        pass
    finally:
        robot_controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
