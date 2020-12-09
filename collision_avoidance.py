
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


class CollisionAvoidance(Node):

    def __init__(self):
        super().__init__('collision_avoidance')

        self.linear_velocity = [0.2, -0.2, 0.0, 0.0, 0.0]  # unit: m/s
        self.angular_velocity = [0.0, 0.5, 0.5, -0.5, 0.0]  # unit: m/s
        self.obstacle = [True, True, True, True] # Values for Front, Back, Right, Left. True if obstacle present False if obstacle is not available.
        self.safety_distance = 0.3  # unit: m
        self.scan_ranges = []
        self.init_scan_state = False  # To get the initial scan data at the beginning

        qos = QoSProfile(depth=10)
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', qos)
        self.scan_sub = self.create_subscription(LaserScan, 'scan', self.scan_callback, qos_profile=qos_profile_sensor_data)
        self.cmd_vel_raw_sub = self.create_subscription(Twist, 'cmd_vel_raw', self.cmd_vel_raw_callback, qos)
        self.update_timer = self.create_timer(0.010, self.update_callback)
        self.get_logger().info("Collision Avoidance Node is Initialized!.")

    def scan_callback(self, msg):
        self.scan_ranges = msg.ranges
        self.front = self.scan_ranges[315:359] + self.scan_ranges[0:45]
        while 0.0 in self.front:
            self.front.remove(0.0)
        self.right = self.scan_ranges[45:135]
        while 0.0 in self.right:
            self.right.remove(0.0)
        self.back = self.scan_ranges[135:225]
        while 0.0 in self.back:
            self.back.remove(0.0)
            #print(self.back)
        self.left = self.scan_ranges[225:315]
        while 0.0 in self.left:
            self.left.remove(0.0)

        self.obstacle = [(min(self.front) < self.safety_distance),
                (min(self.back) < self.safety_distance),
                (min(self.right) < self.safety_distance),
                (min(self.left) < self.safety_distance)]

        self.init_scan_state = True

    def cmd_vel_raw_callback(self, msg):
        self.linear_velocity = msg.linear.x
        self.angular_velocity = msg.angular.z

    def update_callback(self):
        if self.init_scan_state is True:
            self.detect_obstacle()

    def detect_obstacle(self):
        twist = Twist()
        indx = 4
        if self.obstacle[0] is False:
            indx = 0
        elif self.obstacle[1] is False:
            indx = 1
        elif self.obstacle[2] is False:
            indx = 2
        elif self.obstacle[3] is False:
            indx = 3
        else:
            self.get_logger().info("Surrounded by obstacles. Robot stopped.")

        twist.linear.x = self.linear_velocity[indx]
        twist.angular.z = self.angular_velocity[indx]
        
        self.cmd_vel_pub.publish(twist)
