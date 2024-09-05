import rclpy
from rclpy.node import Node
import time, math, threading
from enum import IntEnum
from std_msgs.msg import Float64
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Vector3


class GeoNode(Node):
    # power
    coeff = 0
    left_power = 0.0
    right_power = 0.0
    back_power = 0.0
    period = 0.1
    
    # coords
    latitude = 0
    longitude = 0
    speed = 0
    yaw = 0
    

    def __init__(self, name="run_geopoints"):
        super().__init__(name)
        
        self.right_pub = self.create_publisher(
            Float64,
            '/booblik/thrusters/right/thrust',
            10)
        
        self.left_pub = self.create_publisher(
            Float64,
            '/booblik/thrusters/left/thrust',
            10)
        self.back_pub = self.create_publisher(
            Float64,
            '/booblik/thrusters/back/thrust',
            10)
        
        self.nav_sub = self.create_subscription(
            NavSatFix,
            '/booblik/sensors/gps/navsat/fix',
            self.nav_callback,
            10
        )

        self.euler_sub = self.create_subscription(
            Vector3,
            '/booblik/sensors/imu/imu/euler',
            self.euler_callback,
            10
        )

        self.ground_speed_sub = self.create_subscription(
            Odometry,
            '/booblik/sensors/position/ground_truth_odometry',
            self.ground_speed_callback,
            10
        )
        
        self.start()
        

    def nav_callback(self, msg: NavSatFix): 
        self.latitude = msg.latitude
        self.longitude = msg.longitude

    def euler_callback(self, msg: Vector3):
        self.yaw = msg.y

    def ground_speed_callback(self, msg:Odometry):
        self.speed = msg.twist.twist.linear.x

    def task_loop(self):
        while True:
            self.task()
            self.send_thrust()
            time.sleep(self.period)
    
    def send_thrust(self):
        left_data = Float64()
        left_data.data = self.left_power * self.coeff
        right_data = Float64()
        right_data.data = self.right_power * self.coeff
        back_data = Float64()
        back_data.data = self.back_power * self.coeff

        self.right_pub.publish(right_data)
        self.left_pub.publish(left_data)
        self.back_pub.publish(back_data)

    def task(self):
        print(self.latitude, self.longitude, self.yaw, self.speed)


    def start(self):
        self.taskThread = threading.Thread(
            target=self.task_loop, daemon=True).start()

def main(args=None):
    rclpy.init(args=args)
    node = GeoNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
