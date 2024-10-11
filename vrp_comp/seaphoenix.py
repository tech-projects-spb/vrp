import rclpy
from rclpy.node import Node
import time
import math
import threading
from std_msgs.msg import Float64
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Vector3
import sys

class GeoNode(Node):
    coeff = 20
    left_power = 0.0
    right_power = 0.0   
    back_power = 0.0
    period = 0.1
    R_earth = 6371e3  

    latitude = 0
    longitude = 0
    speed = 0
    yaw = 0
    
    cog = 0
    hdg = 0
    target_index = 0
    latitudes = []
    longitudes = []

    def __init__(self, name="run_geopoints"):
        super().__init__(name)
        self.declare_parameter('longitudes', [55.12351516, 55.123616,  55.123721, 55.123714, 55.123609, 55.12350966])
        self.declare_parameter('latitudes', [36.588970, 36.588933,  36.588919, 36.588967,  36.5890145, 36.5890666])
        
        # Получение координат из параметров
        self.longitudes = self.get_parameter('longitudes').get_parameter_value().double_array_value
        self.latitudes = self.get_parameter('latitudes').get_parameter_value().double_array_value

        self.right_pub = self.create_publisher(Float64, '/booblik/thrusters/right/thrust', 10)
        self.left_pub = self.create_publisher(Float64, '/booblik/thrusters/left/thrust', 10)
        self.back_pub = self.create_publisher(Float64, '/booblik/thrusters/back/thrust', 10)

        self.nav_sub = self.create_subscription(NavSatFix, '/booblik/sensors/gps/navsat/fix', self.nav_callback, 10)
        self.euler_sub = self.create_subscription(Vector3, '/booblik/sensor/Imu/Imu/euler', self.euler_callback, 10)
        self.ground_speed_sub = self.create_subscription(Odometry, '/booblik/sensor/position/ground_truth_odometry', self.ground_speed_callback, 10)
        
        self.start()

    def nav_callback(self, msg: NavSatFix): 
        self.latitude = msg.latitude
        self.longitude = msg.longitude

    def euler_callback(self, msg: Vector3):
        self.yaw = msg.y

    def ground_speed_callback(self, msg: Odometry):
        self.speed = msg.twist.twist.linear.x

    def task_loop(self):
        while rclpy.ok():
            if self.target_index < len(self.latitudes):
                self.task(self.latitude, self.longitude, self.cog, self.hdg, self.speed)  
                self.send_thrust()
            time.sleep(self.period)
    
    def send_thrust(self):
        print(self.coeff, self.right_power, self.left_power, self.back_power)
        left_data = Float64()
        left_data.data = self.left_power * self.coeff
        right_data = Float64()
        right_data.data = self.right_power * self.coeff
        back_data = Float64()
        back_data.data = self.back_power * self.coeff

        self.right_pub.publish(right_data)
        self.left_pub.publish(left_data)
        self.back_pub.publish(back_data)
        print(right_data.data, left_data.data, back_data.data)

    def task(self, latitude, longitude, cog, hdg, speed):
        if self.target_index >= len(self.latitudes):
            return

        target_latitude = self.latitudes[self.target_index]
        target_longitude = self.longitudes[self.target_index]

        delta_lat = math.radians(target_latitude - latitude)
        delta_lon = math.radians(target_longitude - longitude)
        lat_avg = math.radians((latitude + target_latitude) / 2)

        delta_y = delta_lat 
        delta_x = delta_lon * math.cos(lat_avg)
        
        target_cog = math.degrees(math.atan2(delta_x, delta_y))
        delta_angle = target_cog - self.cog 
        
        if delta_angle > 180:
            delta_angle -= 360
        elif delta_angle < -180:
            delta_angle += 360

        forward_speed = 1.0
        turning_speed = 1.0  
        threshold = 5 

        if delta_angle > threshold:
            self.left_power = forward_speed
            self.right_power = -turning_speed  
        elif delta_angle < -threshold:
            self.right_power = forward_speed
            self.left_power = -turning_speed  
        else:
            self.left_power = forward_speed 
            self.right_power = forward_speed

        self.back_power = 0.0  


        if math.sqrt(delta_x**2 + delta_y**2) * self.R_earth < 1.0:
            self.target_index += 1  
    def start(self):
        self.taskThread = threading.Thread(target=self.task_loop, daemon=True)
        self.taskThread.start()

def main(args=None):
    rclpy.init(args=args)
    node = GeoNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()