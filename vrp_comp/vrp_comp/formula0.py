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
    coeff = 20.0
    left_power = 0.0
    right_power = 0.0
    back_power = 0.0
    period = 0.1
    
    # coords
    latitude = 0
    longitude = 0
    speed = 0
    yaw = 0
    R_earth=6378,1
    
    num_point=0 #счетчик для точек

    threshold = 50 #пороговый угол для P регулятора
    threshold_slow = 40 #пороговый угол для замедления
    target_angle = 180 
    trust=0.2

    rot_data=[0,0,0,0,0,0,0,0,0,0]

    coordinates=[[55.12351516, 36.588970], [55.123721, 36.588919], [55.123609, 36.5890145]]
    

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
        #delta
        delta_lat = math.radians(self.coordinates[self.num_point][0] - self.latitude)
        delta_lon = math.radians(self.coordinates[self.num_point][1] - self.longitude)
        lat_avg = math.radians((self.latitude + self.coordinates[self.num_point][0]) / 2)
        delta_y = (delta_lat)
        delta_x = (delta_lon) * math.cos(lat_avg)
        self.delta_angle = math.degrees(math.atan2(delta_x, delta_y))-math.degrees(self.yaw)
        if self.delta_angle > 180:
            print(-360)
            self.delta_angle = self.delta_angle - 360
        elif self.delta_angle < -180:
            print(+360)
            self.delta_angle = self.delta_angle + 360


        
        
        if self.num_point>len(self.coordinates)-1: #завершение
            self.left_power = 0
            self.right_power = 0
            self.back_power = 0
            print('end')
        elif (math.sqrt(delta_x**2 + delta_y**2)*6378100) < 1.0: #в метре от буйка
            self.num_point+=1

        if self.delta_angle < self.threshold and self.delta_angle > -self.threshold:
            self.left_power = 1.0
            self.right_power = -1.0
            self.back_power = self.delta_angle/90
        
        elif self.delta_angle>0:
            self.left_power = self.trust
            self.right_power = self.trust
            self.back_power = self.trust
            print('right')

        elif self.delta_angle<0:
            self.left_power = -self.trust
            self.right_power = -self.trust
            self.back_power = -self.trust
            print('left')


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