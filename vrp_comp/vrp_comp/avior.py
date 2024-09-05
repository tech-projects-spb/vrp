import rclpy
from rclpy.node import Node
import time, math, threading, sys
from enum import IntEnum
from std_msgs.msg import Float64
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Vector3

R_earth=6378100

class Point:
    def __init__(self, lat, lon):
        self.latitude = lat
        self.longitude = lon
class GeoNode(Node):
    # power
    z=0
    coeff = 15
    left_power = 0.0
    right_power = 0.0
    back_power = 0.0
    period = 0.1
    kp=1.5
    idx=0
    # coords
    latitude = 0
    longitude = 0
    speed = 0
    yaw = 0
    route = ([55.12351516, 36.588970], [55.123721, 36.588919], [55.123609, 36.5890145])
    course_power = 1.0
    

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
      
        self.target=0
        
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
    def distance(self):
        dlat = abs(self.latitude - self.route[self.target][0])
        dlon = abs(self.longitude - self.route[self.target][1])
        a = math.sin(dlat / 2) ** 2 + math.cos(self.latitude) * math.cos(self.route[self.target][0]) * math.sin(dlon / 2) ** 2
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
        return R_earth * c

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
       
    
          
        dist = self.distance()
        self.yaw = math.degrees(self.yaw) - 180
        print(f"dist = {dist}")
        if dist > 50:
            delta_lat = math.radians(self.route[self.target][0]- self.latitude)
            delta_lon = math.radians(self.route[self.target][1]- self.longitude)
            lat_avg = math.radians((self.latitude + self.route[self.target][0]) / 2)
            delta_y = (delta_lat) 
            delta_x = (delta_lon) * math.cos(lat_avg)
            angle= math.degrees(math.atan2(delta_x, delta_y))
            if angle > 180: angle =angle - 360
            if self.yaw > 180: self.yaw = self.yaw - 360
            delta_angle = angle - self.yaw
            if delta_angle > 180:
                delta_angle = delta_angle - 360
            elif delta_angle < -180:
                delta_angle = delta_angle + 360
            
            z = delta_angle*self.kp

            self.left_power = -self.course_power
            self.right_power = self.course_power
            self.back_power = z
        else:
            self.target=self.target+1
        print('Расстояние до точки: {:4.1f}'.format(math.sqrt(delta_x**2 + delta_y**2) * R_earth))

       

       
      

        
        
        
        print('yaw',self.yaw)
        print('Угол математика', angle)
        print(z)
        
       
        
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