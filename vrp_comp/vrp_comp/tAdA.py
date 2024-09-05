import rclpy
from rclpy.node import Node
import time, math, threading
from enum import IntEnum
from std_msgs.msg import Float64
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Vector3


R_earth = 6378100

class Point:
    def __init__(self, lat, lon):
        self.latitude = lat
        self.longitude = lon

class PID:
    def __init__(self, kp, ki, kd, dt=1.0):
        self.dt = dt
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.err = 0
        self.last_err = self.err
        self.sum_err = self.err
        self.derr = 0
        self.setpoint = 0
        self.min = -1
        self.max = 1

    def in_range(self, out):
        return max(min(self.max, out), self.min)

    def calculate(self, val):
        self.err = self.setpoint - val
        self.sum_err += self.err * self.dt
        self.derr = (self.err - self.last_err) / self.dt

        self.last_err = self.err

        out = self.kp * self.err + self.ki * self.sum_err + self.kd * self.derr
        return self.in_range(out)


class GeoNode(Node):
    cur_target_idx = 0
    route = (Point(55.12351516, 36.588970), Point(55.123721, 36.588919), Point(55.123609, 36.5890145))

    course_power = 1

    kp = 2
    ki = 0.0
    kd = 0.0
    dt = 0.1

    coeff_mpower = 20.0
    left_motor_kp = 0.85
    left_power = 0.0
    
    right_power = 0.0
    back_power = 0.0
    
    # coords
    latitude = 0
    longitude = 0
    speed = 0
    yaw = 0
    
    def __init__(self, name="run_geopoints"):
        super().__init__(name)

        self.steering = PID(self.kp, self.ki, self.kd, self.dt)
        
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
            time.sleep(self.dt)
    
    def distance(self):
        dlat = abs(self.latitude - self.route[self.cur_target_idx].latitude)
        dlon = abs(self.longitude - self.route[self.cur_target_idx].longitude)
        a = math.sin(dlat / 2) ** 2 + math.cos(self.latitude) * math.cos(self.route[self.cur_target_idx].latitude) * math.sin(dlon / 2) ** 2
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
        return R_earth * c
    
    def send_thrust(self):
        left_data = Float64()
        left_data.data = self.left_power * self.coeff_mpower
        right_data = Float64()
        right_data.data = self.right_power * self.coeff_mpower
        back_data = Float64()
        back_data.data = self.back_power * self.coeff_mpower

        self.right_pub.publish(right_data)
        self.left_pub.publish(left_data)
        self.back_pub.publish(back_data)

    def task(self):
        dist = self.distance()

        self.yaw = math.degrees(self.yaw)
        print(f"dist = {dist}")
        if dist > 20:
            delta_lat = math.radians(self.route[self.cur_target_idx].latitude - self.latitude)
            delta_lon = math.radians(self.route[self.cur_target_idx].longitude - self.longitude)
            lat_avg = math.radians((self.latitude + self.route[self.cur_target_idx].latitude) / 2)

            delta_y = delta_lat
            delta_x = delta_lon * math.cos(lat_avg)
            target_angle = math.degrees(math.atan2(delta_x, delta_y))
            
            if target_angle > 180: target_angle -= 360
            if self.yaw > 180: self.yaw -= 360
            delta_angle = target_angle - self.yaw
            if delta_angle > 180:
                delta_angle = delta_angle - 360
            elif delta_angle < -180:
                delta_angle = delta_angle + 360
            
            print("target_angle:", target_angle, "\tyaw:", self.yaw, "delta_angle", delta_angle)
            steering_power = self.steering.calculate(delta_angle)
            
            print("steering_power", steering_power)
            self.left_power = -self.course_power
            self.right_power = self.course_power
            self.back_power = steering_power
        else:
            self.cur_target_idx += 1

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
