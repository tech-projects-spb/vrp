import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from geometry_msgs.msg import Vector3
import logging
import time
import threading
from math import degrees

MAX_THRUST = 10
PERIOD = 10  # Время на проезд прямо (в секундах)
DIFF = 5  # Допустимое отклонение по компасу

logging.basicConfig(
            level=logging.INFO,
            format='%(asctime)s [%(name)s] [%(levelname)-5.5s] %(message)s',
            filename='gps_log.log',
            filemode='a'
        )

class MotorsTest(Node):     
    def __init__(self, name='motors_test', mode='auto'):
        super().__init__(name)
        
        self.mode = mode
        self.left_pub = self.create_publisher(Float64, '/booblik/thrusters/left/thrust', 10)
        self.right_pub = self.create_publisher(Float64, '/booblik/thrusters/right/thrust', 10)
        
        self.yaw_sub = self.create_subscription(
            Vector3,
            '/booblik/sensors/imu/imu/euler',
            self.yaw_callback,
            10
        )

        self.logger = logging.getLogger('Motors test')
        self.yaw = 0
        self.left_coeff = 1.0
        self.right_coeff = 1.0
        self.start_yaw = None
        self.time_straight = 0  # Время, в течение которого робот движется прямо

        if self.mode == 'manual':
            # В ручном режиме публикуем коэффициенты каждую секунду
            self.timer = self.create_timer(1.0, self.publish_thrust)
            # Запускаем поток для ввода данных от пользователя
            input_thread = threading.Thread(target=self.manual_input_loop)
            input_thread.daemon = True  # Поток завершится вместе с программой
            input_thread.start()
        elif self.mode == 'auto':
            # В автоматическом режиме запускаем авто-цикл
            self.timer = self.create_timer(1.0, self.auto_loop)
        else:
            self.logger.error(f"Неизвестный режим: {self.mode}. Используйте 'auto' или 'manual'.")
            rclpy.shutdown()

        if self.mode == 'manual':
            # В ручном режиме запускаем цикл для ввода и публикации коэффициентов
            self.manual_loop()
        elif self.mode == 'auto':
            # В автоматическом режиме запускаем авто-цикл
            self.timer = self.create_timer(1.0, self.auto_loop)
        else:
            self.logger.error(f"Неизвестный режим: {self.mode}. Используйте 'auto' или 'manual'.")
            rclpy.shutdown()

    
    def yaw_callback(self, msg: Vector3):
        self.yaw = degrees(msg.y) 

    def publish_thrust(self, coefficient: float, thrust: str):
        '''Универсальная функция публикации тяги'''
        power = Float64()
        power.data = coefficient * MAX_THRUST

        # Публикация по строке 'thrust'
        publisher = getattr(self, f'{thrust}_pub', None)
        
        if publisher:
            publisher.publish(power)
        else:
            self.logger.info(f'Неизвестный двигатель: {thrust}') 

    def manual_loop(self):
        '''Цикл для ручного режима, запрашивающий ввод и публикующий коэффициенты'''
        while True:
            # Запрос у пользователя коэффициентов для двигателей
            user_input = input('Введите коэффициенты для левого и правого двигателя (через пробел): ')
            try:
                # Обновляем коэффициенты для двигателей
                self.left_coeff, self.right_coeff = [float(i)/100 for i in user_input.split()]
                self.logger.info(f'Текущие коэффициенты тяги: Left = {self.left_coeff}, Right = {self.right_coeff}, при Yaw {self.yaw}')
                
                # Публикуем значения
                self.publish_thrust(self.left_coeff, 'left')
                self.publish_thrust(self.right_coeff, 'right')

            except ValueError:
                self.logger.error("Неверный формат ввода! Введите два числа от 1 до 100, разделенные пробелом.")

    def auto_loop(self):
        if self.start_yaw is None:
            self.start_yaw = self.yaw # Инициализация начального направления
        
        # Публикация текущих коэффициентов
        self.publish_thrust(self.left_coeff, 'left')
        self.publish_thrust(self.right_coeff, 'right')

        # Проверка отклонения yaw и корректировка коэффициентов
        yaw_diff = self.yaw - self.start_yaw

        if abs(yaw_diff) > DIFF:
            if yaw_diff > DIFF:
                self.left_coeff -= 0.02
                self.logger.info(f'Уход вправо: уменьшение тяги левого двигателя до {self.left_coeff}')
            else:
                self.right_coeff -= 0.02
                self.logger.info(f'Уход влево: уменьшение тяги правого двигателя до {self.left_coeff}')
            self.time_straight = 0 
        else:
            self.time_straight += 1
            self.logger.info(f'Робот движется прямо при отклонении угла {yaw_diff}')

        # Проверка длительности движения
        if abs(time.time() - self.get_clock().now().seconds_nanoseconds()[0]) > PERIOD:
            self.logger.info(f'Бублик успешно проехал 10 секунд. Завершаем тесты. Итоговые коэффициенты тяги: Left = {self.left_coeff}, Right = {self.right_coeff}')
            rclpy.shutdown()
              

def main(args=None):
    rclpy.init(args=args)
    # Получаем параметр запуска (auto или manual) из командной строки
    mode = input("Выберите режим работы ('auto' или 'manual'): ").strip().lower()
    task = MotorsTest(mode=mode)
    rclpy.spin(task)
    rclpy.shutdown


if __name__ == '__main__':
    main()

    




