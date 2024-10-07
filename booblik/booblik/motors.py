import troykahat  # Используем библиотеку TroykaHat для работы с аппаратным обеспечением
import time
from dataclasses import dataclass
from threading import Thread
import rclpy  # Импортируем библиотеку ROS2 для Python
from rclpy.node import Node
from std_msgs.msg import Float64
from booblik.utils import get_directory, load_config
import os
 
PWM_SCALING_FACTOR = 1000
SET_THRUST_SCALING_FACTOR = 20.0 

@dataclass
class MotorConfig:
    """Конфигурация отдельного мотора."""
    pin_ap: int  # Номер пина, к которому подключен мотор
    direction: int  # Направление вращения мотора (1 или -1)
    thrust_coeff: float  # Коэффициент тяги для мотора


@dataclass
class MotorsConfig:
    """Конфигурация всех моторов робота."""
    left: MotorConfig  # Левый мотор
    right: MotorConfig  # Правый мотор
    back: MotorConfig  # Задний мотор

class TroykaMotorDriver:
    """Драйвер для управления моторами через TroykaHat.""" 
    pwm_delta: float = 0.5  # Разница в скорости между стоповым значением PWM и max/min значением
    pwm_stop_value: float = 1.5  # Значение PWM, при котором моторы находятся в состоянии покоя (не вращаются) 

    def __init__(self, config: MotorsConfig, freq: float = 500, block_time: float = 4) -> None:
        self.config = config  # Конфигурация моторов
        self.freq = freq  # Частота PWM в Гц
        self.coeff = self.freq / PWM_SCALING_FACTOR  # Коэффициент масштабирования для PWM
        self.block_time = block_time  # Время блокировки управления

        # Настройка пинов для управления моторами
        self.ap = troykahat.analog_io()
        self.ap._setPwmFreq(self.freq)  # Устанавливаем частоту PWM
        self.ap.pinMode(self.config.left.pin_ap, self.ap.OUTPUT)
        self.ap.pinMode(self.config.right.pin_ap, self.ap.OUTPUT)
        self.ap.pinMode(self.config.back.pin_ap, self.ap.OUTPUT)

        self.initialize_motors()  # Инициализация моторов в нейтральное положение

        self.last_time = time.time()  # Время последней команды управления
        self.alarm = False
        self.thread = Thread(target=self.check_loop, daemon=True)
        self.thread.start()  # Запуск потока для контроля времени блокировки

    def initialize_motors(self):
        """Инициализация моторов в нейтральное положение."""
        p = self.pwm_stop_value * self.coeff
        self.ap.analogWrite(self.config.left.pin_ap, p)
        self.ap.analogWrite(self.config.right.pin_ap, p)
        self.ap.analogWrite(self.config.back.pin_ap, p)
        time.sleep(2)  # Задержка для стабилизации системы

    def set_thrust(self, left: float, right: float, back: float) -> None:
        """Установка тяги для каждого из моторов с учётом направления вращения, коэффициента масштабирования и коэффициентов тяги."""
        self.apply_thrust(left, self.config.left.pin_ap, self.config.left.direction, self.config.left.thrust_coeff)
        self.apply_thrust(right, self.config.right.pin_ap, self.config.right.direction, self.config.right.thrust_coeff)
        self.apply_thrust(back, self.config.back.pin_ap, self.config.back.direction, self.config.back.thrust_coeff)
        
        self.last_time = time.time()  # Обновляем время последней команды

    def apply_thrust(self, value: float, pin: int, direction: int, thrust_coeff: float):
        """Применение тяги к мотору."""
        p = (self.pwm_stop_value + value * self.pwm_delta * direction * thrust_coeff) * self.coeff
        self.ap.analogWrite(pin, p)

    def check_loop(self):
        """Проверка на превышение времени блокировки управления."""
        while rclpy.ok():
            if time.time() - self.last_time > self.block_time and not self.alarm:
                self.alarm = True
                self.initialize_motors()  # Возвращаем моторы в нейтральное положение при блокировке
            else:
                self.alarm = False
            time.sleep(0.1)
    
    def stop(self):
        """Остановка потока для контроля управления."""
        self.thread.join()


class MotorsNode(Node):
    def __init__(self, config, name='motors'):
        super().__init__(name)

        self.left = 0
        self.right = 0
        self.back = 0

        self.config = config
        self.driver = TroykaMotorDriver(MotorsConfig(
            self.create_motor_config('left'),
            self.create_motor_config('right'),
            self.create_motor_config('back')
        ), freq=500)
        
        # Подписка на топики для управления тягой моторов
        self.initialize_subscribers()
    
    def create_motor_config(self, motor_name):
        """ Инициализация драйвера мотора на основе конфигов."""
        motor_data = self.config[motor_name]
        return MotorConfig(motor_data['pin'], motor_data['direction'], motor_data['thrust_coeff'])

    def initialize_subscribers(self):
        """Настройка подписок на топики управления тягой."""
        self.left_subscription = self.create_subscription(
            Float64,
            '/booblik/thrusters/left/thrust',
            self.left_thrust_callback,
            10)
        
        self.right_subscription = self.create_subscription(
            Float64,
            '/booblik/thrusters/right/thrust',
            self.right_thrust_callback,
            10)
        
        self.back_subscription = self.create_subscription(
            Float64,
            '/booblik/thrusters/back/thrust',
            self.back_thrust_callback,
            10)

    # Обработки сообщений от топиков
    def left_thrust_callback(self, data): self.update_thrust('left', data.data)
    def right_thrust_callback(self, data): self.update_thrust('right', data.data)
    def back_thrust_callback(self, data): self.update_thrust('back', data.data) 

    def update_thrust(self, motor: str, value: float):
        """Обновление тяги и отправка значений в драйвер."""
        max_thrust = self.config['max_thrust']
        value = min(max_thrust, max(-max_thrust, value))
        setattr(self, motor, value / SET_THRUST_SCALING_FACTOR)  # Преобразование значения тяги
        self.driver.set_thrust(self.left, self.right, self.back)  # Отправка обновлённых значений тяги в драйвер моторов 
     
    def stop(self):
        """Остановка ноды и драйвера."""
        self.driver.stop()  # Останавливаем поток драйвера


def main(args=None):
    rclpy.init(args=args)
    # Получаем путь к директории booblik
    booblik_dir = get_directory(target='booblik') 
    config_file = os.path.join(booblik_dir, 'config.json') 
    config = load_config(config_file)  # Загрузка конфигурации
    task = MotorsNode(config['motors'])   # Передаем конфигурацию в конструктор ноды
    try:
        rclpy.spin(task)
    except KeyboardInterrupt:
        pass
    finally:
        task.stop()  # Остановка ноды и потоков
        task.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()