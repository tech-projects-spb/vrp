import math
import os
import stat
import time
import json
import logging
import cv2
import numpy as np

logger = logging.getLogger(__name__)

def euler_to_quaternion(yaw: float, pitch: float, roll: float)-> tuple[float, float, float, float]:
    """Преобразование углов Эйлера в кватернионы для описания ориентации в пространстве"""
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    qw = cy * cp * cr + sy * sp * sr
    qx = cy * cp * sr - sy * sp * cr
    qy = sy * cp * sr + cy * sp * cr
    qz = sy * cp * cr - cy * sp * sr

    return qw, qx, qy, qz

def euler_from_quaternion(x: float, y: float, z: float, w: float) -> tuple[float, float, float, float]:
    """
    Преобразование углов Эйлера в кватернионы (roll, pitch, yaw)
    :param roll: крен (вращение вокруг оси x в радианах, против часовой стрелки)
    :param pitch: тангаж (вращение вокруг оси y в радианах, против часовой стрелки)
    :param yaw: рыскание (вращение вокруг оси z в радианах, против часовой стрелки)
    """
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)

    return roll_x, pitch_y, yaw_z  # in radians

def kmph_2_mps(kmph: float) ->  float:
    """Конвертация км/ч в м/с"""
    return kmph / 3.6

def knot_2_mps(knots: float) ->  float:
    """Конвертация узлы в м/с"""
    return knots * 0.514444

def declin_dir(mag_var: float, direction: str) -> float:
    """Обработка направления магнитного склонения"""
    return mag_var if direction == 'E' else -mag_var

def load_config(file_path: str) ->  dict:
    """Функция для загрузки конфигурационного файла"""
    try:
        with open(file_path, 'r') as file:
            config = json.load(file)
            logger.info(f"Конфигурация загружена из {file_path}")
            return config
    except FileNotFoundError:
        logger.error(f"Файл конфигурации {file_path} не найден.")
        return {}
    except json.JSONDecodeError:
        logger.error(f"Ошибка в формате конфигурационного файла {file_path}.")
        return {}
    
def get_directory(target: str = 'log', *path_segments: dict) -> str:
    """
    Универсальная функция для получения пути к директориям.
    Может возвращать путь к папке для логов или к папке с кодом (например, booblik).
    
    :param target: Целевая директория, либо 'log' для папки с логами, либо 'booblik' для кода.
    :param path_segments: Дополнительные сегменты пути, которые можно добавить к базовой директории.
    :return: Абсолютный путь к целевой директории.
    """
    # Получаем первый путь из COLCON_PREFIX_PATH, если их несколько
    workspace_dir = os.environ.get('COLCON_PREFIX_PATH', '').split(':')[0]

    if not workspace_dir:
        print("Error: COLCON_PREFIX_PATH is not set. Falling back to default workspace path.")
        workspace_dir = os.path.expanduser('~/vrp_ws')

    if target == 'log':
        # Переходим в директорию src/vrp/log относительно workspace_dir
        dir_path = os.path.join(workspace_dir, '..', 'src', 'vrp', 'log')

    elif target == 'booblik':
        # Путь к директории booblik
        dir_path = os.path.join(workspace_dir, '..', 'src', 'vrp', 'booblik', 'booblik')

    else:
        raise ValueError(f"Unknown target: {target}. Use 'log' or 'booblik'.")

    # Добавляем дополнительные сегменты пути, если они указаны
    if path_segments:
        dir_path = os.path.join(dir_path, *path_segments)

    # Преобразуем в абсолютный путь
    dir_path = os.path.abspath(dir_path)

    # Отладочный вывод для проверки пути
    print(f"Resolved directory path: {dir_path}")

    # Проверяем, существует ли папка, и создаем её, если не существует
    if not os.path.exists(dir_path):
        print(f"Directory {dir_path} does not exist. Creating it...")
        os.makedirs(dir_path)

    # Проверка прав на запись для логов (или других папок, если нужно)
    try:
        if not os.access(dir_path, os.W_OK):
            print(f'No write permission to directory {dir_path}. Trying to set permissions...')
            os.chmod(dir_path, stat.S_IRWXU | stat.S_IRWXG | stat.S_IROTH | stat.S_IXOTH)
    except PermissionError as e:
        print(f'Error: Unable to set write permissions. {e}')
        return None

    return dir_path

def get_filename(log_dir: str, log_filename: str = None, node_name: str = None, date: bool = False) -> str:
    """
    Возвращает имя файла для логов, используя имя файла, имя ноды или стандартное имя.
    
    :param log_dir: Путь к директории логов.
    :param log_filename: Явное имя файла для логов.
    :param node_name: Имя ноды (используется как имя файла, если log_filename не указано).
    :param date: Если True, добавляет дату в имя файла.
    :return: Полный путь к файлу лога.
    """
    # Формируем имя файла: используем log_filename, если передано, иначе node_name, если нет — "log"
    name = log_filename or node_name or 'log'
    
    # Получаем текущую дату
    localdate = time.strftime("%Y.%m.%d") if date else ''
    
    # Если требуется добавить дату в имя файла
    if date:
        logfile = os.path.join(log_dir, f'{name}-{localdate}.log')
    else:
        logfile = os.path.join(log_dir, f'{name}.log')

    print(f"Log file will be: {logfile}")
    return logfile

def resize(old_size: tuple[int, int], frame:  tuple[int, int]) -> tuple[int, int]:
    proportion = max(old_size[0] / frame[0], old_size[1] / frame[1])
    return (int(old_size[0] / proportion), int(old_size[1] / proportion))

def modified_image(image: np.ndarray, new_height: int, new_width: int) -> np.ndarray:
    """
    Масштабирует изображение под заданные размеры и центрирует его на холсте для работы с LED модулем.
    """
    old_height, old_width = image.shape[:2] 
    new_size = resize((old_height, old_width), (new_height, new_width) ) 
    resized_image = cv2.resize(image, (new_size[1], new_size[0]), interpolation = cv2.INTER_AREA)
    new_image = np.zeros((new_height, new_width, 3), dtype="uint8")
    start_y = (new_height - new_size[0]) // 2
    start_x = (new_width - new_size[1]) // 2 
    # Размещаем масштабированное изображение на новом холсте
    new_image[start_y:start_y + new_size[0], start_x:start_x + new_size[1]] = resized_image
    return new_image
