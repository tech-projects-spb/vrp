import math
import os
import stat


def euler_to_quaternion(yaw, pitch, roll):
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

def euler_from_quaternion(x, y, z, w):
    """
    Convert a quaternion into euler angles (roll, pitch, yaw)
    roll is rotation around x in radians (counterclockwise)
    pitch is rotation around y in radians (counterclockwise)
    yaw is rotation around z in radians (counterclockwise)
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

def kmph_2_mps(kmph):
    """Конвертация км/ч в м/с"""
    return kmph / 3.6

def knot_2_mps(knots):
    """Конвертация узлы в м/с"""
    return knots * 0.514444

def declin_dir(mag_var, direction):
    """Обработка направления магнитного склонения"""
    return mag_var if direction == 'E' else -mag_var

def get_directory(target='log', *path_segments):
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