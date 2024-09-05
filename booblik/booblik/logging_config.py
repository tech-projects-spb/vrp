import os
import time
import logging
import stat  # Для работы с правами доступа

DEBUG = True

def setup_logging(node_name=None, log_filename=None):
    """
    Настройка логгера для записи в папку src/vrp/log.
    
    :param node_name: Имя ноды для использования в качестве имени файла по умолчанию.
    :param log_filename: Имя файла для логов (если передано).
    """
    print("Setting up logging...")

    # Используем переменную COLCON_PREFIX_PATH для нахождения корневой директории рабочего пространства
    workspace_dir = os.environ.get('COLCON_PREFIX_PATH')
    if workspace_dir:
        # Переходим в директорию src/vrp/log относительно COLCON_PREFIX_PATH
        log_dir = os.path.join(workspace_dir, '..', 'src', 'vrp', 'log')
        log_dir = os.path.abspath(log_dir)  # Преобразуем в абсолютный путь
    else:
        print("Error: COLCON_PREFIX_PATH is not set. Falling back to default workspace path.")
        # Если переменная не установлена, можно задать путь вручную (измените на ваш путь)
        workspace_dir = os.path.expanduser('~/vrp_ws')
        log_dir = os.path.join(workspace_dir, 'src', 'vrp', 'log')

    # Проверяем, существует ли папка для логов, и создаем её, если не существует
    if not os.path.exists(log_dir):
        print(f"Directory {log_dir} does not exist. Creating it...")
        os.makedirs(log_dir)

    # Определяем имя файла для логов
    localdate = time.strftime("%Y.%m.%d")
    if log_filename:
        logfile = os.path.join(log_dir, f'{log_filename}-{localdate}.log')
    elif node_name:
        logfile = os.path.join(log_dir, f'{node_name}-{localdate}.log')
    else:
        logfile = os.path.join(log_dir, f'log-{localdate}.log')

    print(f"Log file will be: {logfile}")

    # Проверка прав на запись
    try:
        if not os.access(log_dir, os.W_OK):
            print(f'No write permission to log directory {log_dir}. Trying to set permissions...')
            os.chmod(log_dir, stat.S_IRWXU | stat.S_IRWXG | stat.S_IROTH | stat.S_IXOTH)

        # Проверяем, если файл существует, можно ли в него записывать
        if os.path.exists(logfile) and not os.access(logfile, os.W_OK):
            print(f'No write permission for log file {logfile}. Trying to set permissions...')
            os.chmod(logfile, stat.S_IRWXU | stat.S_IRWXG | stat.S_IROTH | stat.S_IXOTH)

    except PermissionError as e:
        print(f'Error: Unable to set write permissions. {e}')
        return

    # Настройка логгера для записи в файл
    logging.basicConfig(
        level=logging.DEBUG if DEBUG else logging.INFO,
        format='%(asctime)s [%(name)s] [%(levelname)-5.5s] %(message)s',
        filename=logfile,
        filemode='a'
    )

    # Проверяем, что логгер настроен
    logger = logging.getLogger()
    print(f"Logger handlers: {logger.handlers}")
    logger.info("Logging setup complete. Logs will be written to file.")
