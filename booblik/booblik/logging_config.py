from booblik.utils import get_directory, get_filename
import logging 

DEBUG = True

def setup_logging(node_name=None, log_filename=None, date=True):
    """
    Настраивает логирование в файл. Файл сохраняется в директории логов src/vrp/log.
    
    :param node_name: Имя ноды, если нужно включить его в имя лог-файла.
    :param log_filename: Явное имя лог-файла.
    :param date: Включать ли дату в имя файла.
    """
    # Получаем путь к директории логов
    log_dir = get_directory(target='log')
    
    # Получаем имя лог-файла
    logfile = get_filename(log_dir, node_name=node_name, log_filename=log_filename, date=date)
    
    # Настройка логгера
    logging.basicConfig(
        filename=logfile,
        level=logging.DEBUG if DEBUG else logging.INFO,  # Уровень логирования
        format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
    )
    
    print(f"Logging is set up. Logs will be written to {logfile}")
    return logging.getLogger(node_name if node_name else 'default')