"""Configuración del sistema de logging."""
import logging
import sys
from datetime import datetime


class SafeUnicodeStreamHandler(logging.StreamHandler):
    """StreamHandler que no falla con cp1252 en consola Windows (emojis, µm, etc.)."""

    def emit(self, record):
        try:
            msg = self.format(record)
            stream = self.stream
            try:
                stream.write(msg + self.terminator)
            except UnicodeEncodeError:
                enc = getattr(stream, "encoding", None) or "utf-8"
                safe = (msg + self.terminator).encode(enc, errors="replace").decode(enc, errors="replace")
                stream.write(safe)
            self.flush()
        except Exception:
            self.handleError(record)


def setup_logging():
    """
    Configura el sistema de logging según IEEE Software Engineering Standards.
    
    El archivo de log se REINICIA en cada ejecución (mode='w') para facilitar
    la revisión del log de la sesión actual.
    
    Returns:
        logging.Logger: Logger configurado para la aplicación
    """
    # Nombre del archivo de log (se reinicia cada día, pero también cada ejecución)
    log_filename = f'motor_control_{datetime.now().strftime("%Y%m%d")}.log'
    
    # Configurar logging con formato IEEE
    # IMPORTANTE: mode='w' reinicia el archivo en cada ejecución
    logging.basicConfig(
        level=logging.DEBUG,
        format='%(asctime)s | %(levelname)-8s | %(name)s | %(funcName)s:%(lineno)d | %(message)s',
        datefmt='%Y-%m-%d %H:%M:%S',
        handlers=[
            SafeUnicodeStreamHandler(sys.stdout),
            logging.FileHandler(
                log_filename, 
                mode='w',  # 'w' = write (reinicia), 'a' = append (acumula)
                encoding='utf-8'
            )
        ],
        force=True  # Forzar reconfiguración si ya existe
    )
    
    # Silenciar logs de librerías externas (matplotlib, PIL, etc.)
    logging.getLogger('matplotlib').setLevel(logging.WARNING)
    logging.getLogger('matplotlib.font_manager').setLevel(logging.WARNING)
    logging.getLogger('PIL').setLevel(logging.WARNING)
    
    return logging.getLogger('MotorControl_L206')
