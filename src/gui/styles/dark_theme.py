"""Tema oscuro para la aplicación."""

# --- Hoja de estilos para el TEMA OSCURO ---
DARK_STYLESHEET = """
QWidget {
    background-color: #2E2E2E;
    color: #F0F0F0;
    font-family: Arial;
}
QGroupBox {
    background-color: #383838;
    border: 1px solid #505050;
    border-radius: 5px;
    margin-top: 1ex;
    font-weight: bold;
}
QGroupBox::title {
    subcontrol-origin: margin;
    subcontrol-position: top center;
    padding: 0 3px;
}
QLabel, QCheckBox {
    background-color: transparent;
}
QPushButton {
    background-color: #505050;
    border: 1px solid #606060;
    border-radius: 4px;
    padding: 5px;
}
QPushButton:hover {
    background-color: #606060;
}
QPushButton:pressed {
    background-color: #2E86C1;
}
QLineEdit, QTextEdit {
    background-color: #505050;
    border: 1px solid #606060;
    border-radius: 4px;
    padding: 3px;
    color: #F0F0F0;
}
QCheckBox::indicator {
    width: 13px;
    height: 13px;
}
"""


def get_dark_stylesheet():
    """
    Retorna el stylesheet del tema oscuro.
    
    Returns:
        str: Stylesheet en formato Qt CSS
    """
    return DARK_STYLESHEET
