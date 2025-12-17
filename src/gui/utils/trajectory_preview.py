"""
Utilidad para vista previa de trayectorias.

Módulo separado para reducir el tamaño de TestTab.
"""

import numpy as np
from PyQt5.QtWidgets import (QDialog, QVBoxLayout, QHBoxLayout, QLabel,
                             QTableWidget, QTableWidgetItem, QHeaderView, QWidget)
from PyQt5.QtCore import Qt

from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure


def show_trajectory_preview(parent, trajectory: np.ndarray) -> bool:
    """
    Muestra una ventana de vista previa de la trayectoria.
    
    Args:
        parent: Widget padre para el diálogo
        trajectory: Array numpy con puntos (N, 2) en µm
        
    Returns:
        True si se mostró correctamente, False si hubo error
    """
    if trajectory is None or len(trajectory) == 0:
        return False
    
    # Crear ventana de vista previa
    dialog = QDialog(parent)
    dialog.setWindowTitle("📊 Vista Previa de Trayectoria")
    dialog.setGeometry(100, 100, 1200, 700)
    dialog.setStyleSheet("background-color: #2E2E2E; color: white;")
    
    main_layout = QHBoxLayout()
    
    # === LADO IZQUIERDO: GRÁFICO XY ===
    fig = Figure(figsize=(8, 7), facecolor='#2E2E2E')
    ax = fig.add_subplot(111)
    
    x_coords = trajectory[:, 0]
    y_coords = trajectory[:, 1]
    
    # Trayectoria (línea azul)
    ax.plot(x_coords, y_coords, '-', color='#3498DB', linewidth=2, label='Trayectoria', zorder=1)
    
    # Puntos a visitar (puntos rojos)
    ax.scatter(x_coords, y_coords, c='red', s=50, zorder=2, label=f'Puntos ({len(x_coords)})')
    
    # Marcar inicio (verde) y fin (amarillo)
    ax.scatter(x_coords[0], y_coords[0], c='#2ECC71', s=150, marker='s', zorder=3, label='Inicio')
    ax.scatter(x_coords[-1], y_coords[-1], c='#F1C40F', s=150, marker='*', zorder=3, label='Fin')
    
    # Numerar algunos puntos clave
    for i in range(0, len(x_coords), max(1, len(x_coords)//10)):
        ax.annotate(f'{i+1}', (x_coords[i], y_coords[i]), fontsize=8, color='white',
                   xytext=(5, 5), textcoords='offset points')
    
    # Configurar ejes
    ax.set_xlabel('Posición X (µm)', color='white', fontsize=12)
    ax.set_ylabel('Posición Y (µm)', color='white', fontsize=12)
    ax.set_title(f'Trayectoria Zig-Zag - {len(trajectory)} puntos', 
                fontsize=14, fontweight='bold', color='white')
    
    # Límites con margen
    x_min, x_max = x_coords.min(), x_coords.max()
    y_min, y_max = y_coords.min(), y_coords.max()
    margin_x = (x_max - x_min) * 0.1 if x_max > x_min else 500
    margin_y = (y_max - y_min) * 0.1 if y_max > y_min else 500
    ax.set_xlim(x_min - margin_x, x_max + margin_x)
    ax.set_ylim(y_min - margin_y, y_max + margin_y)
    
    # Estilo
    ax.set_facecolor('#1a1a1a')
    ax.tick_params(colors='white', labelsize=10)
    ax.grid(True, alpha=0.3, linestyle='--', color='#555555')
    ax.legend(loc='upper right', facecolor='#383838', edgecolor='#555555', 
             labelcolor='white', fontsize=9)
    
    for spine in ax.spines.values():
        spine.set_color('#555555')
    
    fig.tight_layout()
    
    canvas = FigureCanvas(fig)
    
    # === LADO DERECHO: LISTA DE PUNTOS ===
    right_layout = QVBoxLayout()
    
    title_label = QLabel("📋 Lista de Puntos")
    title_label.setStyleSheet("font-size: 14px; font-weight: bold; padding: 5px;")
    right_layout.addWidget(title_label)
    
    # Tabla de puntos
    table = QTableWidget(len(trajectory), 3)
    table.setHorizontalHeaderLabels(["#", "X (µm)", "Y (µm)"])
    table.setStyleSheet("""
        QTableWidget {
            background-color: #1a1a1a;
            color: white;
            gridline-color: #444444;
            font-family: 'Courier New';
            font-size: 11px;
        }
        QHeaderView::section {
            background-color: #3498DB;
            color: white;
            padding: 5px;
            font-weight: bold;
        }
    """)
    
    # Llenar tabla con coordenadas
    for i, point in enumerate(trajectory):
        x, y = point[0], point[1]
        
        # Número de punto
        item_n = QTableWidgetItem(f"{i+1:03d}")
        item_n.setTextAlignment(Qt.AlignCenter)
        table.setItem(i, 0, item_n)
        
        # Coordenada X
        item_x = QTableWidgetItem(f"{x:.1f}")
        item_x.setTextAlignment(Qt.AlignRight | Qt.AlignVCenter)
        table.setItem(i, 1, item_x)
        
        # Coordenada Y
        item_y = QTableWidgetItem(f"{y:.1f}")
        item_y.setTextAlignment(Qt.AlignRight | Qt.AlignVCenter)
        table.setItem(i, 2, item_y)
    
    # Ajustar columnas
    table.horizontalHeader().setSectionResizeMode(0, QHeaderView.ResizeToContents)
    table.horizontalHeader().setSectionResizeMode(1, QHeaderView.Stretch)
    table.horizontalHeader().setSectionResizeMode(2, QHeaderView.Stretch)
    
    right_layout.addWidget(table)
    
    # Info resumen
    info_label = QLabel(f"Total: {len(trajectory)} puntos | "
                       f"X: [{x_min:.0f}, {x_max:.0f}] µm | "
                       f"Y: [{y_min:.0f}, {y_max:.0f}] µm")
    info_label.setStyleSheet("font-size: 11px; color: #888888; padding: 5px;")
    right_layout.addWidget(info_label)
    
    # Agregar widgets al layout principal
    right_widget = QWidget()
    right_widget.setLayout(right_layout)
    right_widget.setMinimumWidth(350)
    
    main_layout.addWidget(canvas, stretch=2)
    main_layout.addWidget(right_widget, stretch=1)
    
    dialog.setLayout(main_layout)
    dialog.exec_()
    
    return True
