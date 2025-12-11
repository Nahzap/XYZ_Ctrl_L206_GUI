#!/usr/bin/env python
"""
Setup AI - Configuración de modelos de Deep Learning para análisis de imágenes.

Este script:
1. Verifica que PyTorch esté instalado
2. Crea la estructura de carpetas necesaria
3. Descarga los pesos del modelo U2-Net si no existen

Uso:
    python setup_ai.py [--model u2netp|u2net]
"""

import os
import sys
from pathlib import Path

# Colores para terminal
class Colors:
    GREEN = '\033[92m'
    YELLOW = '\033[93m'
    RED = '\033[91m'
    BLUE = '\033[94m'
    RESET = '\033[0m'
    BOLD = '\033[1m'


def print_header():
    print(f"\n{Colors.BOLD}{'='*60}")
    print("  Setup AI - Configuración de U2-Net para Análisis de Imágenes")
    print(f"{'='*60}{Colors.RESET}\n")


def check_pytorch():
    """Verifica que PyTorch esté instalado."""
    print(f"{Colors.BLUE}[1/4] Verificando PyTorch...{Colors.RESET}")
    
    try:
        import torch
        print(f"  {Colors.GREEN}✓ PyTorch {torch.__version__} instalado{Colors.RESET}")
        
        # Verificar CUDA
        if torch.cuda.is_available():
            print(f"  {Colors.GREEN}✓ CUDA disponible: {torch.cuda.get_device_name(0)}{Colors.RESET}")
        else:
            print(f"  {Colors.YELLOW}⚠ CUDA no disponible, se usará CPU{Colors.RESET}")
        
        return True
    except ImportError:
        print(f"  {Colors.RED}✗ PyTorch no instalado{Colors.RESET}")
        print(f"\n  Para instalar PyTorch, ejecuta:")
        print(f"  {Colors.BOLD}pip install torch torchvision{Colors.RESET}")
        print(f"\n  O visita: https://pytorch.org/get-started/locally/")
        return False


def check_gdown():
    """Verifica que gdown esté instalado para descargar desde Google Drive."""
    print(f"\n{Colors.BLUE}[2/4] Verificando gdown...{Colors.RESET}")
    
    try:
        import gdown
        print(f"  {Colors.GREEN}✓ gdown instalado{Colors.RESET}")
        return True
    except ImportError:
        print(f"  {Colors.YELLOW}⚠ gdown no instalado, instalando...{Colors.RESET}")
        os.system(f"{sys.executable} -m pip install gdown")
        try:
            import gdown
            print(f"  {Colors.GREEN}✓ gdown instalado correctamente{Colors.RESET}")
            return True
        except ImportError:
            print(f"  {Colors.RED}✗ No se pudo instalar gdown{Colors.RESET}")
            return False


def create_directories():
    """Crea la estructura de carpetas necesaria."""
    print(f"\n{Colors.BLUE}[3/4] Creando estructura de carpetas...{Colors.RESET}")
    
    base_dir = Path(__file__).parent
    dirs = [
        base_dir / "models" / "weights",
        base_dir / "src" / "models" / "u2net",
    ]
    
    for d in dirs:
        d.mkdir(parents=True, exist_ok=True)
        print(f"  {Colors.GREEN}✓ {d.relative_to(base_dir)}{Colors.RESET}")
    
    return True


def download_weights(model_type='u2netp'):
    """Descarga los pesos del modelo U2-Net."""
    print(f"\n{Colors.BLUE}[4/4] Descargando pesos del modelo {model_type}...{Colors.RESET}")
    
    import gdown
    
    base_dir = Path(__file__).parent
    weights_dir = base_dir / "models" / "weights"
    
    # URLs de Google Drive (oficiales del repositorio U2-Net)
    urls = {
        'u2netp': {
            'url': 'https://drive.google.com/uc?id=1rbSTGKAE-MTxBYHd-51l2hMOQPT_7EPy',
            'file': 'u2netp.pth',
            'size': '~4.7 MB'
        },
        'u2net': {
            'url': 'https://drive.google.com/uc?id=1ao1ovG1Qtx4b7EoskHXmi2E9rp5CHLcZ',
            'file': 'u2net.pth',
            'size': '~176 MB'
        }
    }
    
    if model_type not in urls:
        print(f"  {Colors.RED}✗ Modelo desconocido: {model_type}{Colors.RESET}")
        return False
    
    info = urls[model_type]
    output_path = weights_dir / info['file']
    
    if output_path.exists():
        print(f"  {Colors.GREEN}✓ Pesos ya existen: {info['file']}{Colors.RESET}")
        return True
    
    print(f"  Descargando {info['file']} ({info['size']})...")
    
    try:
        gdown.download(info['url'], str(output_path), quiet=False)
        
        if output_path.exists():
            size_mb = output_path.stat().st_size / (1024 * 1024)
            print(f"  {Colors.GREEN}✓ Descargado: {info['file']} ({size_mb:.1f} MB){Colors.RESET}")
            return True
        else:
            print(f"  {Colors.RED}✗ Error: archivo no creado{Colors.RESET}")
            return False
    except Exception as e:
        print(f"  {Colors.RED}✗ Error descargando: {e}{Colors.RESET}")
        print(f"\n  Descarga manual desde:")
        print(f"  {Colors.BOLD}{info['url']}{Colors.RESET}")
        print(f"  Guarda en: {output_path}")
        return False


def verify_installation():
    """Verifica que todo esté correctamente instalado."""
    print(f"\n{Colors.BOLD}{'='*60}")
    print("  Verificación Final")
    print(f"{'='*60}{Colors.RESET}\n")
    
    base_dir = Path(__file__).parent
    
    # Verificar modelo
    weights_path = base_dir / "models" / "weights" / "u2netp.pth"
    if weights_path.exists():
        print(f"  {Colors.GREEN}✓ Modelo U2-NETP listo{Colors.RESET}")
    else:
        print(f"  {Colors.RED}✗ Modelo U2-NETP no encontrado{Colors.RESET}")
        return False
    
    # Verificar importación
    try:
        sys.path.insert(0, str(base_dir / "src"))
        from ai_segmentation import SalientObjectDetector
        print(f"  {Colors.GREEN}✓ Módulo ai_segmentation importable{Colors.RESET}")
    except Exception as e:
        print(f"  {Colors.RED}✗ Error importando: {e}{Colors.RESET}")
        return False
    
    print(f"\n{Colors.GREEN}{Colors.BOLD}✓ Setup completado exitosamente!{Colors.RESET}")
    print(f"\nPuedes usar el SmartFocusScorer con U2-Net para análisis de imágenes.")
    return True


def main():
    import argparse
    
    parser = argparse.ArgumentParser(description='Setup AI models for image analysis')
    parser.add_argument('--model', choices=['u2netp', 'u2net'], default='u2netp',
                       help='Modelo a descargar (default: u2netp)')
    parser.add_argument('--full', action='store_true',
                       help='Descargar ambos modelos (u2netp y u2net)')
    args = parser.parse_args()
    
    print_header()
    
    # Paso 1: Verificar PyTorch
    if not check_pytorch():
        sys.exit(1)
    
    # Paso 2: Verificar gdown
    if not check_gdown():
        sys.exit(1)
    
    # Paso 3: Crear directorios
    create_directories()
    
    # Paso 4: Descargar pesos
    if args.full:
        download_weights('u2netp')
        download_weights('u2net')
    else:
        download_weights(args.model)
    
    # Verificación final
    verify_installation()


if __name__ == '__main__':
    main()
