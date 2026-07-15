# XYZ_Ctrl_L206_GUI: Automated Microscopy System for Pollen Analysis

[![Python](https://img.shields.io/badge/Python-3.11-blue.svg)](https://www.python.org/)
[![PyQt5](https://img.shields.io/badge/PyQt5-5.15.11-green.svg)](https://www.riverbankcomputing.com/software/pyqt/)
[![License](https://img.shields.io/badge/License-Open_Source-orange.svg)](LICENSE)

## 📋 Overview

**XYZ_Ctrl_L206_GUI** is an advanced automated microscopy system designed for high-throughput analysis of pollen grains in honey samples from the Bío-Bío Region, Chile. The system integrates:

- **Precision XY positioning** using L206 DC motor drivers with H∞ robust control
- **Piezoelectric Z-axis control** (C-Focus, Mad City Labs) for sub-micron autofocus
- **Deep learning-based detection** using U2-Net for salient object detection
- **Automated microscopy workflows** for multi-object capture and analysis
- **Real-time control and data acquisition** via STM32F767ZI microcontroller

### Key Features

✅ **Automated Pollen Detection**: U2-Net deep learning model for robust pollen grain identification  
✅ **Intelligent Autofocus**: Multi-object Z-scanning with Laplacian variance optimization  
✅ **Robust Motion Control**: H∞/H2 synthesis for precise positioning with disturbance rejection  
✅ **High-Throughput Imaging**: Automated trajectory execution with multi-focal capture  
✅ **Real-Time Visualization**: Live camera feed with detection overlays and saliency maps  
✅ **Comprehensive Analysis**: Transfer function identification and system characterization  

---

## 🎯 Scientific Application

This system is specifically designed for **melissopalynology** - the microscopic analysis of pollen in honey to determine botanical origin. The Bío-Bío Region of Chile has unique endemic flora, making accurate pollen identification crucial for:

- **Honey authentication and quality control**
- **Biodiversity monitoring and conservation**
- **Climate change impact assessment**
- **Agricultural and apicultural research**

---

## 🏗️ System Architecture

### Hardware Components

| Component | Model/Type | Function |
|-----------|------------|----------|
| **XY Stage** | L206/L298N + STM32F767ZI | Posicionamiento calibrado (span según banco: p. ej. 3–25 mm) |
| **Z-Axis** | C-Focus Piezo (Mad City Labs) | Sub-micron autofocus (68µm range) |
| **Camera** | Thorlabs / Basler Scientific Camera | High-resolution microscopy imaging |
| **Controller** | NUCLEO-STM32F767ZI (MycoViT_XY_Controller) | Real-time motor control and 12-bit sensing |
| **PC Interface** | USB Serial ST-Link VCP (**1 000 000 baud**) | Command/telemetry communication |

### Software Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                     PRESENTATION LAYER                       │
│  ┌──────────┐ ┌──────────┐ ┌──────────┐ ┌──────────┐       │
│  │ Control  │ │ Camera   │ │  Test    │ │  H∞      │       │
│  │   Tab    │ │   Tab    │ │   Tab    │ │  Tab     │       │
│  └────┬─────┘ └────┬─────┘ └────┬─────┘ └────┬─────┘       │
└───────┼────────────┼────────────┼────────────┼─────────────┘
        │            │            │            │
┌───────┼────────────┼────────────┼────────────┼─────────────┐
│       │       SERVICE LAYER     │            │              │
│  ┌────┴─────┐ ┌────┴──────┐ ┌──┴────┐ ┌─────┴─────┐       │
│  │ Camera   │ │Microscopy │ │ Test  │ │   H∞      │       │
│  │ Service  │ │  Service  │ │Service│ │ Service   │       │
│  └────┬─────┘ └────┬──────┘ └───┬───┘ └─────┬─────┘       │
│       │            │            │            │              │
│  ┌────┴─────┐ ┌────┴──────┐ ┌──┴────┐ ┌─────┴─────┐       │
│  │Detection │ │ Autofocus │ │Traj.  │ │   H∞      │       │
│  │ Service  │ │  Service  │ │Gen.   │ │Controller │       │
│  └────┬─────┘ └────┬──────┘ └───┬───┘ └─────┬─────┘       │
└───────┼────────────┼────────────┼────────────┼─────────────┘
        │            │            │            │
┌───────┼────────────┼────────────┼────────────┼─────────────┐
│       │       HARDWARE LAYER    │            │              │
│  ┌────┴─────┐ ┌────┴──────┐ ┌──┴────┐ ┌─────┴─────┐       │
│  │ Camera   │ │  C-Focus  │ │Serial │ │Data       │       │
│  │ Worker   │ │Controller │ │Handler│ │Recorder   │       │
│  └──────────┘ └───────────┘ └───────┘ └───────────┘       │
└─────────────────────────────────────────────────────────────┘
```

---

## 🚀 Installation

### Prerequisites

- **Python 3.11** (entorno de referencia: `CTRL_ENV`, conda-forge)
- **Windows 10/11**
- **CUDA 12.1** (recomendado para U2-Net; CPU también funciona)
- **Pylon Runtime** (Basler) y/o **ThorCam** (Thorlabs), según cámara
- **Firmware MCU:** `MycoViT_XY_Controller` (NUCLEO-F767ZI)

### Entorno CTRL_ENV (canónico)

El proyecto usa el prefijo `./CTRL_ENV` (Python en la raíz del prefijo, no un `venv` clásico con `Scripts\python.exe`).  
`CTRL_ENV\Lib\site-packages\sitecustomize.py` **desactiva el user-site** (`AppData\Roaming\Python\...`) para que todos los paquetes salgan de CTRL_ENV.

```powershell
# Desde la raíz del repo
$env:PYTHONNOUSERSITE = "1"

# Dependencias PyPI
.\CTRL_ENV\python.exe -m pip install -r requirements.txt

# PyTorch CUDA 12.1 (índice aparte; no está en PyPI)
.\CTRL_ENV\python.exe -m pip install torch==2.5.1+cu121 torchvision==0.20.1+cu121 --index-url https://download.pytorch.org/whl/cu121
```

Verificación rápida:

```powershell
.\CTRL_ENV\python.exe -c "import site,torch,pypylon,h5py; print('user_site', site.ENABLE_USER_SITE); print('torch', torch.__version__, 'cuda', torch.cuda.is_available()); print('pypylon OK')"
```

### Cámaras (SO + Python)

| Cámara | Runtime SO | Paquete Python |
|--------|------------|----------------|
| **Basler** | [Pylon](https://www.baslerweb.com/en/downloads/software-downloads/) | `pypylon` (≥26.6) |
| **Thorlabs** | [ThorCam](https://www.thorlabs.com/software_pages/ViewSoftwarePage.cfm?Code=ThorCam) | `pylablib` (≥1.4.4) |

### Hardware / configuración

1. Flashear `MycoViT_XY_Controller` en NUCLEO-F767ZI  
2. Conectar ST-Link VCP (p. ej. COM5 @ **1 000 000** baud)  
3. Ajustar `src/config/constants.py` y `src/config/calibration.json`:
   ```python
   SERIAL_PORT = 'COM5'
   BAUD_RATE = 1000000
   # ADC 12-bit: system.adc_max = 4095
   ```
4. Pesos U2-Net: `models/u2net/u2netp.pth` ([U-2-Net](https://github.com/xuebinqin/U-2-Net))

---

## 📖 Usage

### Quick Start

```powershell
.\CTRL_ENV\python.exe src\main.py
```

### Basic Workflow

#### 1. **Connect Hardware**
- Click **"Conectar"** in Control tab
- Verify STM32 connection (green status; COM ST-Link @ 1 Mbps)
- Connect camera in Camera tab
- Connect C-Focus in Camera tab

#### 2. **Calibrate System**
- Navigate to **Test tab**
- Run step response tests for system identification
- Save calibration data

#### 3. **Design H∞ Controller** (Optional)
- Go to **H∞ tab**
- Load plant model from calibration
- Synthesize robust controller
- Apply on PC (AUTO via `A,<pwm_a>,<pwm_b>`); MCU hold on-chip is roadmap

#### 4. **Automated Microscopy**
- Go to **Camera tab**
- Define trajectory (grid pattern)
- Configure detection parameters:
  - Min/Max area for pollen grains
  - Autofocus range and step size
- Click **"Start Microscopy"**
- System will:
  1. Move to each point
  2. Detect pollen grains
  3. Autofocus on each grain
  4. Capture multi-focal images
  5. Save results with metadata

#### 5. **Analyze Results**
- Images saved in `captures/YYYYMMDD_HHMMSS/`
- CSV with coordinates and focus scores
- Review detection overlays

---

## 🔬 Technical Details

### U2-Net Detection Pipeline

```python
# Singleton pattern - model loaded once at startup
detector = U2NetDetector.get_instance()

# Real-time detection
saliency_map, objects = detector.detect(frame)

# Objects filtered by:
# - Area (500 - 500,000 px²)
# - Circularity (> 0.45)
# - Probability (> 0.3)
```

**Performance**: ~50-80ms per frame on GPU, ~300-500ms on CPU

### Autofocus Algorithm

**Method**: Hill-climbing with Laplacian variance metric

```python
# Z-scanning parameters
z_range = 68.0  # µm (C-Focus full range)
z_step_coarse = 5.0  # µm (initial scan)
z_step_fine = 0.5  # µm (refinement)

# Sharpness metric
S = Var(∇²I) * 10.0  # Laplacian variance
```

**Typical performance**: 2-3 seconds per object

### H∞ Robust Control

**Plant Model** (identified from step response):
```
G(s) = K / (τs + 1)
```

**Controller Synthesis**:
- Mixed sensitivity H∞/H2 optimization
- Performance weight: Wp(s) for tracking
- Robustness weight: Wu(s) for control effort
- Disturbance rejection: Wd(s)

**Implementation**: Discrete-time state-space (Ts = 33ms)

---

## 📊 Data Output

### Directory Structure

```
captures/
└── 20260114_153045/
    ├── metadata.json          # Experiment configuration
    ├── trajectory.csv         # XY coordinates
    ├── point_001/
    │   ├── object_01_F0.png  # Best focus
    │   ├── object_01_F1.png  # +5µm
    │   ├── object_01_F2.png  # -5µm
    │   └── metadata.json     # Object info
    ├── point_002/
    │   └── ...
    └── summary.csv           # All detections
```

### Metadata Format

```json
{
  "timestamp": "2026-01-14T15:30:45",
  "point_id": 1,
  "xy_position": [1250.5, 2340.8],
  "objects": [
    {
      "id": 1,
      "bbox": [120, 340, 85, 82],
      "area": 5234,
      "z_optimal": 25.02,
      "focus_score": 8.1,
      "probability": 0.87
    }
  ]
}
```

---

## 🛠️ Configuration

### Key Parameters

Edit `src/config/constants.py`:

```python
# Serial Communication (STM32F767ZI ST-Link VCP)
SERIAL_PORT = 'COM5'
BAUD_RATE = 1000000  # 1 Mbps; telemetry CSV 6 fields, ADC 12-bit (0-4095)

# Autofocus Parameters
Z_SCAN_RANGE = 68.0        # µm
Z_STEP_COARSE = 5.0        # µm
Z_STEP_FINE = 0.5          # µm
SETTLE_TIME = 0.05         # seconds
```

---

## 📚 Documentation

- **[Architecture Documentation](docs/ARCHITECTURE.md)** - Detailed system design
- **[API Reference](docs/API_REFERENCE.md)** - Service and controller APIs
- **[User Manual](docs/USER_MANUAL.md)** - Complete usage guide
- **[Development Guide](docs/DEVELOPMENT.md)** - Contributing guidelines
- **[Academic Paper](docs/ACADEMIC_DRAFT.md)** - Scientific publication draft

---

## 🧪 Testing

### Unit Tests

```bash
pytest tests/
```

### Hardware-in-Loop Tests

```bash
python tests/test_hardware_integration.py
```

### Performance Benchmarks

```bash
python tests/benchmark_detection.py
python tests/benchmark_autofocus.py
```

---

## 🐛 Troubleshooting

### Camera Not Detected

**Basler:** Pylon Runtime instalado; USB;  
`.\CTRL_ENV\python.exe -c "from pypylon import pylon; print(len(pylon.TlFactory.GetInstance().EnumerateDevices()))"`

**Thorlabs:** ThorCam instalado; USB 3.0;  
`.\CTRL_ENV\python.exe -c "import pylablib as pll; print(pll.list_cameras())"`

### STM32 / Serial Connection Failed

1. Check COM port in Device Manager (ST-Link Virtual COM Port)
2. Verify baud rate (**1000000**)
3. Close other apps using the port; re-flash `MycoViT_XY_Controller` if needed
4. Confirm telemetry format: `potA,potB,s1,s2,STATE,settled` with sensors 0-4095

### Imports from AppData / wrong packages

Si aparecen rutas `AppData\Roaming\Python\...`, el user-site está filtrándose. Con CTRL_ENV + `sitecustomize.py` debe ser `ENABLE_USER_SITE False`. Reinstalar con `$env:PYTHONNOUSERSITE=1`.

### U2-Net Model Not Loading

1. Verify `models/u2net/u2netp.pth` exists
2. Check PyTorch: `.\CTRL_ENV\python.exe -c "import torch; print(torch.__version__, torch.cuda.is_available())"`
3. Si falta CUDA build: reinstalar `torch==2.5.1+cu121` desde el índice PyTorch (ver Installation)

### Autofocus Not Working

1. Check C-Focus connection
2. Verify Z-range in config
3. Ensure camera is in focus range

---

## 📈 Performance Metrics

| Metric | Value |
|--------|-------|
| Detection Speed | 50-80ms/frame (GPU) |
| Autofocus Time | 2-3s/object |
| Positioning Accuracy (product goal) | ±8 µm XY estable ≥300 ms; Z piezo ~0.1 µm |
| Throughput | ~20-30 objects/minute |
| Classification Accuracy | 93% (7 taxa, phase contrast) |

---

## 🤝 Contributing

We welcome contributions! Please see [CONTRIBUTING.md](CONTRIBUTING.md) for guidelines.

### Development Setup

```bash
# Install development dependencies
pip install -r requirements-dev.txt

# Run linters
flake8 src/
black src/

# Run tests
pytest tests/ --cov=src
```

---

## 📄 License

This project is open source. See [LICENSE](LICENSE) for details.

---

## 🙏 Acknowledgments

- **U2-Net**: Qin et al., "U²-Net: Going deeper with nested U-structure for salient object detection"
- **Thorlabs**: Camera SDK and hardware support
- **Mad City Labs**: C-Focus piezoelectric controller
- **Universidad del Bío-Bío**: Research support and facilities

---

## 📞 Contact

**Project Lead**: [Your Name]  
**Institution**: Universidad de Concepcion, Chile  
**Email**: your.email@udec.cl  
**GitHub**: https://github.com/yourusername/XYZ_Ctrl_L206_GUI

---

## 📖 Citation

If you use this system in your research, please cite:

```bibtex
@article{yourname2026automated,
  title={Automated Microscopy System for Pollen Analysis Using Deep Learning and Robust Control},
  author={Your Name and Collaborators},
  journal={Journal Name},
  year={2026},
  publisher={Publisher}
}
```

---

## 🗺️ Roadmap

### Control micrométrico (en curso — ver Docs)

Plan: `Docs/20260714_0032_Plan_Implementacion_Control_Micrometrico_Rapido.md`  
(firmware: `MycoViT_XY_Controller/Docs/…`)

| Fase | Estado |
|------|--------|
| 0 Baseline telemetría ~3.4 kHz | ✅ |
| 1 ControlWorker + UI ~30 Hz | ✅ |
| 2 Pulsos FOV wall-clock (ms) + permanencia 300 ms | ✅ |
| 3 Mini-pulso MCU + `P,axis,sign,idx` | ⏸️ Pausada (banco) |
| 4 LUT micrométrica STM32 | ⏸️ Pausada |
| 5 Validación ±8 µm microscopía | ⏸️ Pausada |

### Producto

- [ ] Multi-species classification model
- [ ] Automated report generation
- [ ] Framing binario / C(z) en hook 1 MHz (stretch)

---

**Last Updated**: 2026-07-14  
**Version**: 2.3.0 (CTRL_ENV aislado; control host Fases 0–2)
