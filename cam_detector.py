from pylablib.devices import Thorlabs

# Lista las cámaras científicas de Thorlabs (como la Kiralux)
# La primera vez puede tardar un poco mientras busca los drivers.
cameras = Thorlabs.list_cameras_tlcam()

if not cameras:
    print("No se encontraron cámaras Thorlabs.")
else:
    print("Cámaras encontradas:")
    for cam in cameras:
        print(f"- {cam}")