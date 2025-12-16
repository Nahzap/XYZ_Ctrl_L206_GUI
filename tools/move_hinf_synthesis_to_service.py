import re
from pathlib import Path


ROOT = Path(r"c:\Users\askna\PycharmProjects\XYZ_Ctrl_L206_GUI")
PATH_TAB = ROOT / "src" / "gui" / "tabs" / "hinf_tab.py"
PATH_SERVICE = ROOT / "src" / "core" / "services" / "hinf_service.py"


def main() -> None:
    tab_code = PATH_TAB.read_text(encoding="utf-8")
    service_code = PATH_SERVICE.read_text(encoding="utf-8")

    # 1) Extraer bloque de _synthesize_hinf_controller_impl(self) de HInfTab
    m = re.search(r"\n([ \t]*)def _synthesize_hinf_controller_impl\(self\):\n", tab_code)
    if not m:
        raise SystemExit("_synthesize_hinf_controller_impl no encontrado en hinf_tab.py")

    indent = m.group(1)
    start = m.start(0) + 1  # después del '\n' previo al def

    # Buscar el siguiente def con la misma indentación (fin del método)
    rest = tab_code[m.end():]
    next_def = re.search("\n" + re.escape(indent) + r"def ", rest)
    if next_def:
        end = m.end() + next_def.start()
    else:
        end = len(tab_code)

    block = tab_code[start:end]

    # 2) Quitar el bloque del archivo de la tab
    new_tab_code = tab_code[:start] + tab_code[end:]

    # 3) Dedentar el bloque para llevarlo a nivel de módulo
    lines = block.splitlines()
    dedented_lines = []
    for line in lines:
        if line.startswith(indent):
            dedented_lines.append(line[len(indent):])
        else:
            dedented_lines.append(line)
    block_dedented = "\n".join(dedented_lines)

    # 4) Cambiar cabecera a función de servicio y añadir alias self=tab
    old_header = "def _synthesize_hinf_controller_impl(self):"
    if old_header not in block_dedented:
        raise SystemExit("Cabecera de _synthesize_hinf_controller_impl no encontrada tras dedent")

    new_header = "def synthesize_hinf_controller(tab):\n        self = tab"
    block_service = block_dedented.replace(old_header, new_header, 1)

    # 5) Asegurar import de traceback en el servicio
    if "import traceback" not in service_code:
        service_code = service_code.replace("import logging", "import logging\nimport traceback", 1)

    # 6) Eliminar stub previo de synthesize_hinf_controller(tab) en el servicio (si existe)
    stub_pattern = r"\ndef synthesize_hinf_controller\(tab\):\n(?:[ \t].*\n)*?"  # def + cuerpo indentado
    service_code_clean = re.sub(stub_pattern, "\n", service_code, count=1)

    # 7) Añadir la función completa al final del service
    if not service_code_clean.endswith("\n"):
        service_code_clean += "\n"
    service_code_new = service_code_clean + "\n" + block_service + "\n"

    # 8) Guardar archivos modificados
    PATH_TAB.write_text(new_tab_code, encoding="utf-8")
    PATH_SERVICE.write_text(service_code_new, encoding="utf-8")


if __name__ == "__main__":
    main()
