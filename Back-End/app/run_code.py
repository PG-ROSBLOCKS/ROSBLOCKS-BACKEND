import subprocess

def run_python_code(code: str):
    """
    Ejecuta el código Python recibido y devuelve la salida y errores.
    """
    try:
        result = subprocess.run(
            ["python3", "-c", code], 
            capture_output=True, text=True
        )
        return {
            "output": result.stdout,
            "error": result.stderr
        }
    except Exception as e:
        return {"error": str(e)}
