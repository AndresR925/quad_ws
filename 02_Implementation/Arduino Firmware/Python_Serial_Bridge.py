import serial
import time
from datetime import datetime

PUERTO = "COM3"   # ⚠️ CAMBIA ESTO
BAUD = 115200

ser = serial.Serial(PUERTO, BAUD, timeout=1)
time.sleep(2)

print("Conectado al Arduino")

# archivo
nombre = f"log_{datetime.now().strftime('%H%M%S')}.csv"
f = open(nombre, "w")

print("Guardando en:", nombre)

def capturar():
    print("\nCapturando... (Ctrl+C para parar)\n")

    try:
        while True:
            linea = ser.readline()

            if not linea:
                continue

            try:
                texto = linea.decode(errors="ignore").strip()
            except:
                continue

            if texto == "":
                continue

            print(texto)

            # 🔥 GUARDAR TODO (sin filtrar)
            f.write(texto + "\n")
            f.flush()

    except KeyboardInterrupt:
        print("\nDetenido")

while True:
    cmd = input("\n[w=run | n=neutro | q=salir] >> ").lower()

    if cmd == "w":
        ser.reset_input_buffer()
        ser.write(b"w")
        capturar()

    elif cmd == "n":
        ser.write(b"n")

    elif cmd == "q":
        break

f.close()
ser.close()
print("Archivo guardado")