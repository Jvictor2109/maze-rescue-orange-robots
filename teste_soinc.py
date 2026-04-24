from serial_comm import SerialComm
import argparse
import time

serial = SerialComm(
    port="/dev/ttyS0",
    baudrate=115200,
)
while True:
    response = serial.send("SR")

    try:
        parts = response.split(",")
        left_dist = float(parts[0].strip())
        front_dist = float(parts[1].strip())
        right_dist = float(parts[2].strip())
    except (ValueError, IndexError):
        print(f"[ERRO] Resposta invalida dos sensores: '{response}'")
        print("       Formato esperado: 15,45,12 (esq,frente,dir em cm)")

    print(f"Frente: {front_dist}")
    print(f"Direita: {right_dist}")
    print(f"Esquerda: {left_dist}")
    print()
    time.sleep(3)