import sys
import time
import math
from imu import IMU

sys.stdout.reconfigure(encoding='utf-8')

def main():
    print("Iniciando teste da IMU...")
    try:
        # Inicializa a IMU com as mesmas calibrações usadas no main.py
        imu = IMU(
            mag_offset=(-7.3500, 4.5750),
            mag_scale=(0.9841, 1.0164)
        )
    except Exception as e:
        print(f"Erro ao inicializar a IMU: {e}")
        return

    direcoes = {0: "Norte", 1: "Leste", 2: "Sul", 3: "Oeste"}

    while True:
        try:
            gx, gy, gz = imu.get_gyro()
            print(f"Gyro - x:{gx:.2f}, y:{gy:.2f}, z:{gz:.2f}")

            mag = imu.get_mag()
            if mag:
                mx, my, mz = mag
                print(f"Mag - mx:{mx:.2f}, my:{my:.2f}, mz:{mz:.2f}")
                
            heading, pos = imu.get_heading()
            if heading is not None and pos is not None:
                nome_pos = direcoes.get(pos, "Desconhecido")
                print(f"Heading: {heading:.2f}º")
                print(f"Posição: {nome_pos} (Código: {pos})")

            ax, ay, az = imu.get_accel()
            az_clamped = max(-1.0, min(1.0, az))
            inclinacao = math.acos(az_clamped) * 180 / math.pi
            print(f"Acelerómetro: AX: {ax:.3f}, AY: {ay:.3f}, AZ: {az:.3f}")
            print(f"Inclinação: {inclinacao:.2f}º")
            print("-" * 30)

            time.sleep(1)

        except KeyboardInterrupt:
            print("\nTeste interrompido.")
            break
        except Exception as e:
            print(f"Erro durante a leitura: {e}")
            time.sleep(1)

if __name__ == "__main__":
    main()