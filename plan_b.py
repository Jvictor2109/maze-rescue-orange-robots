"""
right_wall_follow.py — Plano B: Right-Hand Wall Following

Algoritmo de reserva para o caso de falha do DFS ou do servo.
A câmara está fixa virada para a frente do robô (sem servo).

Hardware usado:
  - SerialComm → ESP32  (SR, MC, MR, VICTIM LETTER)
  - IMU (magnetómetro + giroscópio) para controlo de heading
  - Picamera2 + LetterDetector para vítimas de letra
  (sensor de cor excluído — câmara parada, sem servo)

Lógica de prioridade (relativo ao robô):
  1. Sem parede à DIREITA → vira direita e avança
  2. Sem parede à FRENTE  → avança em frente
  3. Sem parede à ESQUERDA → vira esquerda e avança
  4. Tudo bloqueado (beco sem saída) → meia-volta e avança

Executar:
  python right_wall_follow.py --simulate --no-camera
  python right_wall_follow.py --port /dev/ttyUSB0
"""

import argparse
import sys
import time

from imu import IMU
from serial_comm import SerialComm
from letter_detector import LetterDetector


# =====================================================================
# CONSTANTES — manter sincronizadas com main.py
# =====================================================================

NORTH = 0
EAST  = 1
SOUTH = 2
WEST  = 3

DIRECTION_NAME = {
    NORTH: "Norte",
    EAST:  "Leste",
    SOUTH: "Sul",
    WEST:  "Oeste",
}

# Ângulo absoluto (graus) de cada cardinal após calibrate_north()
DIRECTION_ANGLE = {
    NORTH: 0.0,
    EAST:  90.0,
    SOUTH: 180.0,
    WEST:  270.0,
}

# Geometria da célula e sensoriamento
CELL_DISTANCE_CM  = 30.0    # distância de uma célula (encoder)
WALL_THRESHOLD_CM = 15.0    # distância <= threshold → parede detetada
MOTOR_SPEED       = 50      # velocidade base dos motores (0-100)
DR_POLL_INTERVAL  = 0.05    # intervalo entre leituras de encoder (s)

# Controlo de rotação (IMU)
TURN_TOLERANCE  = 10.0  # graus — para motores quando dentro desta margem
TURN_SLOW_ZONE  = 30.0  # graus — reduz velocidade quando próximo do alvo
TURN_SPEED_FAST = 30
TURN_SPEED_SLOW = 18
TURN_TIMEOUT    = 10.0  # segundos — proteção contra rotação infinita

# Proteção contra loop infinito
MAX_STEPS = 300

# Calibração da IMU — valores do main.py
IMU_CONFIG = dict(
    mag_offset=(-7.3500, 4.5750),
    mag_scale=(0.9841, 1.0164),
)


# =====================================================================
# UTILITÁRIO IMU
# =====================================================================

def angle_diff(target: float, current: float) -> float:
    """
    Diferença angular com sinal no intervalo [-180, 180].
    Positivo → virar direita; negativo → virar esquerda.
    """
    return (target - current + 180.0) % 360.0 - 180.0


# =====================================================================
# CONTROLO DE MOVIMENTO
# =====================================================================

def turn_to(target_cardinal: int, serial: SerialComm, imu: IMU) -> None:
    """
    Roda o robô para o cardinal absoluto indicado usando a IMU.
    Bloqueia até estar dentro de TURN_TOLERANCE graus ou TURN_TIMEOUT.
    """
    target_angle = DIRECTION_ANGLE[target_cardinal]
    start = time.time()

    while True:
        heading_deg, _ = imu.get_heading()

        if heading_deg is None:
            # IMU sem leitura transitória — aguarda e tenta novamente
            time.sleep(0.05)
            continue

        diff = angle_diff(target_angle, heading_deg)

        if abs(diff) <= TURN_TOLERANCE:
            break

        if time.time() - start > TURN_TIMEOUT:
            print(f"[AVISO] Timeout em turn_to! diff restante={diff:.1f}°")
            break

        speed = TURN_SPEED_SLOW if abs(diff) < TURN_SLOW_ZONE else TURN_SPEED_FAST

        if diff > 0:
            # Virar à direita
            serial.send(f"MC {speed} -{speed} {speed} -{speed}")
        else:
            # Virar à esquerda
            serial.send(f"MC -{speed} {speed} -{speed} {speed}")

        time.sleep(0.02)

    serial.send("MC 0 0 0 0")


def move_forward(serial: SerialComm) -> None:
    """
    Avança uma célula (CELL_DISTANCE_CM) usando os encoders.
    Lê baseline antes de ligar os motores para calcular delta relativo.
    """
    baseline_response = serial.send("MR")
    try:
        baseline = [float(v.strip()) for v in baseline_response.split(",")]
    except (ValueError, IndexError):
        print(f"[ERRO] Leitura base do encoder inválida: '{baseline_response}'")
        baseline = [0.0, 0.0, 0.0, 0.0]

    serial.send(f"MC {MOTOR_SPEED} {MOTOR_SPEED} {MOTOR_SPEED} {MOTOR_SPEED}")

    while True:
        response = serial.send("MR")
        try:
            values = [float(v.strip()) for v in response.split(",")]
            deltas = [v - b for v, b in zip(values, baseline)]
            avg_distance = sum(deltas) / len(deltas)
        except (ValueError, IndexError):
            print(f"[ERRO] Resposta MR inválida: '{response}'")
            avg_distance = 0.0

        if avg_distance >= CELL_DISTANCE_CM:
            break

        time.sleep(DR_POLL_INTERVAL)

    serial.send("MC 0 0 0 0")


# =====================================================================
# SENSORIAMENTO
# =====================================================================

def read_walls(serial: SerialComm) -> dict | None:
    """
    Lê os sensores ultrassónicos via comando SR.
    Devolve {'left': bool, 'front': bool, 'right': bool} ou None em erro.
    """
    response = serial.send("SR")
    try:
        parts = response.split(",")
        left_dist  = float(parts[0].strip())
        front_dist = float(parts[1].strip())
        right_dist = float(parts[2].strip())
    except (ValueError, IndexError):
        print(f"[ERRO] Resposta inválida de SR: '{response}'")
        print("       Formato esperado: 15,45,12  (esq,frente,dir em cm)")
        return None

    print(f"  [SR] esq={left_dist:.1f}cm  frente={front_dist:.1f}cm  dir={right_dist:.1f}cm")

    return {
        "left":  left_dist  <= WALL_THRESHOLD_CM,
        "front": front_dist <= WALL_THRESHOLD_CM,
        "right": right_dist <= WALL_THRESHOLD_CM,
    }


# =====================================================================
# DETECÇÃO DE VÍTIMAS
# =====================================================================

def check_victims(
    camera,
    letter_detector: LetterDetector,
    serial: SerialComm,
    step: int,
    victims_log: list,
) -> None:
    """
    Captura um frame com a câmara fixa (virada para a frente)
    e tenta detetar uma letra de vítima.
    Regista localmente e reporta ao ESP32 via serial.
    """
    if camera is None or letter_detector is None:
        return

    frame = camera.capture_array()
    letra = letter_detector.detect(frame)

    if letra:
        print(f"  [LETRA] Vítima detetada no passo {step}: {letra}")
        serial.send(f"VICTIM LETTER {letra}")
        victims_log.append((step, letra))


# =====================================================================
# RIGHT-HAND WALL FOLLOWING
# =====================================================================

def right_wall_follow(
    serial: SerialComm,
    imu: IMU,
    camera=None,
    letter_detector: LetterDetector = None,
    use_camera: bool = True,
) -> None:
    """
    Algoritmo principal de wall following pela parede direita.

    A câmara está sempre virada para a frente do robô.
    A deteção de vítimas ocorre após cada avanço (sem servo).
    Termina após MAX_STEPS ou interrupção pelo utilizador.
    """

    # --- Calibração de heading ---
    if not imu.calibrate_north():
        print("[AVISO] IMU sem leitura no arranque. A usar Norte como heading inicial.")
        heading = NORTH
    else:
        _, cardinal = imu.get_heading()
        heading = cardinal if cardinal is not None else NORTH

    print("\n" + "=" * 50)
    print("[ROBO] RIGHT-WALL FOLLOWER — Iniciado")
    print(f"   Heading inicial: {DIRECTION_NAME[heading]}")
    print(f"   Máx. passos: {MAX_STEPS}")
    print("=" * 50)

    victims_log: list[tuple[int, str]] = []
    step = 0

    while step < MAX_STEPS:
        step += 1
        print(f"\n{'─' * 40}")
        print(f"[PASSO {step}/{MAX_STEPS}]  Heading: {DIRECTION_NAME[heading]}")

        # --- Leitura de paredes ---
        walls = read_walls(serial)
        if walls is None:
            print("[ERRO] Falha na leitura dos sensores. A abortar.")
            break

        right_wall = walls["right"]
        front_wall = walls["front"]
        left_wall  = walls["left"]

        print(
            f"  Paredes — esq: {'SIM' if left_wall  else 'NÃO'} | "
            f"frente: {'SIM' if front_wall else 'NÃO'} | "
            f"dir: {'SIM' if right_wall else 'NÃO'}"
        )

        # --- Decisão de movimento (right-hand rule) ---

        if not right_wall:
            # 1. Caminho livre à direita → vira e avança
            heading = (heading + 1) % 4
            print(f"  -> [DIREITA] Novo heading: {DIRECTION_NAME[heading]}")
            turn_to(heading, serial, imu)
            move_forward(serial)

        elif not front_wall:
            # 2. Frente livre → avança (heading não muda)
            print(f"  -> [FRENTE]  Heading mantido: {DIRECTION_NAME[heading]}")
            move_forward(serial)

        elif not left_wall:
            # 3. Caminho livre à esquerda → vira e avança
            heading = (heading - 1) % 4
            print(f"  -> [ESQUERDA] Novo heading: {DIRECTION_NAME[heading]}")
            turn_to(heading, serial, imu)
            move_forward(serial)

        else:
            # 4. Beco sem saída → meia-volta e avança
            heading = (heading + 2) % 4
            print(f"  -> [MEIA-VOLTA] Novo heading: {DIRECTION_NAME[heading]}")
            turn_to(heading, serial, imu)
            move_forward(serial)

        # --- Confirmação de heading pela IMU após o movimento ---
        _, actual_cardinal = imu.get_heading()
        if actual_cardinal is not None and actual_cardinal != heading:
            print(
                f"  [IMU] Drift detetado: esperado {DIRECTION_NAME[heading]}, "
                f"lido {DIRECTION_NAME[actual_cardinal]} — a corrigir."
            )
            heading = actual_cardinal

        # --- Detecção de vítimas (câmara fixa à frente) ---
        if use_camera:
            check_victims(camera, letter_detector, serial, step, victims_log)

    # --- Relatório final ---
    print("\n" + "=" * 50)
    print("[FIM] RIGHT-WALL FOLLOWER — Concluído")
    print(f"   Passos executados  : {step}")
    print(f"   Vítimas encontradas: {len(victims_log)}")
    if victims_log:
        print("   Registo de vítimas:")
        for s, letra in victims_log:
            print(f"     Passo {s:>3d}: LETRA {letra}")
    print("=" * 50)


# =====================================================================
# ENTRY POINT
# =====================================================================

def main() -> None:
    parser = argparse.ArgumentParser(
        description="Plano B — Right-Hand Wall Follower"
    )
    parser.add_argument(
        "--simulate", action="store_true",
        help="Modo simulação (responde no terminal em vez de usar serial)",
    )
    parser.add_argument(
        "--port", type=str, default=None,
        help="Porta serial do ESP32  (ex: /dev/ttyUSB0, COM3)",
    )
    parser.add_argument(
        "--baudrate", type=int, default=115200,
        help="Baudrate da comunicação serial (default: 115200)",
    )
    parser.add_argument(
        "--no-camera", action="store_true",
        help="Desativa a câmara (testa só a navegação)",
    )
    args = parser.parse_args()

    if not args.simulate and not args.port:
        print("ERRO: Especifique --port ou use --simulate")
        print("  Exemplo: python right_wall_follow.py --simulate --no-camera")
        print("  Exemplo: python right_wall_follow.py --port /dev/ttyUSB0")
        sys.exit(1)

    # --- Serial ---
    serial = SerialComm(
        port=args.port,
        baudrate=args.baudrate,
        simulate=args.simulate,
    )

    # --- Handshake com ESP32 ---
    if not serial.ping():
        print("[FATAL] Sem comunicação com o ESP32. A encerrar.")
        serial.close()
        sys.exit(1)

    # --- IMU ---
    imu = IMU(**IMU_CONFIG)

    # --- Câmara e detetor de letras ---
    camera = None
    letter_det = None
    use_camera = not args.no_camera

    if use_camera:
        try:
            from picamera2 import Picamera2
            camera = Picamera2()
            camera.configure(camera.create_preview_configuration(
                main={"format": "RGB888", "size": (320, 240)}
            ))
            camera.start()
            print("[CAMERA] Picamera2 iniciada")
            letter_det = LetterDetector()
        except ImportError:
            print("[CAMERA] Picamera2 não disponível — câmara desativada")
            use_camera = False
        except Exception as e:
            print(f"[CAMERA] Erro ao iniciar: {e} — câmara desativada")
            use_camera = False

    # --- Execução ---
    try:
        right_wall_follow(serial, imu, camera, letter_det, use_camera)
    except KeyboardInterrupt:
        print("\n\n[!] Interrompido pelo utilizador")
    finally:
        # Garante que os motores param sempre, mesmo em crash
        serial.send("MC 0 0 0 0")
        serial.close()
        if camera is not None:
            camera.stop()
            print("[CAMERA] Câmara parada")


if __name__ == "__main__":
    main()