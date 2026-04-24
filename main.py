
from multiprocessing import popen_fork
import argparse
import sys
import time
import cv2
from imu import IMU
from serial_comm import SerialComm
from color_victims.teste2 import DetectorVitimas
from letter_detector import LetterDetector


# =====================================================================
# CONSTANTES DE HEADING
# =====================================================================
NORTH = 0
EAST = 1
SOUTH = 2
WEST = 3

# =====================================================================
# CONSTANTES DE PROTOCOLO
# =====================================================================
CELL_DISTANCE_CM = 30.0       # Distancia de uma celula em cm (encoder)
WALL_THRESHOLD_CM = 5.0       # Distancia <= 5cm = parede (ultrassonico)
MOTOR_SPEED = 100              # Velocidade padrao dos motores
DR_POLL_INTERVAL = 0.05        # 50ms entre leituras de encoder

DIRECTION_NAME = {NORTH: "Norte", EAST: "Leste", SOUTH: "Sul", WEST: "Oeste"}
DIRECTION_DELTA = {
    NORTH: (0, 1),
    EAST:  (1, 0),
    SOUTH: (0, -1),
    WEST:  (-1, 0),
}

# Inverso: dado um delta, qual a direcao absoluta
DELTA_TO_DIR = {v: k for k, v in DIRECTION_DELTA.items()}

imu = IMU(
    mag_offset = (-7.3500, 4.5750),
    mag_scale = (0.9841, 1.0164)
)


# =====================================================================
# FUNCOES DE HEADING
# =====================================================================

def relative_to_absolute(heading, relative):
    offsets = {"front": 0, "left": -1, "right": 1, "back": 2}
    return (heading + offsets[relative]) % 4


# =====================================================================
# SENSORIAMENTO
# =====================================================================

def read_walls(heading, serial):
    response = serial.send("SR")

    try:
        parts = response.split(",")
        left_dist = float(parts[0].strip())
        front_dist = float(parts[1].strip())
        right_dist = float(parts[2].strip())
    except (ValueError, IndexError):
        print(f"[ERRO] Resposta invalida dos sensores: '{response}'")
        print("       Formato esperado: 15,45,12 (esq,frente,dir em cm)")
        return None

    left_wall = left_dist <= WALL_THRESHOLD_CM
    front_wall = front_dist <= WALL_THRESHOLD_CM
    right_wall = right_dist <= WALL_THRESHOLD_CM

    print(f"  [SA] Distancias: esq={left_dist:.1f}cm frente={front_dist:.1f}cm dir={right_dist:.1f}cm")

    walls = {}
    walls[relative_to_absolute(heading, "front")] = front_wall
    walls[relative_to_absolute(heading, "left")] = left_wall
    walls[relative_to_absolute(heading, "right")] = right_wall
    # Atras: assumimos livre (viemos de la)
    walls[relative_to_absolute(heading, "back")] = False

    return walls


# =====================================================================
# DETECCAO DE VITIMAS
# =====================================================================

def check_victims_in_cell(serial, camera, color_detector, letter_detector):
    victims = []
    servo_positions = [
        ("SERVO CENTER", "frente"),
        ("SERVO LEFT", "esquerda"),
        ("SERVO RIGHT", "direita"),
    ]

    for servo_cmd, lado in servo_positions:
        # Envia comando de servo e espera OK (confirma que o servo posicionou)
        serial.send(servo_cmd)
        time.sleep(1)
        # Captura um unico frame apos confirmacao
        frame = camera.capture_array()
        cv2.imwrite("/home/litch/debug_frame.jpg", frame)

        # Deteccao de cor
        cor, kits = color_detector.processar_frame(frame)
        if cor:
            print(f"  [COR] Vitima de COR detectada ({lado}): {cor} - Kits: {kits}")
            serial.send(f"VICTIM COLOR {cor}")
            victims.append(("cor", cor, kits))


        # Deteccao de letra
        letra = letter_detector.detect(frame)
        if letra:
            print(f"  [LETRA] Vitima de LETRA detectada ({lado}): {letra}")
            serial.send(f"VICTIM LETTER {letra}")
            victims.append(("letra", letra))

    # Recentra o servo
    serial.send("SERVO CENTER")

    return victims


# =====================================================================
# NAVEGACAO + MOVIMENTACAO
# =====================================================================

def move_forward(serial):
   
    speed = MOTOR_SPEED
    baseline_response = serial.send("MR")
    try:
        baseline = [float(v.strip()) for v in baseline_response.split(",")]
    except (ValueError, IndexError):
        print(f"[ERRO] Leitura base do encoder invalida: '{baseline_response}'")
        baseline = [0.0, 0.0, 0.0, 0.0]

    # Liga motores
    serial.send(f"MC {speed} {speed} {speed} {speed}")

    #  Poll encoders ate atingir distancia de uma celula
    while True:
        response = serial.send("MR")
        try:
            values = [float(v.strip()) for v in response.split(",")]
            # Distancia percorrida = leitura atual - leitura base
            deltas = [v - b for v, b in zip(values, baseline)]
            avg_distance = sum(deltas) / len(deltas)
        except (ValueError, IndexError):
            print(f"[ERRO] Resposta MR invalida: '{response}'")
            avg_distance = 0.0

        if avg_distance >= CELL_DISTANCE_CM:
            break

        time.sleep(DR_POLL_INTERVAL)

    # 3. Para motores
    serial.send("MC 0 0 0 0")

    # Placeholder: deteccao de tiles pretos/azuis pelo sensor de chao
    # TODO: implementar quando sensor de chao estiver ligado ao Raspberry Pi
    tile_response = "OK"

    return tile_response


def turn_to(target_dir, serial):
    _, pos = imu.get_heading()

    if target_dir == pos:
        return

    diff = (target_dir - pos) % 4

    if diff == 1:
        serial.send("MC 30 -30 30 -30")
    elif diff == 2:
        serial.send("MC 30 -30 30 -30")
    elif diff == 3:
        serial.send("MC -30 30 -30 30")

    start = time.time()
    while pos != target_dir:
        if time.time() - start > 10.0:
            print("[ERRO] Timeout no turn_to!")
            break
        _, pos = imu.get_heading()
        time.sleep(0.02)

    serial.send("MC 0 0 0 0")



def move_to_direction(target_dir, serial):
    # Calcula e executa turns
    turn_to(target_dir, serial)
    # Avanca uma celula com controlo direto
    response = move_forward(serial)

    return target_dir, response


def direction_between(from_pos, to_pos):
    dx = to_pos[0] - from_pos[0]
    dy = to_pos[1] - from_pos[1]
    return DELTA_TO_DIR[(dx, dy)]


# =====================================================================
# LOOP PRINCIPAL - DFS ITERATIVO
# =====================================================================

def explorar_labirinto(serial, camera=None, color_detector=None, letter_detector=None, use_camera=True):
    # Estado do robo
    heading = EAST  # Comeca virado para Leste
    pilha_caminho = [(0, 0)]
    visitados = set()
    visitados.add((0, 0))
    memoria_mapa = {}  # {(x,y): [(dir_absoluta, dx, dy), ...]}
    vitimas_encontradas = []  # Registo global de vitimas
    tiles_bloqueados = set()  # Tiles pretos/intransponiveis (tratados como parede)
    tiles_azuis = []  # Posicoes de tiles azuis encontrados

    print("\n" + "=" * 50)
    print("[ROBO] MAZE RESCUE - Exploracao Iniciada")
    print(f"   Posicao: (0, 0) | Heading: {DIRECTION_NAME[heading]}")
    print("=" * 50)

    while len(pilha_caminho) > 0:

        atual_x, atual_y = pilha_caminho[-1]

        # -------------------------------------------------
        # FASE A: SENSORIAMENTO + MAPEAMENTO + VITIMAS
        # -------------------------------------------------
        if (atual_x, atual_y) not in memoria_mapa:

            print(f"\n{'-' * 40}")
            print(f"[POS] Celula ({atual_x}, {atual_y}) | Heading: {DIRECTION_NAME[heading]}")
            print(f"{'-' * 40}")

            # 1. Ler sensores
            walls = read_walls(heading, serial)
            if walls is None:
                print("[ERRO] Falha na leitura dos sensores. A abortar.")
                break

            # Mostra paredes detectadas
            for d in [NORTH, EAST, SOUTH, WEST]:
                status = "PAREDE" if walls[d] else "livre"
                print(f"  {DIRECTION_NAME[d]:5s}: {status}")

            # 2. Verificar vitimas (se camera ativa)
            if use_camera and camera is not None:
                victims = check_victims_in_cell(serial, camera, color_detector, letter_detector)
                for v in victims:
                    vitimas_encontradas.append(((atual_x, atual_y), v))
            
            # 3. Montar opcoes livres (direcoes absolutas sem parede e nao visitadas)
            opcoes_livres = []
            # Prioridade: Norte, Leste, Sul, Oeste
            for d in [NORTH, EAST, SOUTH, WEST]:
                if not walls[d]:
                    dx, dy = DIRECTION_DELTA[d]
                    opcoes_livres.append((d, dx, dy))

            memoria_mapa[(atual_x, atual_y)] = opcoes_livres

        # -------------------------------------------------
        # FASE B: DECISAO DE MOVIMENTO (DFS)
        # -------------------------------------------------
        moveu = False
        opcoes = memoria_mapa[(atual_x, atual_y)]

        while len(opcoes) > 0:
            direcao, dx, dy = opcoes.pop(0)  # Consome a primeira opcao

            prox_x = atual_x + dx
            prox_y = atual_y + dy

            # Verifica se o tile esta bloqueado (preto detectado anteriormente)
            if (prox_x, prox_y) in tiles_bloqueados:
                print(f"  [X] {DIRECTION_NAME[direcao]} -> ({prox_x}, {prox_y}) - BLOQUEADO (tile preto)")
                continue

            if (prox_x, prox_y) not in visitados:
                print(f"\n-> Avancando para {DIRECTION_NAME[direcao]} -> ({prox_x}, {prox_y})")

                # Move o robo fisicamente
                heading, response = move_to_direction(direcao, serial)

                # Verifica resposta do ESP32
                if response == "BLACK":
                    print(f"  [BLACK] Tile preto em ({prox_x}, {prox_y})! Robo retornou automaticamente.")
                    tiles_bloqueados.add((prox_x, prox_y))
                    # Robo ja voltou ao tile atual — heading mantem-se, posicao nao muda
                    continue

                elif response == "BLUE":
                    print(f"  [BLUE] Tile azul em ({prox_x}, {prox_y})! Info guardada.")
                    tiles_azuis.append((prox_x, prox_y))
                    # Robo avancou com sucesso — continua exploracao a partir daqui

                # OK — movimento bem-sucedido
                visitados.add((prox_x, prox_y))
                pilha_caminho.append((prox_x, prox_y))
                moveu = True
                break
            else:
                print(f"  [!] {DIRECTION_NAME[direcao]} -> ({prox_x}, {prox_y}) - ja visitado")

        # -------------------------------------------------
        # FASE C: BACKTRACKING
        # -------------------------------------------------
        if not moveu:
            pilha_caminho.pop()

            if len(pilha_caminho) > 0:
                anterior_x, anterior_y = pilha_caminho[-1]
                print(f"\n<- Backtrack para ({anterior_x}, {anterior_y})")

                # Calcula direcao para a celula anterior
                target_dir = direction_between(
                    (atual_x, atual_y),
                    (anterior_x, anterior_y)
                )

                # Move fisicamente
                heading, _ = move_to_direction(target_dir, serial)

    # -------------------------------------------------
    # FINALIZACAO
    # -------------------------------------------------
    print("\n" + "=" * 50)
    print("[FIM] EXPLORACAO CONCLUIDA!")
    print(f"   Celulas mapeadas: {len(visitados)}")
    print(f"   Tiles bloqueados (pretos): {len(tiles_bloqueados)}")
    print(f"   Tiles azuis: {len(tiles_azuis)}")
    print(f"   Vitimas encontradas: {len(vitimas_encontradas)}")

    if tiles_azuis:
        print("\n   Tiles azuis encontrados:")
        for pos in tiles_azuis:
            print(f"     ({pos[0]}, {pos[1]})")

    if vitimas_encontradas:
        print("\n   Registo de vitimas:")
        for pos, victim in vitimas_encontradas:
            if victim[0] == "cor":
                print(f"     ({pos[0]},{pos[1]}): COR {victim[1]} - {victim[2]} kits")
            else:
                print(f"     ({pos[0]},{pos[1]}): LETRA {victim[1]}")

    print("=" * 50)


# =====================================================================
# ENTRY POINT
# =====================================================================

def main():
    parser = argparse.ArgumentParser(description="Robo Maze Rescue")
    parser.add_argument("--simulate", action="store_true",
                        help="Modo simulacao (terminal em vez de serial)")
    parser.add_argument("--port", type=str, default=None,
                        help="Porta serial do ESP32 (ex: COM3, /dev/ttyUSB0)")
    parser.add_argument("--baudrate", type=int, default=115200,
                        help="Baudrate da comunicacao serial")
    parser.add_argument("--no-camera", action="store_true",
                        help="Desativa a camera (testa so navegacao)")
    args = parser.parse_args()

    # Validacao
    if not args.simulate and not args.port:
        print("ERRO: Especifique --port ou use --simulate")
        print("  Exemplo: python main.py --simulate --no-camera")
        print("  Exemplo: python main.py --port COM3")
        sys.exit(1)

    # Serial
    serial = SerialComm(
        port=args.port,
        baudrate=args.baudrate,
        simulate=args.simulate,
    )

    # Handshake — verifica conexão com ESP32 antes de tudo
    if not serial.ping():
        print("[FATAL] Não foi possível comunicar com o ESP32. A encerrar.")
        serial.close()
        sys.exit(1)

    # Camera e detectores
    camera = None
    color_detector = None
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
        except ImportError:
            print("[CAMERA] Picamera2 nao disponivel - camera desativada")
            use_camera = False
        except Exception as e:
            print(f"[CAMERA] Erro ao iniciar: {e} - camera desativada")
            use_camera = False

    if use_camera:
        color_detector = DetectorVitimas()
        letter_det = LetterDetector()

    # Executa exploracao
    try:
        explorar_labirinto(serial, camera, color_detector, letter_det, use_camera)
    except KeyboardInterrupt:
        print("\n\n[!] Exploracao interrompida pelo utilizador")
    finally:
        serial.close()
        if camera:
            camera.stop()
            print("[CAMERA] Camera parada")


if __name__ == "__main__":
    main()
