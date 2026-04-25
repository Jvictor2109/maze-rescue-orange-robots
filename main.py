
from cv2 import detail
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
WALL_THRESHOLD_CM = 15.0      # Distancia <= threshold = parede (ultrassonico)
MOTOR_SPEED = 35        # Velocidade padrao dos motores
DR_POLL_INTERVAL = 0.01       # 10ms entre leituras de encoder para maior precisão

# Convencao standard: N=+Y, E=+X, S=-Y, W=-X
DIRECTION_NAME = {NORTH: "Norte", EAST: "Leste", SOUTH: "Sul", WEST: "Oeste"}
DIRECTION_DELTA = {
    NORTH: (0, 1),
    EAST:  (1, 0),
    SOUTH: (0, -1),
    WEST:  (-1, 0),
}

# Inverso: dado um delta, qual a direcao absoluta
DELTA_TO_DIR = {v: k for k, v in DIRECTION_DELTA.items()}

# Angulo alvo (graus) para cada cardinal, apos maze_north_offset aplicado
DIRECTION_ANGLE = {NORTH: 0.0, EAST: 90.0, SOUTH: 180.0, WEST: 270.0}

# Constantes de controlo de rotacao
TURN_TOLERANCE = 5         # graus — para motores quando dentro desta margem
TURN_SLOW_ZONE = 30.0         # graus — reduz velocidade quando proximo do alvo
TURN_SPEED_FAST = 30
TURN_SPEED_SLOW = 30
TURN_TIMEOUT = 10.0           # segundos

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

    # for servo_cmd, lado in servo_positions:
    #     # Envia comando de servo e espera OK (confirma que o servo posicionou)
    #     serial.send(servo_cmd)
    #     time.sleep(1)
    #     # Captura um unico frame apos confirmacao
    #     frame = camera.capture_array()
    #     cv2.imwrite("/home/litch/debug_frame.jpg", frame)

    #     # Deteccao de cor
    #     cor, kits = color_detector.processar_frame(frame)
    #     if cor:
    #         print(f"  [COR] Vitima de COR detectada ({lado}): {cor} - Kits: {kits}")
    #         serial.send(f"VICTIM COLOR {cor}")
    #         victims.append(("cor", cor, kits))


    #     # Deteccao de letra
    #     letra = letter_detector.detect(frame)
    #     if letra:
    #         print(f"  [LETRA] Vitima de LETRA detectada ({lado}): {letra}")
    #         serial.send(f"VICTIM LETTER {letra}")
    #         victims.append(("letra", letra))

    # # Recentra o servo
    # serial.send("SERVO CENTER")

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
        # Verifica se esta numa rampa
        inclination = imu.get_inclination()
        print(f"Inclinção: {inclination}")
        # Assumindo 178 como normal e ~170-174 como inclinado. Se for menor ou igual a 175, detecta rampa.
        # Caso o eixo de montagem esteja invertido, pode ser necessario ajustar para o eixo X na classe IMU.
        if inclination is not None and inclination <= 174.5:
            print(f"[RAMPA] Inclinação detectada: {inclination:.1f}º. Iniciando subida!")
            ramp_speed = 60 # Aumenta a velocidade para vencer a rampa
            serial.send(f"MC {ramp_speed} {ramp_speed} {ramp_speed} {ramp_speed}")
            
            while True:
                inc_atual = imu.get_inclination()
                # Verifica se voltou ao normal (>= 177)
                if inc_atual is not None and inc_atual >= 178.0:
                    print(f"[RAMPA] Fim da rampa. Inclinação normalizada: {inc_atual:.1f}º. Parando.")
                    serial.send("MC 0 0 0 0")
                    return "OK" # Retorna imediatamente após a rampa
                time.sleep(0.02)
                
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
    print(f"Começo: {baseline}")
    print(f"Encoders: {deltas}")
    # 3. Para motores
    serial.send("MC 0 0 0 0")

    # Placeholder: deteccao de tiles pretos/azuis pelo sensor de chao
    # TODO: implementar quando sensor de chao estiver ligado ao Raspberry Pi
    tile_response = "OK"

    return tile_response


def angle_diff(target, current):
    """Diferenca angular com sinal: positivo = virar direita, negativo = virar esquerda."""
    diff = (target - current + 180) % 360 - 180
    return diff


def turn_to(target_dir, serial):
    target_angle = DIRECTION_ANGLE[target_dir]

    start_heading, _ = imu.get_heading()
    if start_heading is None:
        print("[ERRO] IMU sem leitura antes de virar!")
        return

    diff = angle_diff(target_angle, start_heading)
    if abs(diff) <= TURN_TOLERANCE:
        return

    # Decide a direção de rotação UMA VEZ no início
    # Restauramos o comando original: assumimos que MC -speed speed vira à DIREITA
    turn_right = diff > 0
    speed = TURN_SPEED_FAST
    
    start = time.time()
    
    if turn_right:
        serial.send(f"MC -{speed} {speed} -{speed} {speed}")  # Virar direita
    else:
        serial.send(f"MC {speed} -{speed} {speed} -{speed}")  # Virar esquerda

    while True:
        heading_deg, _ = imu.get_heading()
        if heading_deg is None:
            time.sleep(0.02)
            continue
            
        current_diff = angle_diff(target_angle, heading_deg)
        
        # Condição de paragem 1: chegou ao alvo dentro da tolerância
        if abs(current_diff) <= TURN_TOLERANCE:
            break
            
        # Condição de paragem 2: o erro mudou de sinal, ou seja, já passámos do alvo (evita oscilação infinita)
        if turn_right and current_diff < 0:
            break
        if not turn_right and current_diff > 0:
            break

        if time.time() - start > TURN_TIMEOUT:
            print(f"[ERRO] Timeout no turn_to! diff={current_diff:.1f}deg")
            break

        time.sleep(0.02)

    # Pára os motores
    serial.send("MC 0 0 0 0")


def move_to_direction(target_dir, serial):
    # Lê a direção atual real
    _, actual_cardinal = imu.get_heading()
    if actual_cardinal is None:
        actual_cardinal = target_dir  # fallback

    # Apenas roda se a direção cardinal alvo for diferente da atual
    if actual_cardinal != target_dir:
        turn_to(target_dir, serial)
    else:
        print(f"  [INFO] Já virado para {DIRECTION_NAME[target_dir]}. Seguindo em frente.")

    # Avanca uma celula
    response = move_forward(serial)

    # Retorna o heading REAL atualizado apos movimento
    _, actual_cardinal = imu.get_heading()
    if actual_cardinal is None:
        actual_cardinal = target_dir
    return actual_cardinal, response


def direction_between(from_pos, to_pos):
    dx = to_pos[0] - from_pos[0]
    dy = to_pos[1] - from_pos[1]
    return DELTA_TO_DIR[(dx, dy)]


# =====================================================================
# LOOP PRINCIPAL - DFS ITERATIVO
# =====================================================================

def explorar_labirinto(serial, camera=None, color_detector=None, letter_detector=None, use_camera=True):
    # Auto-calibra o heading inicial para que a direcao atual seja sempre o Norte (0) do labirinto
    if not imu.calibrate_north():
        print("[AVISO] Falha ao ler IMU no arranque. A assumir Norte como inicial.")
        heading = NORTH
    else:
        _, heading = imu.get_heading()
        if heading is None:
            heading = NORTH
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
            # Prioridade relativa: Frente, Direita, Esquerda, Tras
            for rel_dir in ["front", "right", "left", "back"]:
                d = relative_to_absolute(heading, rel_dir)
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
                # TODO: ativar quando sensor de chao estiver implementado
                if response == "BLACK":
                    print(f"  [BLACK] Tile preto em ({prox_x}, {prox_y})!")
                    tiles_bloqueados.add((prox_x, prox_y))
                    # Fisicamente voltar a celula anterior
                    back_dir = direction_between((prox_x, prox_y), (atual_x, atual_y))
                    heading, _ = move_to_direction(back_dir, serial)
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
                time.sleep(1)

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

    serial.send("MC 0 0 0 0")
    time.sleep(3)
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
