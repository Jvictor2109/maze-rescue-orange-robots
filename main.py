"""
Main - Robo Maze Rescue
Integra navegacao DFS, deteccao de vitimas (cor + letra), e comunicacao serial.

Uso:
  Simulacao (sem hardware):  python main.py --simulate --no-camera
  Simulacao (com camera):    python main.py --simulate
  Real:                      python main.py --port COM3
"""

import argparse
import sys
import time

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

DIRECTION_NAME = {NORTH: "Norte", EAST: "Leste", SOUTH: "Sul", WEST: "Oeste"}
DIRECTION_DELTA = {
    NORTH: (0, 1),
    EAST:  (1, 0),
    SOUTH: (0, -1),
    WEST:  (-1, 0),
}

# Inverso: dado um delta, qual a direcao absoluta?
DELTA_TO_DIR = {v: k for k, v in DIRECTION_DELTA.items()}


# =====================================================================
# FUNCOES DE HEADING
# =====================================================================

def relative_to_absolute(heading, relative):
    """
    Converte direcao relativa ao robo para direcao absoluta no mapa.
    
    Args:
        heading: Orientacao atual (NORTH/EAST/SOUTH/WEST)
        relative: 'front', 'left', 'right'
    """
    offsets = {"front": 0, "left": -1, "right": 1, "back": 2}
    return (heading + offsets[relative]) % 4


def calculate_turn(current_heading, target_direction):
    """
    Calcula a sequencia de TURNs necessaria para virar de current_heading para target_direction.
    
    Returns:
        Lista de comandos ('TURN LEFT' ou 'TURN RIGHT')
        Novo heading apos os turns
    """
    diff = (target_direction - current_heading) % 4

    if diff == 0:
        return [], current_heading
    elif diff == 1:
        return ["TURN RIGHT"], target_direction
    elif diff == 2:
        # Meia-volta: 2x RIGHT e mais simples que 2x LEFT
        return ["TURN RIGHT", "TURN RIGHT"], target_direction
    elif diff == 3:
        return ["TURN LEFT"], target_direction

    return [], current_heading


# =====================================================================
# SENSORIAMENTO
# =====================================================================

def read_walls(heading, serial):
    """
    Le os sensores e mapeia para direcoes absolutas.
    
    Envia SENSOR ALL -> recebe 'frente,esquerda,direita' (0=livre, 1=parede)
    Converte para dicionario {direcao_absoluta: bool_tem_parede}
    
    Returns:
        dict com 4 direcoes: {NORTH: True/False, EAST: True/False, ...}
    """
    response = serial.send("SENSOR ALL")

    try:
        parts = response.split(",")
        front_wall = int(parts[0].strip()) == 1
        left_wall = int(parts[1].strip()) == 1
        right_wall = int(parts[2].strip()) == 1
    except (ValueError, IndexError):
        print(f"[ERRO] Resposta invalida dos sensores: '{response}'")
        print("       Formato esperado: 0,1,0 (frente,esq,dir)")
        return None

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
    """
    Verifica vitimas nos 3 lados (frente, esquerda, direita) usando o servo da camera.
    
    Returns:
        Lista de vitimas encontradas: [('cor', 'VERMELHO', 2), ('letra', 'H'), ...]
    """
    victims = []
    servo_positions = [
        ("SERVO CENTER", "frente"),
        ("SERVO LEFT", "esquerda"),
        ("SERVO RIGHT", "direita"),
    ]

    for servo_cmd, lado in servo_positions:
        serial.send(servo_cmd)
        time.sleep(0.3)  # Espera o servo estabilizar

        frame = camera.capture_array()

        # Deteccao de cor
        cor, kits = color_detector.processar_frame(frame)
        if cor:
            print(f"  [COR] Vitima de COR detectada ({lado}): {cor} - Kits: {kits}")
            victims.append(("cor", cor, kits))

        # Deteccao de letra
        letra = letter_detector.detect(frame)
        if letra:
            print(f"  [LETRA] Vitima de LETRA detectada ({lado}): {letra}")
            victims.append(("letra", letra))

    # Recentra o servo
    serial.send("SERVO CENTER")

    return victims


# =====================================================================
# NAVEGACAO + MOVIMENTACAO
# =====================================================================

def move_to_direction(heading, target_dir, serial):
    """
    Vira o robo para a direcao target_dir e avanca uma celula.
    
    Args:
        heading: Heading atual
        target_dir: Direcao absoluta desejada (NORTH/EAST/SOUTH/WEST)
        serial: Instancia de SerialComm
    
    Returns:
        Novo heading apos o movimento
    """
    # Calcula e executa turns
    turns, new_heading = calculate_turn(heading, target_dir)
    for turn_cmd in turns:
        serial.send(turn_cmd)

    # Avanca
    serial.send("MOVE FORWARD")

    return new_heading


def direction_between(from_pos, to_pos):
    """
    Calcula a direcao absoluta para ir de from_pos para to_pos (celulas adjacentes).
    
    Returns:
        Direcao absoluta (NORTH/EAST/SOUTH/WEST)
    """
    dx = to_pos[0] - from_pos[0]
    dy = to_pos[1] - from_pos[1]
    return DELTA_TO_DIR[(dx, dy)]


# =====================================================================
# LOOP PRINCIPAL - DFS ITERATIVO
# =====================================================================

def explorar_labirinto(serial, camera=None, color_detector=None, letter_detector=None, use_camera=True):
    """
    Exploracao DFS iterativa do labirinto com robo real/simulado.
    
    Args:
        serial: Instancia de SerialComm
        camera: Instancia de Picamera2 (ou None se --no-camera)
        color_detector: Instancia de DetectorVitimas
        letter_detector: Instancia de LetterDetector
        use_camera: Se False, pula verificacao de vitimas
    """

    # Estado do robo
    heading = NORTH  # Comeca virado para Norte
    pilha_caminho = [(0, 0)]
    visitados = set()
    visitados.add((0, 0))
    memoria_mapa = {}  # {(x,y): [(dir_absoluta, dx, dy), ...]}
    vitimas_encontradas = []  # Registo global de vitimas

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

            if (prox_x, prox_y) not in visitados:
                print(f"\n-> Avancando para {DIRECTION_NAME[direcao]} -> ({prox_x}, {prox_y})")

                # Move o robo fisicamente
                heading = move_to_direction(heading, direcao, serial)

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
                heading = move_to_direction(heading, target_dir, serial)

    # -------------------------------------------------
    # FINALIZACAO
    # -------------------------------------------------
    print("\n" + "=" * 50)
    print("[FIM] EXPLORACAO CONCLUIDA!")
    print(f"   Celulas mapeadas: {len(visitados)}")
    print(f"   Vitimas encontradas: {len(vitimas_encontradas)}")

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
