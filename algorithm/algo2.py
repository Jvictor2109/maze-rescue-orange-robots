from collections import deque

# ---------------------------------------------------------------------------
# Direções cardinais
# ---------------------------------------------------------------------------
DIRS   = {'N': (-1, 0), 'S': (1, 0), 'E': (0, 1), 'W': (0, -1)}
OPOSTO = {'N': 'S', 'S': 'N', 'E': 'W', 'W': 'E'}

# ---------------------------------------------------------------------------
# Estado global
# ---------------------------------------------------------------------------
mapa   = {}       # (row, col) -> "visited" | "unknown"
paredes = set()   # conjunto de ((row,col), direcao)

# ---------------------------------------------------------------------------
# Utilitários básicos
# ---------------------------------------------------------------------------

def vizinho(pos, d):
    dr, dc = DIRS[d]
    return (pos[0] + dr, pos[1] + dc)

def direcao_entre(a, b):
    """Calcula a direção cardinal de a para b (tiles adjacentes)."""
    dr, dc = b[0] - a[0], b[1] - a[1]
    for d, (r, c) in DIRS.items():
        if (r, c) == (dr, dc):
            return d
    raise ValueError(f"Tiles {a} e {b} não são adjacentes.")

# ---------------------------------------------------------------------------
# Simulação de sensores (substituir por código real no Webots)
# ---------------------------------------------------------------------------

def verificar_paredes(pos):
    """
    Pergunta ao utilizador as paredes em cada direção da posição atual.
    Em hardware real: substituir pelo sonar/IR.
    Não re-pergunta paredes já conhecidas.
    """
    print(f"\n  [SENSORES] Verificando paredes em {pos}:")
    for d in ['N', 'S', 'E', 'W']:
        # Parede já conhecida por este lado ou pelo lado oposto do vizinho
        if (pos, d) in paredes or (vizinho(pos, d), OPOSTO[d]) in paredes:
            estado = "bloqueado" if (pos, d) in paredes else "bloqueado"
            print(f"    {d}: já registado — parede presente")
            continue
        resp = input(f"    Há parede em {d}? (s/n): ").strip().lower()
        if resp == 's':
            paredes.add((pos, d))
            paredes.add((vizinho(pos, d), OPOSTO[d]))  # simetria: parede é partilhada

def mover(pos_atual, direcao):
    """
    Simula o movimento para o tile adjacente.
    Em hardware real: substituir por andar() + girar().
    """
    destino = vizinho(pos_atual, direcao)
    print(f"\n  [MOVER] {pos_atual} → {destino}  (direção: {direcao})")
    return destino

# ---------------------------------------------------------------------------
# Lógica de mapeamento
# ---------------------------------------------------------------------------

def vizinhos_unvisited(pos):
    """
    Retorna lista de (direcao, tile) para tiles adjacentes sem parede
    e ainda não visitados. Só chama isto DEPOIS de verificar_paredes(pos).
    """
    resultado = []
    for d in ['N', 'S', 'E', 'W']:
        if (pos, d) not in paredes:
            v = vizinho(pos, d)
            if mapa.get(v, 'unknown') == 'unknown':
                resultado.append((d, v))
    return resultado

# ---------------------------------------------------------------------------
# BFS — caminho mais curto entre dois tiles já visitados
# ---------------------------------------------------------------------------

def bfs_caminho(inicio, fim):
    """
    Encontra o caminho mais curto de inicio até fim,
    passando apenas por tiles já visitados e sem atravessar paredes.
    Retorna lista de tiles (excluindo inicio, incluindo fim), ou None.
    """
    if inicio == fim:
        return []
    fila = deque([(inicio, [])])
    visitados_bfs = {inicio}
    while fila:
        atual, caminho = fila.popleft()
        for d in ['N', 'S', 'E', 'W']:
            if (atual, d) in paredes:
                continue
            prox = vizinho(atual, d)
            if prox in visitados_bfs:
                continue
            if mapa.get(prox, 'unknown') == 'unknown':
                continue  # não atravessa território desconhecido
            novo_caminho = caminho + [prox]
            if prox == fim:
                return novo_caminho
            visitados_bfs.add(prox)
            fila.append((prox, novo_caminho))
    return None  # sem caminho encontrado

# ---------------------------------------------------------------------------
# Visualização do mapa no terminal
# ---------------------------------------------------------------------------

def imprimir_mapa(pos_atual=None):
    if not mapa:
        return
    rs = [p[0] for p in mapa]
    cs = [p[1] for p in mapa]
    print()
    for r in range(min(rs), max(rs) + 1):
        linha = ""
        for c in range(min(cs), max(cs) + 1):
            tile = (r, c)
            if tile == pos_atual:
                linha += " [R] "   # robô
            elif mapa.get(tile) == 'visited':
                linha += " [V] "   # visitado
            else:
                linha += " [ ] "   # desconhecido
        print(linha)
    print()

# ---------------------------------------------------------------------------
# ALGORITMO PRINCIPAL — DFS + BFS
# ---------------------------------------------------------------------------

def explorar():
    pos = (0, 0)
    mapa[pos] = 'visited'
    stack = []  # stack DFS explícita (caminho percorrido)

    print("=" * 60)
    print(f"  INÍCIO DA EXPLORAÇÃO — posição inicial: {pos}")
    print("  Legenda mapa: [R]=robô  [V]=visitado  [ ]=desconhecido")
    print("=" * 60)

    while True:
        print(f"\n{'─'*60}")
        print(f"  Posição atual : {pos}")
        print(f"  Stack DFS     : {stack}")

        # 1. Ler sensores — descobrir paredes nesta posição
        verificar_paredes(pos)

        # 2. Que vizinhos posso explorar daqui?
        unvisited = vizinhos_unvisited(pos)
        print(f"\n  [MAPA] Direções livres não visitadas: {[d for d,_ in unvisited]}")

        imprimir_mapa(pos)

        # ── CASO A: há tiles por explorar daqui ────────────────────────────
        if unvisited:
            d, destino = unvisited[0]
            stack.append(pos)
            pos = mover(pos, d)
            mapa[pos] = 'visited'

        # ── CASO B: dead end — procurar via BFS o backtrack correto ────────
        else:
            print("\n  [DFS] Dead end. A procurar próximo alvo na stack...")

            alvo = None
            idx  = len(stack) - 1

            # Percorre a stack do topo para a base
            while idx >= 0:
                candidato = stack[idx]
                # Paredes do candidato já são conhecidas (foi visitado antes)
                if vizinhos_unvisited(candidato):
                    alvo = candidato
                    break
                idx -= 1

            if alvo is None:
                print("\n" + "=" * 60)
                print("  EXPLORAÇÃO COMPLETA — sem mais tiles acessíveis.")
                print("=" * 60)
                break

            # Trunca a stack até antes do alvo
            # (alvo voltará a ser empilhado quando avançarmos a partir dele)
            stack = stack[:idx]

            print(f"  [BFS] Backtrack para {alvo}")
            caminho = bfs_caminho(pos, alvo)

            if caminho is None:
                print(f"  [ERRO] Não foi possível calcular caminho de {pos} para {alvo}.")
                break

            print(f"  [BFS] Caminho: {[pos] + caminho}")
            for tile in caminho:
                d = direcao_entre(pos, tile)
                pos = mover(pos, d)
                # Não altera mapa — tiles já visitados

    print("\n=== MAPA FINAL ===")
    imprimir_mapa()


# ---------------------------------------------------------------------------
if __name__ == '__main__':
    explorar()