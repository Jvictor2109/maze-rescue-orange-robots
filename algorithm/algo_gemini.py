# =======================================================
# SIMULADOR DE EXPLORAÇÃO DE LABIRINTO (DFS ITERATIVO)
# Focado em Robótica (Sem Recursão)
# =======================================================

def explorar_labirinto_robo_real():
    """
    Conceito: Usamos uma estrutura de dados explícita de Pilha (LIFO - Last In, First Out)
    para rastrear o caminho físico do robô. O topo da pilha é sempre onde o robô
    está fisicamente agora. Se ele ficar sem opções, removemos o topo da pilha
    e ordenamos aos motores que recuem fisicamente para a célula anterior.
    """
    
    # 1. VARIÁVEIS DE ESTADO DO ROBÔ
    # pilha_caminho atua como o rastreador de posição do robô no mundo.
    pilha_caminho = [(0, 0)] 
    
    # visitados impede que o robô ande em círculos (loop infinito)
    visitados = set()
    visitados.add((0, 0))
    
    # memoria_mapa atua como o mapeamento interno (SRAM do robô).
    # Salva os caminhos livres de cada célula para não usar os sensores 
    # de forma redundante quando fizer o caminho de volta (backtracking).
    memoria_mapa = {}

    print("Iniciando a exploração do labirinto (Modo Iterativo).")
    print("O robô parte da origem (0, 0).")

    # 2. LOOP PRINCIPAL DE CONTROLE
    # Continua rodando enquanto a pilha de caminho não estiver vazia.
    # Quando esvaziar, significa que ele testou tudo e retrocedeu até fora do mapa (0,0).
    while len(pilha_caminho) > 0:
        
        # Onde o robô está AGORA? (Sempre o último elemento da lista/topo da pilha)
        atual_x, atual_y = pilha_caminho[-1]
        
        # -------------------------------------------------------------
        # FASE A: SENSORIAMENTO E MAPEAMENTO
        # -------------------------------------------------------------
        # Se for a primeira vez que ele pisa nesta célula, usa os sensores.
        if (atual_x, atual_y) not in memoria_mapa:
            print("\n" + "="*40)
            print(f"📍 SENSORES ATIVOS: Lendo arredores da célula ({atual_x}, {atual_y})")
            print("="*40)
            
            parede_n = input("Parede ao Norte (Y+1)? [s/n]: ").strip().lower() == 's'
            parede_s = input("Parede ao Sul (Y-1)? [s/n]: ").strip().lower() == 's'
            parede_l = input("Parede ao Leste (X+1)? [s/n]: ").strip().lower() == 's'
            parede_o = input("Parede ao Oeste (X-1)? [s/n]: ").strip().lower() == 's'
            
            # Montamos uma lista das opções possíveis a partir desta célula
            # Prioridade de exploração: Norte, Sul, Leste, Oeste
            opcoes_livres = []
            if not parede_n: opcoes_livres.append((0, 1, "Norte"))
            if not parede_s: opcoes_livres.append((0, -1, "Sul"))
            if not parede_l: opcoes_livres.append((1, 0, "Leste"))
            if not parede_o: opcoes_livres.append((-1, 0, "Oeste"))
            
            # Gravamos na memória do robô as portas de saída desta coordenada
            memoria_mapa[(atual_x, atual_y)] = opcoes_livres

        # -------------------------------------------------------------
        # FASE B: TOMADA DE DECISÃO E MOVIMENTAÇÃO
        # -------------------------------------------------------------
        moveu_para_frente = False
        opcoes_da_celula = memoria_mapa[(atual_x, atual_y)]
        
        # Inspeciona as opções salvas na memória para esta célula
        while len(opcoes_da_celula) > 0:
            # Retira (consome) a primeira opção da lista.
            # Isso é vital: garante que o robô não tente o mesmo caminho duas vezes.
            dx, dy, direcao = opcoes_da_celula.pop(0)
            
            prox_x = atual_x + dx
            prox_y = atual_y + dy
            
            # Se a célula apontada por este caminho livre AINDA NÃO foi visitada...
            if (prox_x, prox_y) not in visitados:
                # O robô decide ir para lá!
                print(f"\n➔ COMANDO MOTOR: Avançando para o {direcao} até a célula ({prox_x}, {prox_y}).")
                
                # Marca como visitada
                visitados.add((prox_x, prox_y))
                
                # EMPILHA a nova célula. No próximo ciclo do 'while', esta célula
                # será lida como a posição atual.
                pilha_caminho.append((prox_x, prox_y))
                
                moveu_para_frente = True
                break # Quebra o 'while' de opções, pois o robô já escolheu para onde ir
            else:
                print(f"  [!] Ignorando {direcao} para ({prox_x}, {prox_y}) - Já mapeado no passado.")

        # -------------------------------------------------------------
        # FASE C: BACKTRACKING (BECO SEM SAÍDA)
        # -------------------------------------------------------------
        # Se 'moveu_para_frente' continuou False, significa que a lista de opções da 
        # célula esvaziou ou todas levavam a lugares já visitados. É um Beco Sem Saída!
        if not moveu_para_frente:
            
            # DESEMPILHA: Remove a célula atual do caminho do robô.
            pilha_caminho.pop()
            
            # Se ainda sobrou caminho na pilha, o robô deve fisicamente dar ré / girar
            # para voltar à célula anterior.
            if len(pilha_caminho) > 0:
                anterior_x, anterior_y = pilha_caminho[-1]
                print(f"\n🔙 COMANDO MOTOR: Beco sem saída. Retornando fisicamente (Backtracking) para ({anterior_x}, {anterior_y}).")
                
            # Se a pilha esvaziou, o 'while' principal será encerrado naturalmente.

    # 3. FINALIZAÇÃO
    print("\n" + "="*40)
    print("🏁 EXPLORAÇÃO CONCLUÍDA!")
    print("A pilha esvaziou. O robô explorou todos os caminhos acessíveis")
    print("e retornou em segurança para a origem em (0, 0).")
    print(f"Total de células descobertas e mapeadas: {len(visitados)}")
    print("="*40)

# ==========================================
# EXECUÇÃO DO PROGRAMA
# ==========================================
if __name__ == "__main__":
    explorar_labirinto_robo_real()