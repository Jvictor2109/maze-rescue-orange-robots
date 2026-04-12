def main():
    # INICIALIZAÇÃO DAS VARIÁVEIS

    mapa = {}
    visitados = ()
    caminho = ()
    explorar = ()
    pos_atual = (0,0)

    # VARIÁVEIS DE SUPORTE PRA MOVIMENTAÇÃO
    DC = [+1, 0, -1, 0]
    DL = [0, +1, 0, -1]

    #  COMEÇO DO ALGORITMO
    print(f"Posição atual: {pos_atual}")


    paredes = verificar_paredes()
    mapa[pos_atual] = {"paredes":paredes}

    #  VERIFICA SE TEM PAREDES EM CADA LADO E ADICIONA A EXPLORAR
    for p in paredes:
        print(p)
    if paredes[0] == 1:
        vizinho = (pos_atual[0], pos_atual[1]+DC[0])
        print(vizinho)






#  FUNÇÕES
def verificar_paredes():
    paredes = input("Há paredes? (Colocar 0 ou 1, ordem N-E-S-W): ")
    map(int(), paredes.split())
    return paredes










main()