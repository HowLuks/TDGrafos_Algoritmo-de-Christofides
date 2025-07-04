import networkx as nx

class Grafo:
    def __init__(self, num_vertices):
        self.num_vertices = num_vertices
        self.adj_matriz = [[0 for _ in range(num_vertices)] for _ in range(num_vertices)]

    def add_aresta(self, u, v, peso):
        self.adj_matriz[u][v] = peso
        self.adj_matriz[v][u] = peso  #grafo não direcionado

    def get_peso(self, u, v):
        return self.adj_matriz[u][v]

    def get_vizinhos(self, u):
        vizinhos = []
        for v in range(self.num_vertices):
            if self.adj_matriz[u][v] > 0:
                vizinhos.append(v)
        return vizinhos

def ler_arquivo(filepath):
    with open(filepath, 'r') as f:
        linhas = f.readlines()
    
    num_vertices = int(linhas[0].strip())
    grafo = Grafo(num_vertices)

    for i in range(1, num_vertices + 1):
        pesos = list(map(int, linhas[i].strip().split()))
        for j in range(num_vertices):
            grafo.adj_matriz[i-1][j] = pesos[j]

    return grafo

#algoritmo de Prim para arvore geradora minima
def prim_agm(grafo):
    num_vertices = grafo.num_vertices
    arv_grd_min = Grafo(num_vertices)
    
    key = [float('inf')] * num_vertices
    pai = [-1] * num_vertices
    in_agm = [False] * num_vertices

    key[0] = 0
    
    for _ in range(num_vertices):
        min_key = float('inf')
        indice_min = -1
        for v in range(num_vertices):
            if not in_agm[v] and key[v] < min_key:
                min_key = key[v]
                indice_min = v
        
        in_agm[indice_min] = True
        
        if pai[indice_min] != -1:
            arv_grd_min.add_aresta(pai[indice_min], indice_min, grafo.get_peso(pai[indice_min], indice_min))

        for v in range(num_vertices):
            peso = grafo.get_peso(indice_min, v)
            if peso > 0 and not in_agm[v] and peso < key[v]:
                key[v] = peso
                pai[v] = indice_min
                
    return arv_grd_min

def find_verticesGrauImpar(agm_grafo):
    verticesGrauImpar = []
    for i in range(agm_grafo.num_vertices):
        grau = 0
        for j in range(agm_grafo.num_vertices):
            if agm_grafo.adj_matriz[i][j] > 0:
                grau += 1
        if grau % 2 != 0:
            verticesGrauImpar.append(i)
    return verticesGrauImpar

def min_cost_perfect_emparelha(grafo, vert_impar):
    #cria um subgrafo com apenas os vértices de grau ímpar
    subgrafo = nx.Graph()
    for i in vert_impar:
        subgrafo.add_node(i)
    
    for i in range(len(vert_impar)):
        for j in range(i + 1, len(vert_impar)):
            u = vert_impar[i]
            v = vert_impar[j]
            peso = grafo.get_peso(u, v)
            if peso > 0:
                subgrafo.add_edge(u, v, peso=peso)
    
    #encontra o emparelhamento perfeito de custo mínimo
    emparelha = nx.algorithms.matching.min_weight_matching(subgrafo)
    
    emparelha_grafo = Grafo(grafo.num_vertices)
    for u, v in emparelha:
        emparelha_grafo.add_aresta(u, v, grafo.get_peso(u, v))
    
    return emparelha_grafo

def combina_grafos(agm_grafo, emparelha_grafo):
    multigrafo = Grafo(agm_grafo.num_vertices)
    for i in range(agm_grafo.num_vertices):
        for j in range(agm_grafo.num_vertices):
            multigrafo.adj_matriz[i][j] = agm_grafo.adj_matriz[i][j] + emparelha_grafo.adj_matriz[i][j]
    return multigrafo


#implementação do algoritmo de Hierhozer para encontrar ciclo euleriano
#adaptado de: https://www.geeksforgeeks.org/dsa/hierholzers-algorithm-directed-graph/
def find_circuitoEuleriano(multigrafo):
    num_vertices = multigrafo.num_vertices
    adj = [[] for _ in range(num_vertices)]
    for i in range(num_vertices):
        for j in range(num_vertices):
            for _ in range(multigrafo.adj_matriz[i][j]): # Adiciona arestas múltiplas
                adj[i].append(j)

    #encontra um vértice com arestas para começar
    atual_v = 0
    for i in range(num_vertices):
        if len(adj[i]) > 0:
            atual_v = i
            break

    #armazena o circuito euleriano
    circuito = []
    
    #pilha para busca em profundidade
    atual_cam = [atual_v]
    
    while atual_cam:
        atual_v = atual_cam[-1]
        
        if adj[atual_v]: #ve se o vértice atual ainda tem arestas
            prox_v = adj[atual_v].pop()
            #remove a aresta reversa também para grafos não direcionados
            adj[prox_v].remove(atual_v)
            atual_cam.append(prox_v)
        else:
            circuito.append(atual_cam.pop())
            
    return circuito[::-1] #inverte para obter a ordem correta

def atalho_circuitoEuleriano(circuitoEuleriano):
    circuito_hamiltoniano = []
    visitado = [False] * len(circuitoEuleriano)
    
    for vert in circuitoEuleriano:
        if not visitado[vert]:
            circuito_hamiltoniano.append(vert)
            visitado[vert] = True
            
    #adiciona o primeiro vértice para fechar o ciclo
    if circuito_hamiltoniano and circuito_hamiltoniano[0] != circuito_hamiltoniano[-1]:
        circuito_hamiltoniano.append(circuito_hamiltoniano[0])

    return circuito_hamiltoniano

def calcula_custo(circuito, grafo):
    custo = 0
    for i in range(len(circuito) - 1):
        custo += grafo.get_peso(circuito[i], circuito[i+1])
    return custo

def christofides(filepath):
    grafo = ler_arquivo(filepath)
    
    #construção de uma arvore geradora mínima ddo grafo
    agm_grafo = prim_agm(grafo)
    
    #identificação do conjunto de vértices de grau impar na arvore
    verticesGrauImpar = find_verticesGrauImpar(agm_grafo)
    
    #construção de um emparelhamento perfeito de custo mínimo no subgrafo induzido pelos vértices de impar da arvore
    emparelha_grafo = min_cost_perfect_emparelha(grafo, verticesGrauImpar)
    
    #união da arvore e do emparelhamento
    multigrafo = combina_grafos(agm_grafo, emparelha_grafo)
    
    #determinação de um ciclo euleriano na união
    circuitoEuleriano = find_circuitoEuleriano(multigrafo)
    
    #atalho do ciclo euleriano para pegar o hamiltoniano 
    circuito_hamiltoniano = atalho_circuitoEuleriano(circuitoEuleriano)
    
    print("ciclo hamiltoniano:", circuito_hamiltoniano)
    print("custo total:", calcula_custo(circuito_hamiltoniano, grafo))

if __name__ == "__main__":
    filepath = "exemplo_entrada.txt"
    christofides(filepath)
