import cv2
from teste2 import DetectorVitimas

frame = cv2.imread("C:/Users/Aluno.AEA/Documents/Joao/visao computacional/fotos/3.04.2026.png")

detector = DetectorVitimas()

nome, kits = detector.processar_frame(frame)
if nome:
    print(f"Cor: {nome}, Kits: {kits}")
else:
    print("Sem vitimas identificadas")

