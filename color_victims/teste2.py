import cv2
import numpy as np

class DetectorVitimas:
    def __init__(self):
        # Definição dos limites HSV (ajustar na calibração no dia)
        self.cores = {
            "VERMELHO": {
                "baixo1": np.array([0, 150, 70]), "alto1": np.array([10, 255, 255]),
                "baixo2": np.array([170, 150, 70]), "alto2": np.array([180, 255, 255]),
                "kits": 2 # Regra 5.6.5.b.i [cite: 631]
            },
            "AMARELO": {
                "baixo": np.array([20, 100, 100]), "alto": np.array([30, 255, 255]),
                "kits": 1 # Regra 5.6.5.b.ii [cite: 632]
            },
            "VERDE": {
                "baixo": np.array([40, 70, 70]), "alto": np.array([80, 255, 255]),
                "kits": 0 # Regra 5.6.5.b.iii [cite: 633]
            }
        }

    def processar_frame(self, frame):
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        for nome, limites in self.cores.items():
            if nome == "VERMELHO":
                mask = cv2.add(cv2.inRange(hsv, limites["baixo1"], limites["alto1"]),
                               cv2.inRange(hsv, limites["baixo2"], limites["alto2"]))
            else:
                mask = cv2.inRange(hsv, limites["baixo"], limites["alto"])

            # Filtro morfológico para remover ruído
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5,5), np.uint8))
            contornos, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            for c in contornos:
                area = cv2.contourArea(c)
                if area > 1000: # Ajustar conforme a distância da câmera
                    return nome, limites["kits"]
        return None, 0

    