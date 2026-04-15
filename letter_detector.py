"""
Detector de letras H, S e U para vítimas do Maze Rescue.
Extraído e refatorado a partir do letter_v1.py.
Recebe um frame e retorna a letra detectada (ou None).
"""

import cv2


class LetterDetector:
    """Detecta letras H, S, U num frame de câmera."""

    def __init__(self, blocksize=81, c_thresh=20, area_minima=5000, solidez_minima=0.55):
        self.blocksize = blocksize
        self.c_thresh = c_thresh
        self.area_minima = area_minima
        self.solidez_minima = solidez_minima

    def detect(self, frame):
        """
        Analisa um frame e tenta detectar uma letra (H, S ou U).
        
        Args:
            frame: Imagem BGR capturada pela câmera
        
        Returns:
            String 'H', 'S', 'U' se detectada, ou None
        """
        # Pré-processamento
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (5, 5), 0)
        thresh = cv2.adaptiveThreshold(
            blur, 255,
            cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
            cv2.THRESH_BINARY_INV,
            self.blocksize,
            self.c_thresh
        )

        # Encontra contornos
        contornos, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        melhor_letra = None
        maior_area = 0

        for cnt in contornos:
            area = cv2.contourArea(cnt)

            if area > self.area_minima and area > maior_area:
                x, y, w, h = cv2.boundingRect(cnt)
                letra_recortada = thresh[y:y+h, x:x+w]

                # Calcula solidez
                hull = cv2.convexHull(cnt)
                hull_area = cv2.contourArea(hull)
                if hull_area == 0:
                    continue
                solidez = float(area) / hull_area

                # Lógica de decisão
                if solidez > self.solidez_minima:
                    melhor_letra = "S"
                else:
                    # Diferenciar H de U pelo centro da imagem
                    centro_x, centro_y = w // 2, h // 2
                    offset = int(min(w, h) * 0.1)

                    if offset == 0:
                        offset = 1

                    # Recorta o miolo da letra
                    miolo = letra_recortada[
                        centro_y - offset : centro_y + offset,
                        centro_x - offset : centro_x + offset
                    ]

                    if miolo.size > 0 and cv2.countNonZero(miolo) > 0:
                        melhor_letra = "H"
                    else:
                        melhor_letra = "U"

                maior_area = area

        return melhor_letra
