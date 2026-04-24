
import cv2
import numpy as np


class LetterDetector:

    TEMPLATE_SIZE = 80
    LETTERS = ["H", "S", "U"]

    def __init__(self, min_confidence=0.40, area_minima=2000):
        self.min_confidence = min_confidence
        self.area_minima = area_minima
        self.templates = self._generate_templates()

    # -----------------------------------------------------------------
    # Geração de templates
    # -----------------------------------------------------------------

    def _render_letter(self, letter, font, font_scale, thickness):
        size = self.TEMPLATE_SIZE
        img = np.zeros((size, size), dtype=np.uint8)

        (tw, th), _ = cv2.getTextSize(letter, font, font_scale, thickness)
        x = (size - tw) // 2
        y = (size + th) // 2

        cv2.putText(img, letter, (x, y), font, font_scale, 255, thickness)
        return img

    def _generate_templates(self):
        font_configs = [
            (cv2.FONT_HERSHEY_SIMPLEX, 2.5, 5),
            (cv2.FONT_HERSHEY_SIMPLEX, 2.5, 7),
            (cv2.FONT_HERSHEY_SIMPLEX, 2.5, 9),
            (cv2.FONT_HERSHEY_DUPLEX, 2.2, 5),
            (cv2.FONT_HERSHEY_DUPLEX, 2.2, 7),
            (cv2.FONT_HERSHEY_COMPLEX, 2.2, 5),
            (cv2.FONT_HERSHEY_COMPLEX, 2.2, 7),
            (cv2.FONT_HERSHEY_TRIPLEX, 2.0, 5),
        ]

        templates = {}
        for letter in self.LETTERS:
            letter_templates = []
            for font, scale, thick in font_configs:
                img = self._render_letter(letter, font, scale, thick)
                letter_templates.append(img)
            templates[letter] = letter_templates

        return templates

    # -----------------------------------------------------------------
    # Detecção
    # -----------------------------------------------------------------

    def detect(self, frame):
        # --- Pré-processamento ---
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (5, 5), 0)

        # Otsu: mais robusto que adaptativo para contraste claro preto/branco
        _, thresh = cv2.threshold(
            blur, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU
        )

        # Limpeza morfológica: fecha buracos pequenos, remove ruído
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
        thresh = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel, iterations=1)
        thresh = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel, iterations=1)

        # --- Encontra contornos ---
        contours, _ = cv2.findContours(
            thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )

        best_letter = None
        best_confidence = 0.0

        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < self.area_minima:
                continue

            x, y, w, h = cv2.boundingRect(cnt)

            # Filtro de aspect ratio — H, S, U são aproximadamente quadradas
            aspect = float(w) / h if h > 0 else 0
            if aspect < 0.35 or aspect > 2.5:
                continue

            # Extrai ROI com pequena margem e redimensiona ao tamanho do template
            pad = 5
            y1 = max(0, y - pad)
            y2 = min(thresh.shape[0], y + h + pad)
            x1 = max(0, x - pad)
            x2 = min(thresh.shape[1], x + w + pad)
            roi = thresh[y1:y2, x1:x2]

            if roi.size == 0:
                continue

            roi_resized = cv2.resize(
                roi,
                (self.TEMPLATE_SIZE, self.TEMPLATE_SIZE),
                interpolation=cv2.INTER_AREA,
            )

            # --- Template matching contra todas as variações ---
            for letter, letter_templates in self.templates.items():
                for template in letter_templates:
                    result = cv2.matchTemplate(
                        roi_resized, template, cv2.TM_CCOEFF_NORMED
                    )
                    score = result[0][0]

                    if score > self.min_confidence and score > best_confidence:
                        best_confidence = score
                        best_letter = letter

        return best_letter
