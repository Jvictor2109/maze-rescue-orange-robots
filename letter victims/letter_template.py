import cv2
import numpy as np
from picamera2 import Picamera2

# --- CONFIGURAÇÃO ---
TEMPLATE_DIR = "/home/litch/Desktop/maze-rescue-orange-robots/letter victims/templates/"   # pasta com H.png, S.png, U.png
ANGULOS = [0, 90, 180, 270]   # ângulos a testar (adiciona mais se necessário)
THRESHOLD_MATCH = 0.6         # confiança mínima (ajustar experimentalmente)
AREA_MINIMA = 3000            # ignora contornos pequenos (ruído)
BLOCKSIZE = 81
C_THRESH = 20


def carregar_templates(letras=["h", "s", "u"]):
    """Carrega e pré-processa os templates em escala de cinza."""
    templates = {}
    for letra in letras:
        img = cv2.imread(f"{TEMPLATE_DIR}{letra}.png", cv2.IMREAD_GRAYSCALE)
        if img is None:
            raise FileNotFoundError(f"Template não encontrado: {TEMPLATE_DIR}{letra}.png")
        templates[letra] = img
    return templates


def rodar_imagem(img, angulo):
    """Roda uma imagem em torno do seu centro."""
    h, w = img.shape
    centro = (w // 2, h // 2)
    M = cv2.getRotationMatrix2D(centro, angulo, 1.0)
    return cv2.warpAffine(img, M, (w, h))


def classificar_roi(roi_gray, templates):
    """
    Faz template matching da ROI contra todos os templates e ângulos.
    Retorna (letra, score) do melhor match, ou (None, 0) se abaixo do threshold.
    """
    melhor_letra = None
    melhor_score = 0.0

    roi_h, roi_w = roi_gray.shape

    for letra, template in templates.items():
        for angulo in ANGULOS:
            t = rodar_imagem(template, angulo)

            # Redimensiona o template para caber na ROI, mantendo proporção
            t_h, t_w = t.shape
            scale = min(roi_h / t_h, roi_w / t_w)
            novo_w = max(1, int(t_w * scale))
            novo_h = max(1, int(t_h * scale))
            t_resized = cv2.resize(t, (novo_w, novo_h))

            # Template maior que ROI após resize não é possível de comparar
            if t_resized.shape[0] > roi_h or t_resized.shape[1] > roi_w:
                continue
            print(f"Template '{letra}' rotacao {angulo}: {t_resized.shape}")
            resultado = cv2.matchTemplate(roi_gray, t_resized, cv2.TM_CCOEFF_NORMED)
            _, score, _, _ = cv2.minMaxLoc(resultado)

            if score > melhor_score:
                melhor_score = score
                melhor_letra = letra

    if melhor_score >= THRESHOLD_MATCH:
        return melhor_letra, melhor_score
    return None, melhor_score


# --- SETUP ---
templates = carregar_templates()

picam2 = Picamera2()
picam2.configure(picam2.create_preview_configuration(
    main={"format": "RGB888", "size": (320, 240)}
))
picam2.start()

print("Pressione 'q' para sair.")

while True:
    frame = picam2.capture_array()
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, (5, 5), 0)
    thresh = cv2.adaptiveThreshold(
        blur, 255,
        cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
        cv2.THRESH_BINARY_INV,
        BLOCKSIZE, C_THRESH
    )

    contornos, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    for cnt in contornos:
        if cv2.contourArea(cnt) < AREA_MINIMA:
            continue

        x, y, w, h = cv2.boundingRect(cnt)

        roi_gray = gray[y:y+h, x:x+w]  # matching em cinza, não em thresh
        print(f"ROI shape: {roi_gray.shape}")
        vitima, score = classificar_roi(roi_gray, templates)

        if vitima:
            cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)
            cv2.putText(frame, f"{vitima} ({score:.2f})", (x, y - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
            print(f"Vítima: {vitima} | Score: {score:.2f}")

    cv2.imshow("Frame", frame)
    cv2.imshow("Thresh", thresh)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

picam2.stop()
cv2.destroyAllWindows()
