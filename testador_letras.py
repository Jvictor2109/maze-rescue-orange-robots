from letter_detector import LetterDetector
import cv2

frame = cv2.imread("C:/Users/Aluno.AEA/Documents/Joao/visao computacional/fotos/6.04.2026.png")

detector = LetterDetector(area_minima=300, min_confidence=0.1)

detector._generate_templates()

letra= detector.detect(frame)
if letra:
    print(f"Letra: {letra}")
else:
    print("Sem vitimas identificadas")

