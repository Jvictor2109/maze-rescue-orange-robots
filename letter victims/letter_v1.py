import cv2
from picamera2 import Picamera2

# Inicia a câmera e define a resolução de imagem
picam2 = Picamera2()
picam2.configure(picam2.create_preview_configuration(
    main={"format": "RGB888", "size": (320, 240)}
))
picam2.start()

print("Pressione q pra sair")

while True:
    # Captura a imagem
    frame = picam2.capture_array()
    
    # Colocar imagem em escala de cinza
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Borra a imagem
    blur = cv2.GaussianBlur(gray, (5,5), 0)

    # Threshold adaptiva
    thresh = cv2.adaptiveThreshold(blur, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY_INV, 51, 12)

    # Econtra os contornos da imagem
    contornos, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    for cnt in contornos:
        area = cv2.contourArea(cnt)
        
        # 1. Filtro de tamanho para ignorar sujeira
        if area > 2000: 
            x, y, w, h = cv2.boundingRect(cnt)
            
            # 2. Recorta a letra e calcula a Solidez
            letra_recortada = thresh[y:y+h, x:x+w]
            hull = cv2.convexHull(cnt)
            solidez = float(area) / cv2.contourArea(hull)

            # --- LÓGICA DE DECISÃO ---
            
            # O 'S' é o mais "sólido/preenchido" de todos
            if solidez > 0.63:
                vitima = "S"
                print(vitima)
            
            else:
                # 3. Diferenciar H de U pelo CENTRO da imagem
                # Criamos uma pequena área no centro exato (10% da letra)
                centro_x, centro_y = w // 2, h // 2
                offset = int(min(w, h) * 0.1) # Pequena margem de erro
                
                # Recorta o miolo da letra
                miolo = letra_recortada[centro_y-offset : centro_y+offset, 
                                        centro_x-offset : centro_x+offset]
                
                # Conta pixels brancos no miolo
                if miolo.size > 0 and cv2.countNonZero(miolo) > 0:
                    # Se tem branco no centro, é a barra horizontal do 'H'
                    vitima = "H"
                    print(vitima)
                else:
                    # Se o centro está vazio, é o buraco do 'U'
                    vitima = "U"
                    print(vitima)
            
            # --- EXIBIÇÃO ---
            cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)
            cv2.putText(frame, vitima, (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)
            print(f"Detectado: {vitima} | Solidez: {solidez:.2f}")


    # 6. Mostra as janelas na tela do Raspberry
    cv2.imshow("Original", frame)
    cv2.imshow("Filtro Binarizado", thresh)

    # 7. Fecha o programa se a tecla 'q' for pressionada
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break


