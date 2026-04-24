import pigpio
import sensor_cor as tcs 
import time

pi = pigpio.pi()
s = tcs.sensor(pi, OUT=24, S2=22, S3=23, S0=4, S1=17, OE=18)
s.set_frequency(2)       # escala 20%
s.set_sample_size(20)

def identificar_cor(rgb):
    r, g, b = rgb
    if r > 150 and g < 80 and b < 80:
        return "vermelho"
    elif r < 80 and g > 150 and b < 80:
        return "verde"
    elif r < 80 and g < 80 and b > 150:
        return "azul"
    elif r > 200 and g > 200 and b > 200:
        return "branco"
    elif r < 50 and g < 50 and b < 50:
        return "preto"
    else:
        return "desconhecido"

while True:
    rgb = s.get_rgb()
    print(f"Cor: {identificar_cor(s.get_rgb())}")
    print(f"r: {rgb[0]}, G:{rgb[1]}, B:{rgb[2]}")