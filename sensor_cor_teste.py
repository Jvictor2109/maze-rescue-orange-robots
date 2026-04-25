import pigpio
import sensor_cor as tcs 
import time

pi = pigpio.pi()
s = tcs.sensor(pi, OUT=24, S2=22, S3=23, S0=4, S1=17, OE=18)
s.set_frequency(2)       # escala 20%
s.set_sample_size(20)


while True:
    cor = s.get_cor()
    print(f"Cor: {cor}")
    time.sleep(3)

    hertz = s.get_Hertz()
    print(f"Hertz: {hertz}")