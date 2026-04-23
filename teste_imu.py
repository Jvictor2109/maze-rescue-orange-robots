import sys
import smbus2
import time
import math

sys.stdout.reconfigure(encoding='utf-8')

bus = smbus2.SMBus(1)

ICM20600_ADDR = 0x69
AK09918_ADDR  = 0x0C

# Acorda ICM20600
bus.write_byte_data(ICM20600_ADDR, 0x6B, 0x00)
time.sleep(0.1)

# AK09918 modo contínuo 100Hz
bus.write_byte_data(AK09918_ADDR, 0x31, 0x08)
time.sleep(0.1)

def read_word_2c(addr, reg):
    high = bus.read_byte_data(addr, reg)
    low  = bus.read_byte_data(addr, reg + 1)
    val  = (high << 8) + low
    return val - 65536 if val >= 0x8000 else val

def get_gyro():
    gx = read_word_2c(ICM20600_ADDR, 0x43)
    gy = read_word_2c(ICM20600_ADDR, 0x45)
    gz = read_word_2c(ICM20600_ADDR, 0x47)
    scale = 131.0
    return gx/scale, gy/scale, gz/scale

def get_mag():
    # Verifica se dados estão prontos (DRDY bit)
    status = bus.read_byte_data(AK09918_ADDR, 0x10)
    if not (status & 0x01):
        return None
    data = bus.read_i2c_block_data(AK09918_ADDR, 0x11, 6)
    # Lê ST2 para libertar o buffer
    bus.read_byte_data(AK09918_ADDR, 0x18)
    mx = (data[1] << 8) | data[0]
    my = (data[3] << 8) | data[2]
    mz = (data[5] << 8) | data[4]
    if mx >= 0x8000: mx -= 65536
    if my >= 0x8000: my -= 65536
    if mz >= 0x8000: mz -= 65536
    return mx * 0.15, my * 0.15, mz * 0.15

def get_heading(mx, my):
    heading = math.atan2(my, mx) * 180 / math.pi
    if heading < 0:
        heading += 360
    return heading

# Integração do giroscópio para ângulo acumulado
angle_z = 0.0
last_time = time.time()

while True:
    now = time.time()
    dt = now - last_time
    last_time = now

    gx, gy, gz = get_gyro()
    angle_z += gz * dt  # integra gz para obter ângulo acumulado

    mag = get_mag()
    if mag:
        mx, my, mz = mag
        heading = get_heading(mx, my)
        print(f"Gyro (°/s):     x={gx:.2f} y={gy:.2f} z={gz:.2f}")
        print(f"Ângulo Z (°):   {angle_z:.2f}")
        print(f"Heading (°):    {heading:.1f}")
        print("---")

    time.sleep(0.05)