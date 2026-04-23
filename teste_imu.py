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

def get_accel():
    ax = read_word_2c(ICM20600_ADDR, 0x3B)
    ay = read_word_2c(ICM20600_ADDR, 0x3D)
    az = read_word_2c(ICM20600_ADDR, 0x3F)
    scale = 16384.0  # ±2g
    return ax/scale, ay/scale, az/scale

def get_mag():
    # Espera até dados estarem prontos (max 10 tentativas)
    for _ in range(10):
        status = bus.read_byte_data(AK09918_ADDR, 0x10)
        if status & 0x01:
            break
        time.sleep(0.01)
    else:
        return None  # timeout

    data = bus.read_i2c_block_data(AK09918_ADDR, 0x11, 6)
    bus.read_byte_data(AK09918_ADDR, 0x18)  # liberta buffer

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

    if heading >= 0 and heading < 90:
        pos = "N"  
    elif heading >= 90 and heading < 180:
        pos = "O"
    elif heading >= 180 and heading < 270:
        pos = "S"
    elif heading >= 270:
        pos = "E"

    
    return heading, pos

# Integração do giroscópio para ângulo acumulado
angle_z = 0.0
last_time = time.time()

while True:
    now = time.time()
    dt = now - last_time
    last_time = now

    gx, gy, gz = get_gyro()

    print(f"Gyro - x:{gx:.2f}, y:{gy:.2f}, z:{gz:.2f}")

    mag = get_mag()
    if mag:
        mx, my, mz = mag
        heading, pos = get_heading(mx, my)
        print(f"Mag - mx:{mx:.2f}, my:{my:.2f}: mz{mz:.2f}")
        print(f"Heading: {heading:.2f}")
        print(f"Posição: {pos}")

    ax, ay, az = get_accel()
    az_clamped = max(-1.0, min(1.0, az))
    inclinacao = math.acos(az_clamped) * 180 / math.pi
    print(f"Acelerómetro: AX: {ax}, AY: {ay} AZ: {az}")
    print(f"Inclinação: {inclinacao:.2f}º")
    print()

    time.sleep(3)