import smbus2
import time

bus = smbus2.SMBus(1)

ICM20600_ADDR = 0x69
AK09918_ADDR  = 0x0C

# Acorda o ICM20600
bus.write_byte_data(ICM20600_ADDR, 0x6B, 0x00)

def read_word_2c(addr, reg):
    high = bus.read_byte_data(addr, reg)
    low  = bus.read_byte_data(addr, reg + 1)
    val  = (high << 8) + low
    return val - 65536 if val >= 0x8000 else val

def get_gyro():
    gx = read_word_2c(ICM20600_ADDR, 0x43)
    gy = read_word_2c(ICM20600_ADDR, 0x45)
    gz = read_word_2c(ICM20600_ADDR, 0x47)
    scale = 131.0  # ±250 dps
    return gx/scale, gy/scale, gz/scale

def get_mag():
    # Coloca AK09918 em single measurement mode
    bus.write_byte_data(AK09918_ADDR, 0x31, 0x01)
    time.sleep(0.01)
    data = bus.read_i2c_block_data(AK09918_ADDR, 0x11, 6)
    mx = (data[1] << 8) | data[0]
    my = (data[3] << 8) | data[2]
    mz = (data[5] << 8) | data[4]
    if mx >= 0x8000: mx -= 65536
    if my >= 0x8000: my -= 65536
    if mz >= 0x8000: mz -= 65536
    return mx * 0.15, my * 0.15, mz * 0.15  # µT

while True:
    gx, gy, gz = get_gyro()
    mx, my, mz = get_mag()
    print(f"Gyro (°/s): x={gx:.2f} y={gy:.2f} z={gz:.2f}")
    print(f"Mag  (µT):  x={mx:.2f} y={my:.2f} z={mz:.2f}")
    time.sleep(0.1)