import smbus2
import time
import math


class IMU:
    ICM20600_ADDR = 0x69
    AK09918_ADDR  = 0x0C

    def __init__(self, bus_number=1,
                 mag_offset=(0.0, 0.0),
                 mag_scale=(1.0, 1.0)):
        self.bus = smbus2.SMBus(bus_number)

        # Calibração do magnetómetro
        self.mag_offset = mag_offset
        self.mag_scale = mag_scale

        # Acorda ICM20600
        self.bus.write_byte_data(self.ICM20600_ADDR, 0x6B, 0x00)
        time.sleep(0.1)

        # AK09918 modo contínuo 100Hz
        self.bus.write_byte_data(self.AK09918_ADDR, 0x31, 0x08)
        time.sleep(0.1)

    def _read_word_2c(self, addr, reg):
        high = self.bus.read_byte_data(addr, reg)
        low  = self.bus.read_byte_data(addr, reg + 1)
        val  = (high << 8) + low
        return val - 65536 if val >= 0x8000 else val

    def get_gyro(self):
        gx = self._read_word_2c(self.ICM20600_ADDR, 0x43)
        gy = self._read_word_2c(self.ICM20600_ADDR, 0x45)
        gz = self._read_word_2c(self.ICM20600_ADDR, 0x47)
        scale = 131.0
        return gx / scale, gy / scale, gz / scale

    def get_accel(self):
        ax = self._read_word_2c(self.ICM20600_ADDR, 0x3B)
        ay = self._read_word_2c(self.ICM20600_ADDR, 0x3D)
        az = self._read_word_2c(self.ICM20600_ADDR, 0x3F)
        scale = 16384.0  # ±2g
        return ax / scale, ay / scale, az / scale

    def get_mag(self):
        # Espera até dados estarem prontos (max 10 tentativas)
        for _ in range(10):
            status = self.bus.read_byte_data(self.AK09918_ADDR, 0x10)
            if status & 0x01:
                break
            time.sleep(0.01)
        else:
            return None  # timeout

        data = self.bus.read_i2c_block_data(self.AK09918_ADDR, 0x11, 6)
        self.bus.read_byte_data(self.AK09918_ADDR, 0x18)  # liberta buffer

        mx = (data[1] << 8) | data[0]
        my = (data[3] << 8) | data[2]
        mz = (data[5] << 8) | data[4]
        if mx >= 0x8000: mx -= 65536
        if my >= 0x8000: my -= 65536
        if mz >= 0x8000: mz -= 65536
        return mx * 0.15, my * 0.15, mz * 0.15

    def get_heading(self):
        mag = self.get_mag()
        if mag is None:
            return None, None

        mx, my, _ = mag

        # Aplicar calibração: remover offset e corrigir escala
        mx_cal = (mx - self.mag_offset[0]) * self.mag_scale[0]
        my_cal = (my - self.mag_offset[1]) * self.mag_scale[1]

        # Para inverter APENAS Norte e Sul (sem afetar Leste/Oeste), 
        # invertemos apenas o sinal do eixo X no atan2:
        heading = math.atan2(-my_cal, mx_cal) * 180 / math.pi
        heading = heading % 360

        pos = int(heading // 90) % 4

        return heading, pos

