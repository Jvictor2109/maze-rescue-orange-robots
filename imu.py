import smbus2
import time
import math


class IMU:
    ICM20600_ADDR = 0x69
    AK09918_ADDR  = 0x0C

    def __init__(self, bus_number=1):
        self.bus = smbus2.SMBus(bus_number)

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

    def get_heading(self, mx, my):
        heading = math.atan2(my, mx) * 180 / math.pi
        if heading < 0:
            heading += 360
        
        if heading >= 0 and heading < 90:
            return "N"  
        elif heading >= 90 and heading < 180:
            return "O"
        elif heading >= 180 and heading < 270:
            return "S"
        elif heading >= 270:
            return "E"

        return heading
    
