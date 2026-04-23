from grove.grove_imu_9dof_icm20600_ak09918 import GroveIMU9DOFICM20600, GroveIMU9DOFAK09918
import time

gyro = GroveIMU9DOFICM20600()
mag  = GroveIMU9DOFAK09918()

while True:
    gx, gy, gz = gyro.get_gyro()
    mx, my, mz = mag.get_magnetic()
    print(f"Gyro: {gx:.2f} {gy:.2f} {gz:.2f}")
    print(f"Mag:  {mx:.2f} {my:.2f} {mz:.2f}")
    time.sleep(0.1)