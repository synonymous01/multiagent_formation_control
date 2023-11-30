import robomaster
from robomaster import robot
import time

class rob:
    def __init__(self):
        self.robo = robot.Robot()
        self.robo.initialize(conn_type="sta", sn="3JKDH6C0019B1H")
        self.chas = self.robo.chassis
        self.chas.sub_imu(10, self.update_vels)

    def update_vels(self, imu_info):
        a, b, c, d, e, velocity = imu_info
        print(velocity)


test = rob()
print("------------------------0.4")
test.chas.drive_speed(x=0, y=0, z=30)
time.sleep(15)
test.chas.drive_speed(x=0, y=0, z=40)
time.sleep(10)
test.chas.drive_speed(x=0, y=0, z=0)
