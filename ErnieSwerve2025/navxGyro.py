import navx
import wpimath

class Gyro:
    def __init__(self):
        self.angler = navx.AHRS.create_spi()

    def getGyro(self):
        self.angle = self.angler.getAngle()
        self.radiansgyro = wpimath.units.degreesToRadians(self.angle)