import ctre
import time

class TFMini:
    def __init__(self, canID: int) -> None:
        self.device = ctre.CANifier(canID)
        self.lastTime = time.time()
        self.lastState = self.device.getGeneralInput(ctre.CANifier.GeneralPin.SPI_CLK)

    def getDistance(self) -> float:
        currentTime = time.time()
        currentState = self.device.getGeneralInput(ctre.CANifier.GeneralPin.SPI_CLK)

        if currentState != self.lastState:
            period = currentTime - self.lastTime
            self.lastTime = currentTime
            self.lastState = currentState
            distance = period * 100000.0 / 10.0  # Convert period to distance in cm
            return distance
        return -1.0

    def isValid(self) -> bool:
        return self.getDistance() > 0
