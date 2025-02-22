import wpilib
import struct

class TFMini:
    def __init__(self, port: wpilib.SerialPort.Port) -> None:
        self.serial = wpilib.SerialPort(115200, port)
        self.serial.setTimeout(1.0)

    def getDistanceCM(self) -> float:
        """Read the distance in centimeters from the TF Mini sensor."""
        data = self.serial.read(9)
        if len(data) == 9 and [data[0], data[1]] == 0x59:
            return struct.unpack('<H', data[2:4])[0]
        return -1.0  # Return -1.0 if reading fails
    
    def getDistanceIN(self) -> float:
        """Read the distance in inches from the TF Mini sensor."""
        data = self.serial.read(9)
        if len(data) == 9 and [data[0], data[1]] == 0x59:
            return struct.unpack('<H', data[2:4])[0] / 2.54
        return -1.0  # Return -1.0 if reading fails

    def close(self) -> None:
        """Close the serial port."""
        self.serial.close()