import binascii
import logging
import time
from wpilib import I2C, SerialPort
import wpimath.geometry

# I2C addresses
BNO055_ADDRESS_A = 0x28
BNO055_ADDRESS_B = 0x29
BNO055_ID = 0xA0

# Page id register definition
BNO055_PAGE_ID_ADDR = 0x07

# PAGE0 REGISTER DEFINITION START
BNO055_CHIP_ID_ADDR = 0x00
BNO055_ACCEL_REV_ID_ADDR = 0x01
BNO055_MAG_REV_ID_ADDR = 0x02
BNO055_GYRO_REV_ID_ADDR = 0x03
BNO055_SW_REV_ID_LSB_ADDR = 0x04
BNO055_SW_REV_ID_MSB_ADDR = 0x05
BNO055_BL_REV_ID_ADDR = 0x06

# Accel data register
BNO055_ACCEL_DATA_X_LSB_ADDR = 0x08
BNO055_ACCEL_DATA_X_MSB_ADDR = 0x09
BNO055_ACCEL_DATA_Y_LSB_ADDR = 0x0A
BNO055_ACCEL_DATA_Y_MSB_ADDR = 0x0B
BNO055_ACCEL_DATA_Z_LSB_ADDR = 0x0C
BNO055_ACCEL_DATA_Z_MSB_ADDR = 0x0D

# Mag data register
BNO055_MAG_DATA_X_LSB_ADDR = 0x0E
BNO055_MAG_DATA_X_MSB_ADDR = 0x0F
BNO055_MAG_DATA_Y_LSB_ADDR = 0x10
BNO055_MAG_DATA_Y_MSB_ADDR = 0x11
BNO055_MAG_DATA_Z_LSB_ADDR = 0x12
BNO055_MAG_DATA_Z_MSB_ADDR = 0x13

# Gyro data registers
BNO055_GYRO_DATA_X_LSB_ADDR = 0x14
BNO055_GYRO_DATA_X_MSB_ADDR = 0x15
BNO055_GYRO_DATA_Y_LSB_ADDR = 0x16
BNO055_GYRO_DATA_Y_MSB_ADDR = 0x17
BNO055_GYRO_DATA_Z_LSB_ADDR = 0x18
BNO055_GYRO_DATA_Z_MSB_ADDR = 0x19

# Euler data registers
BNO055_EULER_H_LSB_ADDR = 0x1A
BNO055_EULER_H_MSB_ADDR = 0x1B
BNO055_EULER_R_LSB_ADDR = 0x1C
BNO055_EULER_R_MSB_ADDR = 0x1D
BNO055_EULER_P_LSB_ADDR = 0x1E
BNO055_EULER_P_MSB_ADDR = 0x1F

# Quaternion data registers
BNO055_QUATERNION_DATA_W_LSB_ADDR = 0x20
BNO055_QUATERNION_DATA_W_MSB_ADDR = 0x21
BNO055_QUATERNION_DATA_X_LSB_ADDR = 0x22
BNO055_QUATERNION_DATA_X_MSB_ADDR = 0x23
BNO055_QUATERNION_DATA_Y_LSB_ADDR = 0x24
BNO055_QUATERNION_DATA_Y_MSB_ADDR = 0x25
BNO055_QUATERNION_DATA_Z_LSB_ADDR = 0x26
BNO055_QUATERNION_DATA_Z_MSB_ADDR = 0x27

# Linear acceleration data registers
BNO055_LINEAR_ACCEL_DATA_X_LSB_ADDR = 0x28
BNO055_LINEAR_ACCEL_DATA_X_MSB_ADDR = 0x29
BNO055_LINEAR_ACCEL_DATA_Y_LSB_ADDR = 0x2A
BNO055_LINEAR_ACCEL_DATA_Y_MSB_ADDR = 0x2B
BNO055_LINEAR_ACCEL_DATA_Z_LSB_ADDR = 0x2C
BNO055_LINEAR_ACCEL_DATA_Z_MSB_ADDR = 0x2D

# Gravity data registers
BNO055_GRAVITY_DATA_X_LSB_ADDR = 0x2E
BNO055_GRAVITY_DATA_X_MSB_ADDR = 0x2F
BNO055_GRAVITY_DATA_Y_LSB_ADDR = 0x30
BNO055_GRAVITY_DATA_Y_MSB_ADDR = 0x31
BNO055_GRAVITY_DATA_Z_LSB_ADDR = 0x32
BNO055_GRAVITY_DATA_Z_MSB_ADDR = 0x33

# Temperature data register
BNO055_TEMP_ADDR = 0x34

# Status registers
BNO055_CALIB_STAT_ADDR = 0x35
BNO055_SELFTEST_RESULT_ADDR = 0x36
BNO055_INTR_STAT_ADDR = 0x37

BNO055_SYS_CLK_STAT_ADDR = 0x38
BNO055_SYS_STAT_ADDR = 0x39
BNO055_SYS_ERR_ADDR = 0x3A

# Unit selection register
BNO055_UNIT_SEL_ADDR = 0x3B
BNO055_DATA_SELECT_ADDR = 0x3C

# Mode registers
BNO055_OPR_MODE_ADDR = 0x3D
BNO055_PWR_MODE_ADDR = 0x3E

BNO055_SYS_TRIGGER_ADDR = 0x3F
BNO055_TEMP_SOURCE_ADDR = 0x40

# Axis remap registers
BNO055_AXIS_MAP_CONFIG_ADDR = 0x41
BNO055_AXIS_MAP_SIGN_ADDR = 0x42

# Axis remap values
AXIS_REMAP_X = 0x00
AXIS_REMAP_Y = 0x01
AXIS_REMAP_Z = 0x02
AXIS_REMAP_POSITIVE = 0x00
AXIS_REMAP_NEGATIVE = 0x01

# SIC registers
BNO055_SIC_MATRIX_0_LSB_ADDR = 0x43
BNO055_SIC_MATRIX_0_MSB_ADDR = 0x44
BNO055_SIC_MATRIX_1_LSB_ADDR = 0x45
BNO055_SIC_MATRIX_1_MSB_ADDR = 0x46
BNO055_SIC_MATRIX_2_LSB_ADDR = 0x47
BNO055_SIC_MATRIX_2_MSB_ADDR = 0x48
BNO055_SIC_MATRIX_3_LSB_ADDR = 0x49
BNO055_SIC_MATRIX_3_MSB_ADDR = 0x4A
BNO055_SIC_MATRIX_4_LSB_ADDR = 0x4B
BNO055_SIC_MATRIX_4_MSB_ADDR = 0x4C
BNO055_SIC_MATRIX_5_LSB_ADDR = 0x4D
BNO055_SIC_MATRIX_5_MSB_ADDR = 0x4E
BNO055_SIC_MATRIX_6_LSB_ADDR = 0x4F
BNO055_SIC_MATRIX_6_MSB_ADDR = 0x50
BNO055_SIC_MATRIX_7_LSB_ADDR = 0x51
BNO055_SIC_MATRIX_7_MSB_ADDR = 0x52
BNO055_SIC_MATRIX_8_LSB_ADDR = 0x53
BNO055_SIC_MATRIX_8_MSB_ADDR = 0x54

# Accelerometer Offset registers
ACCEL_OFFSET_X_LSB_ADDR = 0x55
ACCEL_OFFSET_X_MSB_ADDR = 0x56
ACCEL_OFFSET_Y_LSB_ADDR = 0x57
ACCEL_OFFSET_Y_MSB_ADDR = 0x58
ACCEL_OFFSET_Z_LSB_ADDR = 0x59
ACCEL_OFFSET_Z_MSB_ADDR = 0x5A

# Magnetometer Offset registers
MAG_OFFSET_X_LSB_ADDR = 0x5B
MAG_OFFSET_X_MSB_ADDR = 0x5C
MAG_OFFSET_Y_LSB_ADDR = 0x5D
MAG_OFFSET_Y_MSB_ADDR = 0x5E
MAG_OFFSET_Z_LSB_ADDR = 0x5F
MAG_OFFSET_Z_MSB_ADDR = 0x60

# Gyroscope Offset register s
GYRO_OFFSET_X_LSB_ADDR = 0x61
GYRO_OFFSET_X_MSB_ADDR = 0x62
GYRO_OFFSET_Y_LSB_ADDR = 0x63
GYRO_OFFSET_Y_MSB_ADDR = 0x64
GYRO_OFFSET_Z_LSB_ADDR = 0x65
GYRO_OFFSET_Z_MSB_ADDR = 0x66

# Radius registers
ACCEL_RADIUS_LSB_ADDR = 0x67
ACCEL_RADIUS_MSB_ADDR = 0x68
MAG_RADIUS_LSB_ADDR = 0x69
MAG_RADIUS_MSB_ADDR = 0x6A

# Power modes
POWER_MODE_NORMAL = 0x00
POWER_MODE_LOWPOWER = 0x01
POWER_MODE_SUSPEND = 0x02

# Operation mode settings
OPERATION_MODE_CONFIG = 0x00
OPERATION_MODE_ACCONLY = 0x01
OPERATION_MODE_MAGONLY = 0x02
OPERATION_MODE_GYRONLY = 0x03
OPERATION_MODE_ACCMAG = 0x04
OPERATION_MODE_ACCGYRO = 0x05
OPERATION_MODE_MAGGYRO = 0x06
OPERATION_MODE_AMG = 0x07
OPERATION_MODE_IMUPLUS = 0x08
OPERATION_MODE_COMPASS = 0x09
OPERATION_MODE_M4G = 0x0A
OPERATION_MODE_NDOF_FMC_OFF = 0x0B
OPERATION_MODE_NDOF = 0x0C

logger = logging.getLogger(__name__)

class BNO055:
    def __init__(self, rst=None, address=BNO055_ADDRESS_A, i2c=None, port=None, timeout=5, **kwargs):
        self.rst = rst
        self.serial = None
        self.i2c = None

        if port is not None:
            self.serial = SerialPort(baud_rate=115200, port=port, timeout=timeout)
        else:
            if i2c is None:
                i2c = I2C(I2C.Port.kOnboard, address)
            self.i2c = i2c

    def serialSend(self, command, ack=True, max_attempts=5):
        attempts = 0
        while True:
            self.serial.flush()
            self.serial.write(command)
            logger.debug('Serial send: 0x{0}'.format(binascii.hexlify(command)))
            if not ack:
                return
            resp = bytearray(self.serial.read(2))
            logger.debug('Serial receive: 0x{0}'.format(binascii.hexlify(resp)))
            if resp is None or len(resp) != 2:
                raise RuntimeError('Timeout waiting for serial acknowledge, is the BNO055 connected?')
            if not (resp[0] == 0xEE and resp[1] == 0x07):
                return resp
            attempts += 1
            if attempts >= max_attempts:
                raise RuntimeError('Exceeded maximum attempts to acknowledge serial command without bus error!')

    def writeBytes(self, address, data, ack=True):
        if self.i2c is not None:
            self.i2c.write(address, data)
        else:
            command = bytearray(4 + len(data))
            command[0] = 0xAA
            command[1] = 0x00
            command[2] = address & 0xFF
            command[3] = len(data) & 0xFF
            command[4:] = map(lambda x: x & 0xFF, data)
            resp = self.serialSend(command, ack=ack)
            if resp[0] != 0xEE and resp[1] != 0x01:
                raise RuntimeError('Register write error: 0x{0}'.format(binascii.hexlify(resp)))

    def writeByte(self, address, value, ack=True):
        if self.i2c is not None:
            self.i2c.write(address, [value])
        else:
            command = bytearray(5)
            command[0] = 0xAA
            command[1] = 0x00
            command[2] = address & 0xFF
            command[3] = 1
            command[4] = value & 0xFF
            resp = self.serialSend(command, ack=ack)
            if ack and resp[0] != 0xEE and resp[1] != 0x01:
                raise RuntimeError('Register write error: 0x{0}'.format(binascii.hexlify(resp)))

    def readBytes(self, address, length):
        if self.i2c is not None:
            return bytearray(self.i2c.read(address, length))
        else:
            command = bytearray(4)
            command[0] = 0xAA
            command[1] = 0x01
            command[2] = address & 0xFF
            command[3] = length & 0xFF
            resp = self.serialSend(command)
            if resp[0] != 0xBB:
                raise RuntimeError('Register read error: 0x{0}'.format(binascii.hexlify(resp)))
            length = resp[1]
            resp = bytearray(self.serial.read(length))
            logger.debug('Received: 0x{0}'.format(binascii.hexlify(resp)))
            if resp is None or len(resp) != length:
                raise RuntimeError('Timeout waiting to read data, is the BNO055 connected?')
            return resp

    def readByte(self, address):
        if self.i2c is not None:
            return self.i2c.read(address, 1)[0]
        else:
            return self.readBytes(address, 1)[0]

    def readSignedByte(self, address):
        data = self.readByte(address)
        if data > 127:
            return data - 256
        else:
            return data

    def configMode(self):
        self.setMode(OPERATION_MODE_CONFIG)

    def operationMode(self):
        self.setMode(self._mode)

    def begin(self, mode=OPERATION_MODE_NDOF):
        self._mode = mode
        try:
            self.writeByte(BNO055_PAGE_ID_ADDR, 0, ack=False)
        except IOError:
            pass
        self.configMode()
        self.writeByte(BNO055_PAGE_ID_ADDR, 0)
        bno_id = self.readByte(BNO055_CHIP_ID_ADDR)
        logger.debug('Read chip ID: 0x{0:02X}'.format(bno_id))
        if bno_id != BNO055_ID:
            return False
        if self.rst is not None:
            self._gpio.set_low(self.rst)
            time.sleep(0.01)
            self._gpio.set_high(self.rst)
        else:
            self.writeByte(BNO055_SYS_TRIGGER_ADDR, 0x20, ack=False)
        time.sleep(0.65)
        self.writeByte(BNO055_PWR_MODE_ADDR, POWER_MODE_NORMAL)
        self.writeByte(BNO055_SYS_TRIGGER_ADDR, 0x0)
        self.operationMode()
        return True

    def setMode(self, mode):
        self.writeByte(BNO055_OPR_MODE_ADDR, mode & 0xFF)
        time.sleep(0.03)

    def getRevision(self):
        accel = self.readByte(BNO055_ACCEL_REV_ID_ADDR)
        mag = self.readByte(BNO055_MAG_REV_ID_ADDR)
        gyro = self.readByte(BNO055_GYRO_REV_ID_ADDR)
        bl = self.readByte(BNO055_BL_REV_ID_ADDR)
        sw_lsb = self.readByte(BNO055_SW_REV_ID_LSB_ADDR)
        sw_msb = self.readByte(BNO055_SW_REV_ID_MSB_ADDR)
        sw = ((sw_msb << 8) | sw_lsb) & 0xFFFF
        return (sw, bl, accel, mag, gyro)

    def setExternalCrystal(self, external_crystal):
        self.configMode()
        if external_crystal:
            self.writeByte(BNO055_SYS_TRIGGER_ADDR, 0x80)
        else:
            self.writeByte(BNO055_SYS_TRIGGER_ADDR, 0x00)
        self.operationMode()

    def getSystemStatus(self, run_self_test=True):
        self_test = None
        if run_self_test:
            self.configMode()
            sys_trigger = self.readByte(BNO055_SYS_TRIGGER_ADDR)
            self.writeByte(BNO055_SYS_TRIGGER_ADDR, sys_trigger | 0x1)
            time.sleep(1.0)
            self_test = self.readByte(BNO055_SELFTEST_RESULT_ADDR)
            self.operationMode()
        status = self.readByte(BNO055_SYS_STAT_ADDR)
        error = self.readByte(BNO055_SYS_ERR_ADDR)
        return (status, self_test, error)

    def getCalibrationstatus(self):
        cal_status = self.readByte(BNO055_CALIB_STAT_ADDR)
        sys = (cal_status >> 6) & 0x03
        gyro = (cal_status >> 4) & 0x03
        accel = (cal_status >> 2) & 0x03
        mag = cal_status & 0x03
        return (sys, gyro, accel, mag)

    def getCalibration(self):
        self.configMode()
        cal_data = list(self.readBytes(ACCEL_OFFSET_X_LSB_ADDR, 22))
        self.operationMode()
        return cal_data

    def setCalibration(self, data):
        if data is None or len(data) != 22:
            raise ValueError('Expected a list of 22 bytes for calibration data.')
        self.configMode()
        self.writeBytes(ACCEL_OFFSET_X_LSB_ADDR, data)
        self.operationMode()

    def getAxisRemap(self):
        map_config = self.readByte(BNO055_AXIS_MAP_CONFIG_ADDR)
        z = (map_config >> 4) & 0x03
        y = (map_config >> 2) & 0x03
        x = map_config & 0x03
        sign_config = self.readByte(BNO055_AXIS_MAP_SIGN_ADDR)
        x_sign = (sign_config >> 2) & 0x01
        y_sign = (sign_config >> 1) & 0x01
        z_sign = sign_config & 0x01
        return (x, y, z, x_sign, y_sign, z_sign)

    def setAxisRemap(self, x, y, z,
                       x_sign=AXIS_REMAP_POSITIVE, y_sign=AXIS_REMAP_POSITIVE,
                       z_sign=AXIS_REMAP_POSITIVE):
        self.configMode()
        map_config = 0x00
        map_config |= (z & 0x03) << 4
        map_config |= (y & 0x03) << 2
        map_config |= x & 0x03
        self.writeByte(BNO055_AXIS_MAP_CONFIG_ADDR, map_config)
        sign_config = 0x00
        sign_config |= (x_sign & 0x01) << 2
        sign_config |= (y_sign & 0x01) << 1
        sign_config |= z_sign & 0x01
        self.writeByte(BNO055_AXIS_MAP_SIGN_ADDR, sign_config)
        self.operationMode()

    def readVector(self, address, count=3):
        data = self.readBytes(address, count*2)
        result = [0]*count
        for i in range(count):
            result[i] = ((data[i*2+1] << 8) | data[i*2]) & 0xFFFF
            if result[i] > 32767:
                result[i] -= 65536
        return result

    def readEuler(self) -> wpimath.geometry.Rotation3d:
        heading, roll, pitch = self.readVector(BNO055_EULER_H_LSB_ADDR)
        return wpimath.geometry.Rotation3d(heading / 16.0, roll / 16.0, pitch / 16.0)

    def readMagnetometer(self) -> wpimath.geometry.Translation3d:
        x, y, z = self.readVector(BNO055_MAG_DATA_X_LSB_ADDR)
        return wpimath.geometry.Translation3d(x / 16.0, y / 16.0, z / 16.0)

    def readGyroscope(self) -> wpimath.geometry.Translation3d:
        x, y, z = self.readVector(BNO055_GYRO_DATA_X_LSB_ADDR)
        return wpimath.geometry.Translation3d(x / 900.0, y / 900.0, z / 900.0)

    def readAccelerometer(self) -> wpimath.geometry.Translation3d:
        x, y, z = self.readVector(BNO055_ACCEL_DATA_X_LSB_ADDR)
        return wpimath.geometry.Translation3d(x / 100.0, y / 100.0, z / 100.0)

    def readLinearAcceleration(self) -> wpimath.geometry.Translation3d:
        x, y, z = self.readVector(BNO055_LINEAR_ACCEL_DATA_X_LSB_ADDR)
        return wpimath.geometry.Translation3d(x / 100.0, y / 100.0, z / 100.0)

    def readGravity(self) -> wpimath.geometry.Translation3d:
        x, y, z = self.readVector(BNO055_GRAVITY_DATA_X_LSB_ADDR)
        return wpimath.geometry.Translation3d(x / 100.0, y / 100.0, z / 100.0)

    def readQuaternion(self) -> wpimath.geometry.Quaternion:
        w, x, y, z = self.readVector(BNO055_QUATERNION_DATA_W_LSB_ADDR, 4)
        scale = (1.0 / (1<<14))
        return wpimath.geometry.Quaternion(x * scale, y * scale, z * scale, w * scale)

    def readTemp(self):
        return self.readSignedByte(BNO055_TEMP_ADDR)