"""
MicroPython driver for the QMC5883L 3-Axis Magnetic Sensor.
Compatible with Raspberry Pi Pico 2W.

Usage example:

  from machine import Pin, I2C
  import qmc5883l_micropython as qmc5883l
  
  # Initialize I2C (adjust pins as needed)
  i2c = I2C(0, scl=Pin(9), sda=Pin(8), freq=400000)
  
  # Create sensor instance
  sensor = qmc5883l.QMC5883L(i2c=i2c)
  
  # Read magnetic data
  m = sensor.get_magnet()
  print(m)
  
  # Read bearing
  bearing = sensor.get_bearing()
  print(f"Bearing: {bearing}°")

You will get three 16 bit signed integers, representing the values
of the magnetic sensor on axis X, Y and Z, e.g. [-1257, 940, -4970].
"""
import math
import time
from machine import I2C

__author__ = "Yanfu Zhou (adapted for MicroPython)"
__copyright__ = "Copyright 2022 Yanfu Zhou <yanfu.zhou@outlook.com>"
__license__ = "GPLv3-or-later"
__version__ = "1.0.0-micropython"

DFLT_ADDRESS = 0x0d

REG_XOUT_LSB = 0x00     # Output Data Registers for magnetic sensor.
REG_XOUT_MSB = 0x01
REG_YOUT_LSB = 0x02
REG_YOUT_MSB = 0x03
REG_ZOUT_LSB = 0x04
REG_ZOUT_MSB = 0x05
REG_STATUS_1 = 0x06     # Status Register.
REG_TOUT_LSB = 0x07     # Output Data Registers for temperature.
REG_TOUT_MSB = 0x08
REG_CONTROL_1 = 0x09    # Control Register #1.
REG_CONTROL_2 = 0x0a    # Control Register #2.
REG_RST_PERIOD = 0x0b   # SET/RESET Period Register.
REG_CHIP_ID = 0x0d      # Chip ID register.

# Flags for Status Register #1.
STAT_DRDY = 0b00000001  # Data Ready.
STAT_OVL = 0b00000010   # Overflow flag.
STAT_DOR = 0b00000100   # Data skipped for reading.

# Flags for Status Register #2.
INT_ENB = 0b00000001    # Interrupt Pin Enabling.
POL_PNT = 0b01000000    # Pointer Roll-over.
SOFT_RST = 0b10000000   # Soft Reset.

# Flags for Control Register 1.
MODE_STBY = 0b00000000  # Standby mode.
MODE_CONT = 0b00000001  # Continuous read mode.
ODR_10HZ = 0b00000000   # Output Data Rate Hz.
ODR_50HZ = 0b00000100
ODR_100HZ = 0b00001000
ODR_200HZ = 0b00001100
RNG_2G = 0b00000000     # Range 2 Gauss: for magnetic-clean environments.
RNG_8G = 0b00010000     # Range 8 Gauss: for strong magnetic fields.
OSR_512 = 0b00000000    # Over Sample Rate 512: less noise, more power.
OSR_256 = 0b01000000
OSR_128 = 0b10000000
OSR_64 = 0b11000000     # Over Sample Rate 64: more noise, less power.


class QMC5883L:
    """Interface for the QMC5883l 3-Axis Magnetic Sensor."""
    
    def __init__(self,
                 i2c,
                 address=DFLT_ADDRESS,
                 output_data_rate=ODR_200HZ,
                 output_range=RNG_8G,
                 oversampling_rate=OSR_512):
        """
        Initialize the QMC5883L sensor.
        
        Args:
            i2c: machine.I2C object
            address: I2C address (default 0x0d)
            output_data_rate: ODR_10HZ, ODR_50HZ, ODR_100HZ, or ODR_200HZ
            output_range: RNG_2G or RNG_8G
            oversampling_rate: OSR_512, OSR_256, OSR_128, or OSR_64
        """
        self.i2c = i2c
        self.address = address
        self.output_range = output_range
        self._declination = 0.0
        self._calibration = [[1.0, 0.0, 0.0],
                             [0.0, 1.0, 0.0],
                             [0.0, 0.0, 1.0]]
        
        # Check chip ID
        chip_id = self._read_byte(REG_CHIP_ID)
        if chip_id != 0xff:
            print(f"Warning: Chip ID returned 0x{chip_id:02x} instead of 0xff")
        
        self.mode_cont = (MODE_CONT | output_data_rate | output_range
                          | oversampling_rate)
        self.mode_stby = (MODE_STBY | ODR_10HZ | RNG_2G | OSR_64)
        
        # Set to continuous mode
        self.mode_continuous()
    
    def __del__(self):
        """Once finished using the sensor, switch to standby mode."""
        try:
            self.mode_standby()
        except:
            pass
    
    def _read_byte(self, register):
        """Read a single byte from a register."""
        return self.i2c.readfrom_mem(self.address, register, 1)[0]
    
    def _read_bytes(self, register, length):
        """Read multiple bytes from a register."""
        return self.i2c.readfrom_mem(self.address, register, length)
    
    def _write_bytes(self, register, data):
        """Write multiple bytes to a register."""
        if isinstance(data, list):
            data = bytes(data)
        self.i2c.writeto_mem(self.address, register, data)
    
    def mode_continuous(self):
        """Set the device in continuous read mode."""
        # Soft reset
        self._write_bytes(REG_CONTROL_1, [
            (MODE_CONT | ODR_10HZ | RNG_2G | OSR_512)])
        self._write_bytes(REG_CONTROL_2, [SOFT_RST])
        time.sleep_ms(10)
        
        # Set period
        self._write_bytes(REG_RST_PERIOD, [0x01])
        
        # Enable interrupt
        self._write_bytes(REG_CONTROL_2, [INT_ENB])
        
        # Set mode
        self._write_bytes(REG_CONTROL_1, [self.mode_cont])
        time.sleep_ms(10)
    
    def mode_standby(self):
        """Set the device in standby mode."""
        self._write_bytes(REG_CONTROL_1, [self.mode_stby])
        self._write_bytes(REG_CONTROL_2, [SOFT_RST])
        time.sleep_ms(10)
    
    @staticmethod
    def _extract_from_bytes(data, offset):
        """Extract 16-bit signed integer from byte array."""
        if offset == REG_TOUT_LSB:
            val = data[0] | (data[1] << 8)
        else:
            val = data[offset] | (data[offset + 1] << 8)
        
        # Convert to signed integer
        if val >= 2 ** 15:
            return val - 2 ** 16
        else:
            return val
    
    def get_data(self):
        """Read data from magnetic and temperature data registers."""
        # Read magnetic data (6 bytes starting from X LSB)
        data = self._read_bytes(REG_XOUT_LSB, 6)
        x = self._extract_from_bytes(data=data, offset=REG_XOUT_LSB)
        y = self._extract_from_bytes(data=data, offset=REG_YOUT_LSB)
        z = self._extract_from_bytes(data=data, offset=REG_ZOUT_LSB)
        
        # Read temperature data (2 bytes)
        temp_data = self._read_bytes(REG_TOUT_LSB, 2)
        t = self._extract_from_bytes(data=temp_data, offset=REG_TOUT_LSB)
        
        return [x, y, z, t]
    
    def get_magnet_raw(self):
        """Get the 3 axis values from magnetic sensor."""
        data = self.get_data()
        return [data[0], data[1], data[2]]
    
    def get_magnet(self):
        """Return the horizontal magnetic sensor vector with (x, y) calibration applied."""
        [x, y, z] = self.get_magnet_raw()
        
        if x is None or y is None:
            return [x, y]
        else:
            c = self._calibration
            x1 = x * c[0][0] + y * c[0][1] + c[0][2]
            y1 = x * c[1][0] + y * c[1][1] + c[1][2]
            return [x1, y1]
    
    def get_bearing_raw(self):
        """Horizontal bearing (in degrees) from magnetic value X and Y."""
        [x, y, z] = self.get_magnet_raw()
        
        if x is None or y is None:
            return None
        else:
            b = math.degrees(math.atan2(y, x))
            if b < 0:
                b += 360.0
            return b
    
    def get_bearing(self):
        """Horizontal bearing, adjusted by calibration and declination."""
        [x, y] = self.get_magnet()
        
        if x is None or y is None:
            return None
        else:
            b = math.degrees(math.atan2(y, x))
            if b < 0:
                b += 360.0
            b += self._declination
            if b < 0.0:
                b += 360.0
            elif b >= 360.0:
                b -= 360.0
            return b
    
    def get_temp(self):
        """Raw (uncalibrated) data from temperature sensor."""
        data = self.get_data()
        return data[3]
    
    def set_declination(self, value):
        """Set the magnetic declination, in degrees."""
        if isinstance(value, (int, float)):
            d = float(value)
            if d < -180.0 or d > 180.0:
                print('Error: Declination must be >= -180 and <= 180.')
            else:
                self._declination = d
        else:
            print('Error: Declination must be a float value.')
    
    def get_declination(self):
        """Return the current set value of magnetic declination."""
        return self._declination
    
    def set_calibration(self, value):
        """Set the 3x3 matrix for horizontal (x, y) magnetic vector calibration."""
        c = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
        
        if (isinstance(value, list) and len(value) == 3 and
            len(value[0]) == 3 and len(value[1]) == 3 and len(value[2]) == 3):
            for i in range(0, 3):
                for j in range(0, 3):
                    c[i][j] = float(value[i][j])
            self._calibration = c
        else:
            print('Error: Calibration must be a 3x3 float matrix.')
    
    def get_calibration(self):
        """Return the current set value of the calibration matrix."""
        return self._calibration
    
    @property
    def declination(self):
        """Magnetic declination to adjust bearing."""
        return self.get_declination()
    
    @declination.setter
    def declination(self, value):
        self.set_declination(value)
    
    @property
    def calibration(self):
        """Transformation matrix to adjust (x, y) magnetic vector."""
        return self.get_calibration()
    
    @calibration.setter
    def calibration(self, value):
        self.set_calibration(value)
    
    def read_status(self):
        """Read the status register."""
        return self._read_byte(REG_STATUS_1)
    
    def is_data_ready(self):
        """Check if new data is ready to read."""
        status = self.read_status()
        return bool(status & STAT_DRDY)
    
    def is_overflow(self):
        """Check if overflow occurred."""
        status = self.read_status()
        return bool(status & STAT_OVL)


# Helper function to create sensor with default I2C pins for Pico 2W
def create_sensor(scl_pin=5, sda_pin=4, freq=400000, **kwargs):
    """
    Create QMC5883L sensor with default I2C configuration for Pico 2W.
    
    Args:
        scl_pin: GPIO pin for I2C SCL (default 9)
        sda_pin: GPIO pin for I2C SDA (default 8)
        freq: I2C frequency (default 400000)
        **kwargs: Additional arguments for QMC5883L constructor
    
    Returns:
        QMC5883L instance
    """
    from machine import Pin
    i2c = I2C(0, scl=Pin(scl_pin), sda=Pin(sda_pin), freq=freq)
    return QMC5883L(i2c=i2c, **kwargs)

