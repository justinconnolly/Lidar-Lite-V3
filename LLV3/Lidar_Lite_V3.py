import smbus
import time

class Lidar():
    def __init__(self, address=0x62):
        self.address = address
        self.distance_write_register = 0x00
        self.distance_write_value = 0x04
        self.distance_write_value_no_bias_correction = 0x03
        self.distance_read_high = 0x8f
        self.distance_read_low = 0x10
        self.i2c_connect(1)
        self.i2c_init(self.address)

    """
    Connect to the I2C bus.
    """
    def i2c_connect(self, bus):
        try:
            self.bus = smbus.SMBus(bus)
            time.sleep(1)
            print(f'Successfully connected to I2C bus {bus}.')
            return 0
        except:
            print(f'Error connecting to I2C bus {bus}.')
            return -1
        
    """
    Connect to the LL3V via I2C on the default address.
    """
    def i2c_init(self, address):
        try:
            self.configure()
            print('LiDAR successfully connected and configured.')
            return 0
        except:
            print(f'Error connecting to LiDAR over I2C address {hex(self.address)}.')
            return -1
    """
    Set default values per Garmin Lidar Lite for Arduino repo.
    """
    def configure(self):
        self.write_to_register(0x02, 0x80)
        self.write_to_register(0x04, 0x08)
        self.write_to_register(0x1c, 0x00)
        self.write_to_register(0x12, 0x05)

    def reset(self):
        self.write_to_register(0x00, 0x00)

    def write_to_register(self, reg, val):
        self.bus.write_byte_data(self.address, reg, val)
    
    def read_register(self, reg):
        val = self.bus.read_byte_data(self.address, reg)
        return val
    
    def get_distance(self, bias_correction=True):
        if bias_correction:
            self.write_to_register(self.distance_write_register, self.distance_write_value)
        else:
            self.write_to_register(self.distance_write_register, self.distance_write_value_no_bias_correction)
        
        # Poll status bit
        busy_flag = 1
        while busy_flag != 0:
            busy_flag = self.get_busy_flag()

        distance = self.bus.read_i2c_block_data(self.address, self.distance_read_high, 2)
        return (distance[0] << 8) + distance[1]
    
    def get_busy_flag(self):
        status_byte = 0
        status_byte = self.read_register(0x01)
        return status_byte & 0x01
    
    def burst_measurement(self, count=10, delay=None):
        distance_measurements = []

        # Initial measurement to populate registers 0x8f and 0x10
        self.get_distance()

        for _ in range(count):
            start = time.time()
            # Initiate distance measurement
            self.write_to_register(self.distance_write_register, self.distance_write_value)

            # Immediately read distance registers before new measurement command is complete
            distance = distance = self.bus.read_i2c_block_data(self.address, self.distance_read_high, 2)
            distance = (distance[0] << 8) + distance[1]
            distance_measurements.append([distance, time.time()])

            # Poll status bit
            busy_flag = 1
            while busy_flag != 0:
                busy_flag = self.get_busy_flag()

            end = time.time()
            diff = end - start
            if delay:
                if diff < delay:
                    time.sleep(delay - diff)
                diff = time.time() - start
            print(distance, diff)

        return distance_measurements
    
    def continuous(self, delay=None):
         # Initial measurement to populate registers 0x8f and 0x10
        self.get_distance()

        while True:
            start = time.time()
            # Initiate distance measurement
            self.write_to_register(self.distance_write_register, self.distance_write_value)

            # Immediately read distance registers before new measurement command is complete
            distance = distance = self.bus.read_i2c_block_data(self.address, self.distance_read_high, 2)
            distance = (distance[0] << 8) + distance[1]

            # Poll status bit
            busy_flag = 1
            while busy_flag != 0:
                busy_flag = self.get_busy_flag()
            end = time.time()
            diff = end - start
            if delay:
                if diff < delay:
                    time.sleep(delay - diff)
                diff = time.time() - start
            print(distance, diff)