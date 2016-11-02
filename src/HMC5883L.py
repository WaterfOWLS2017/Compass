import smbus


def signed16(N):
    if N >= 2**15:
        N = -2**16 + N
    return N


class HMC5883L:

    def __init__(self, i2cbus):
        self.i2c1 = smbus.SMBus(i2cbus)
        self.addr = 0x1E
        self.data_out_rate = {0.75: 0x00, 1.5: 0x01, 3: 0x02, 7.5: 0x03, 15: 0x04, 30: 0x05, 75: 0x06}
        self.data_out_rate_inv = {v: k for (k, v) in self.data_out_rate.items()}
        self.sample_average = {1: 0x00, 2: 0x01, 4: 0x02, 8: 0x03}
        self.sample_average_inv = {v: k for (k, v) in self.sample_average.items()}
        self.gain_setting = {0.88: 0, 1.3: 1, 1.9: 2, 2.5: 3, 4.0: 4, 4.7: 5, 5.6: 6, 8.1: 7}
        self.gain_setting_inv = {v: k for (k, v) in self.gain_setting.items()}
        self.measurement_mode = {"continuous": 0x00, "single": 0x01, "idle": 0x2}
        self.measurement_mode_inv = {0x00: "continuous", 0x01: "single", 0x02: "idle", 0x03: "idle"}

    def read(self):
        '''Reads X, Y, and Z values from magnetometer
        
        If any of the values read -4096 the ADC is overflowing, try reducing the gain.'''
        bites = self.i2c1.read_i2c_block_data(self.addr, 0x03, 6)
        MSB = bites[0::2]
        LSB = bites[1::2]
        (X, Z, Y) = tuple(map(lambda (a, b): signed16(a*256+b), zip(MSB, LSB)))
        return (X, Y, Z)

    def isLocked(self):
        '''Checks if data registers in magnetometer are locked
        
        Common causes from lock are:
         *  Partial reading of the data registers. Do a full read to solve.
         *  Mode register has been read. Change the mode to solve.
        Note: A full reset also resets the lock.
        '''
        return bool(self.i2c1.read_byte_data(self.addr, 0x09) & 0x02)
        
    def isReady(self):
        '''Checks if new magnetic data is available in the data registers

        This goes low as soon as one data register is read, and won't go high until
        all registers are read (lock is gone) and a new measurement has again been
        made.
        '''
        return bool(self.i2c1.read_byte_data(self.addr, 0x09) & 0x01)

    def setRate(self, rate):
        '''Select a data output rate for the magnetometer

        Available values (in Hz):
         * .75  * 1.5   * 3.0
         * 7.5  * 15    * 30
         * 75

        Note: 15 Hz is the default data output rate.OD
        '''
        assert rate in self.data_out_rate, "Selected non-valid data output rate, see docstring"
        old = self.i2c1.read_byte_data(self.addr, 0x00)
        self.i2c1.write_byte_data(self.addr, 0x00, (old & 0xE3) | (self.data_out_rate[rate] << 2))

    def getRate(self):
        '''Gets current data output rate configuration from magnetometer'''
        return self.data_out_rate_inv[(self.i2c1.read_byte_data(self.addr, 0x00) & 0x1C) >> 2]

    def setSamplesAveraged(self, avg):
        '''Select a number of samples to average for each magnetometer measurement

        Available values (in samples): 1, 2, 4, 8

        Note: using a large value with a small data output rate might not be a good
        idea, unless the object is particularly slow moving.
        '''
        assert avg in self.sample_average, "Selected non-valid sample average rate, see docstring"
        old = self.i2c1.read_byte_data(self.addr, 0x00)
        self.i2c1.write_byte_data(self.addr, 0x00, (old & 0x9F) | (self.sample_average[avg] << 5))

    def getSamplesAveraged(self):
        '''Gets current average sample per measurement configuration from magnetometer'''
        return self.sample_average_inv[(self.i2c1.read_byte_data(self.addr, 0x00) & 0x1C) >> 5]

    def getGainSetting(self):
        '''Gets current gain setting from magnetometer'''
        return self.gain_setting_inv[(self.i2c1.read_byte_data(self.addr, 0x01) & 0xE0) >> 5]

    def setGainSetting(self, gain):
        '''Select a gain to apply to the magnetic readings
        
        Available values (in gain):
         * .88  * 1.3   * 1.9   * 2.5
         * 4.0  * 4.7   * 5.6   * 8.1

        Note: Gain setting of 1.3 is the default.
        Usage: increasing gain increases resolution of reading, but increases chances
        of reading overflow.
        '''
        assert gain in self.gain_setting, "Selected non-valid gain setting, see docstring"
        old = self.i2c1.read_byte_data(self.addr, 0x01)
        self.i2c1.write_byte_data(self.addr, 0x01, (old & 0x1F) | (self.gain_setting[gain] << 5))
        
    def getMeasurementMode(self):
        '''Gets current measurement mode from magnetometer

        CAUTION: Once this measurement is read, it must change before the lock is released.
        '''
        return self.measurement_mode_inv[(self.i2c1.read_byte_data(self.addr, 0x02) & 3)]

    def setMeasurementMode(self, mode):
        '''Select a measurement mode for the magnetometer to run in

        Modes available:
         *  Continuous: gets measurements at given rate and places in data register.
         *  Single: gets a single measurement and places in data register, then enters idle mode.
         *  Idle: No measurements are taken. I2C is available, but the chip is in power save.
        '''
        assert mode in self.measurement_mode, "Selected non-valid measurement mode, see docstring"
        old = self.i2c1.read_byte_data(self.addr, 0x02)
        self.i2c1.write_byte_data(self.addr, 0x02, (old & 0xFC) | (self.measurement_mode[mode]))

