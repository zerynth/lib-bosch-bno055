"""
.. module:: bno055

*************
BNO055 Module
*************

This module contains the driver for BOSCH BNO055 9-axis Absolute Orientation Sensor. The BNO055 is a System in Package (SiP), integrating a triaxial 14-bit accelerometer, a triaxial 16-bit gyroscope with a range of ±2000 degrees per second, a triaxial geomagnetic sensor (`datasheet <https://ae-bst.resource.bosch.com/media/_tech/media/datasheets/BST_BNO055_DS000_14.pdf>`_).
    """


import i2c

# I2C addresses
BNO055_ADDRESS_A                     = 0x28
BNO055_ADDRESS_B                     = 0x29
BNO055_ID                            = 0xA0

# Page id register definition
BNO055_PAGE_ID_ADDR                  = 0X07

# PAGE0 REGISTER DEFINITION START
BNO055_CHIP_ID_ADDR                  = 0x00
BNO055_ACCEL_REV_ID_ADDR             = 0x01
BNO055_MAG_REV_ID_ADDR               = 0x02
BNO055_GYRO_REV_ID_ADDR              = 0x03
BNO055_SW_REV_ID_LSB_ADDR            = 0x04
BNO055_SW_REV_ID_MSB_ADDR            = 0x05
BNO055_BL_REV_ID_ADDR                = 0X06

# Accel data register
BNO055_ACCEL_DATA_X_LSB_ADDR         = 0X08
BNO055_ACCEL_DATA_X_MSB_ADDR         = 0X09
BNO055_ACCEL_DATA_Y_LSB_ADDR         = 0X0A
BNO055_ACCEL_DATA_Y_MSB_ADDR         = 0X0B
BNO055_ACCEL_DATA_Z_LSB_ADDR         = 0X0C
BNO055_ACCEL_DATA_Z_MSB_ADDR         = 0X0D

# Mag data register
BNO055_MAG_DATA_X_LSB_ADDR           = 0X0E
BNO055_MAG_DATA_X_MSB_ADDR           = 0X0F
BNO055_MAG_DATA_Y_LSB_ADDR           = 0X10
BNO055_MAG_DATA_Y_MSB_ADDR           = 0X11
BNO055_MAG_DATA_Z_LSB_ADDR           = 0X12
BNO055_MAG_DATA_Z_MSB_ADDR           = 0X13

# Gyro data registers
BNO055_GYRO_DATA_X_LSB_ADDR          = 0X14
BNO055_GYRO_DATA_X_MSB_ADDR          = 0X15
BNO055_GYRO_DATA_Y_LSB_ADDR          = 0X16
BNO055_GYRO_DATA_Y_MSB_ADDR          = 0X17
BNO055_GYRO_DATA_Z_LSB_ADDR          = 0X18
BNO055_GYRO_DATA_Z_MSB_ADDR          = 0X19

# Euler data registers
BNO055_EULER_H_LSB_ADDR              = 0X1A
BNO055_EULER_H_MSB_ADDR              = 0X1B
BNO055_EULER_R_LSB_ADDR              = 0X1C
BNO055_EULER_R_MSB_ADDR              = 0X1D
BNO055_EULER_P_LSB_ADDR              = 0X1E
BNO055_EULER_P_MSB_ADDR              = 0X1F

# Quaternion data registers
BNO055_QUATERNION_DATA_W_LSB_ADDR    = 0X20
BNO055_QUATERNION_DATA_W_MSB_ADDR    = 0X21
BNO055_QUATERNION_DATA_X_LSB_ADDR    = 0X22
BNO055_QUATERNION_DATA_X_MSB_ADDR    = 0X23
BNO055_QUATERNION_DATA_Y_LSB_ADDR    = 0X24
BNO055_QUATERNION_DATA_Y_MSB_ADDR    = 0X25
BNO055_QUATERNION_DATA_Z_LSB_ADDR    = 0X26
BNO055_QUATERNION_DATA_Z_MSB_ADDR    = 0X27

# Linear acceleration data registers
BNO055_LINEAR_ACCEL_DATA_X_LSB_ADDR  = 0X28
BNO055_LINEAR_ACCEL_DATA_X_MSB_ADDR  = 0X29
BNO055_LINEAR_ACCEL_DATA_Y_LSB_ADDR  = 0X2A
BNO055_LINEAR_ACCEL_DATA_Y_MSB_ADDR  = 0X2B
BNO055_LINEAR_ACCEL_DATA_Z_LSB_ADDR  = 0X2C
BNO055_LINEAR_ACCEL_DATA_Z_MSB_ADDR  = 0X2D

# Gravity data registers
BNO055_GRAVITY_DATA_X_LSB_ADDR       = 0X2E
BNO055_GRAVITY_DATA_X_MSB_ADDR       = 0X2F
BNO055_GRAVITY_DATA_Y_LSB_ADDR       = 0X30
BNO055_GRAVITY_DATA_Y_MSB_ADDR       = 0X31
BNO055_GRAVITY_DATA_Z_LSB_ADDR       = 0X32
BNO055_GRAVITY_DATA_Z_MSB_ADDR       = 0X33

# Temperature data register
BNO055_TEMP_ADDR                     = 0X34

# Status registers
BNO055_CALIB_STAT_ADDR               = 0X35
BNO055_SELFTEST_RESULT_ADDR          = 0X36
BNO055_INTR_STAT_ADDR                = 0X37

BNO055_SYS_CLK_STAT_ADDR             = 0X38
BNO055_SYS_STAT_ADDR                 = 0X39
BNO055_SYS_ERR_ADDR                  = 0X3A

# Unit selection register
BNO055_UNIT_SEL_ADDR                 = 0X3B
BNO055_DATA_SELECT_ADDR              = 0X3C

# Mode registers
BNO055_OPR_MODE_ADDR                 = 0X3D
BNO055_PWR_MODE_ADDR                 = 0X3E

BNO055_SYS_TRIGGER_ADDR              = 0X3F
BNO055_TEMP_SOURCE_ADDR              = 0X40

# Axis remap registers
BNO055_AXIS_MAP_CONFIG_ADDR          = 0X41
BNO055_AXIS_MAP_SIGN_ADDR            = 0X42

# Axis remap values
AXIS_REMAP_X                         = 0x00
AXIS_REMAP_Y                         = 0x01
AXIS_REMAP_Z                         = 0x02
AXIS_REMAP_POSITIVE                  = 0x00
AXIS_REMAP_NEGATIVE                  = 0x01

# SIC registers
BNO055_SIC_MATRIX_0_LSB_ADDR         = 0X43
BNO055_SIC_MATRIX_0_MSB_ADDR         = 0X44
BNO055_SIC_MATRIX_1_LSB_ADDR         = 0X45
BNO055_SIC_MATRIX_1_MSB_ADDR         = 0X46
BNO055_SIC_MATRIX_2_LSB_ADDR         = 0X47
BNO055_SIC_MATRIX_2_MSB_ADDR         = 0X48
BNO055_SIC_MATRIX_3_LSB_ADDR         = 0X49
BNO055_SIC_MATRIX_3_MSB_ADDR         = 0X4A
BNO055_SIC_MATRIX_4_LSB_ADDR         = 0X4B
BNO055_SIC_MATRIX_4_MSB_ADDR         = 0X4C
BNO055_SIC_MATRIX_5_LSB_ADDR         = 0X4D
BNO055_SIC_MATRIX_5_MSB_ADDR         = 0X4E
BNO055_SIC_MATRIX_6_LSB_ADDR         = 0X4F
BNO055_SIC_MATRIX_6_MSB_ADDR         = 0X50
BNO055_SIC_MATRIX_7_LSB_ADDR         = 0X51
BNO055_SIC_MATRIX_7_MSB_ADDR         = 0X52
BNO055_SIC_MATRIX_8_LSB_ADDR         = 0X53
BNO055_SIC_MATRIX_8_MSB_ADDR         = 0X54

# Accelerometer Offset registers
ACCEL_OFFSET_X_LSB_ADDR              = 0X55
ACCEL_OFFSET_X_MSB_ADDR              = 0X56
ACCEL_OFFSET_Y_LSB_ADDR              = 0X57
ACCEL_OFFSET_Y_MSB_ADDR              = 0X58
ACCEL_OFFSET_Z_LSB_ADDR              = 0X59
ACCEL_OFFSET_Z_MSB_ADDR              = 0X5A

# Magnetometer Offset registers
MAG_OFFSET_X_LSB_ADDR                = 0X5B
MAG_OFFSET_X_MSB_ADDR                = 0X5C
MAG_OFFSET_Y_LSB_ADDR                = 0X5D
MAG_OFFSET_Y_MSB_ADDR                = 0X5E
MAG_OFFSET_Z_LSB_ADDR                = 0X5F
MAG_OFFSET_Z_MSB_ADDR                = 0X60

# Gyroscope Offset register s
GYRO_OFFSET_X_LSB_ADDR               = 0X61
GYRO_OFFSET_X_MSB_ADDR               = 0X62
GYRO_OFFSET_Y_LSB_ADDR               = 0X63
GYRO_OFFSET_Y_MSB_ADDR               = 0X64
GYRO_OFFSET_Z_LSB_ADDR               = 0X65
GYRO_OFFSET_Z_MSB_ADDR               = 0X66

# Radius registers
ACCEL_RADIUS_LSB_ADDR                = 0X67
ACCEL_RADIUS_MSB_ADDR                = 0X68
MAG_RADIUS_LSB_ADDR                  = 0X69
MAG_RADIUS_MSB_ADDR                  = 0X6A

# pwr modes p. 19
# Power modes
POWER_MODE_NORMAL                    = 0X00
POWER_MODE_LOWPOWER                  = 0X01
POWER_MODE_SUSPEND                   = 0X02


# opr modes p. 21
# Operation mode settings
OPERATION_MODE_CONFIG                = 0X00
OPERATION_MODE_ACCONLY               = 0X01
OPERATION_MODE_MAGONLY               = 0X02
OPERATION_MODE_GYRONLY               = 0X03
OPERATION_MODE_ACCMAG                = 0X04
OPERATION_MODE_ACCGYRO               = 0X05
OPERATION_MODE_MAGGYRO               = 0X06
OPERATION_MODE_AMG                   = 0X07
OPERATION_MODE_IMU                   = 0X08
OPERATION_MODE_COMPASS               = 0X09
OPERATION_MODE_M4G                   = 0X0A
OPERATION_MODE_NDOF_FMC_OFF          = 0X0B
OPERATION_MODE_NDOF                  = 0X0C

_modes = {
    "acc" : OPERATION_MODE_ACCONLY,
    "magn": OPERATION_MODE_MAGONLY,
    "gyro": OPERATION_MODE_GYRONLY,
    "accmag": OPERATION_MODE_ACCMAG,
    "accgyro": OPERATION_MODE_ACCGYRO,
    "maggyro": OPERATION_MODE_MAGGYRO,
    "amg": OPERATION_MODE_AMG,
    "imu": OPERATION_MODE_IMU,
    "comp": OPERATION_MODE_COMPASS,
    "m4g": OPERATION_MODE_M4G,
    "ndof_off": OPERATION_MODE_NDOF_FMC_OFF,
    "ndof": OPERATION_MODE_NDOF
}


class BNO055(i2c.I2C):
    """

.. class:: BNO055(i2cdrv, addr=0x28, clk=100000)

    Creates an intance of a new BNO055.

    :param i2cdrv: I2C Bus used '( I2C0, ... )'
    :param addr: Slave address, default 0x28
    :param clk: Clock speed, default 100kHz

    Example: ::

        from bosch.bno055 import bno055

        ...

        bno = bno055.BNO055(I2C0)
        bno.start()
        bno.init()
        abs_orientation = bno.get_euler()

    """

    def __init__(self, i2cdrv, addr=0x28, clk=100000):
        i2c.I2C.__init__(self,i2cdrv,addr,clk)
        self._addr = addr
        
    def init(self, mode = None):
        """

.. method:: init(mode=None)

        Initialize the BNO055 setting the mode value.

        :param mode: Mode value selectable from Mode table (refer to page 20 of the BNO055 datasheet), default None

        """

        self.write_bytes(BNO055_PWR_MODE_ADDR, POWER_MODE_NORMAL)
        sleep(10) 

        self.write_bytes(BNO055_UNIT_SEL_ADDR, 0)
        if mode == None:
            self.set_mode("ndof")
        else:
            self.set_mode(mode)

    def set_mode(self, mode):
        """

.. method:: set_mode(mode)

        Sets the Mode value of the BNO055.

        :param mode: Mode value (refer to page 20 of the BNO055 datasheet)

.. note:: The BNO055 provides a variety of output signals, which can be chosen by selecting the appropriate operation mode. The table below lists the different modes and the available sensor signals.

========== ============ ========= ========== ========== ===========
         Mode           Available Library Functions     
----------------------- -------------------------------------------
Value      Name         get_acc() get_magn() get_gyro() get_euler()
========== ============ ========= ========== ========== ===========
acc        ACCONLY      Yes       No         No         No  
magn       MAGONLY      No        Yes        No         No  
gyro       GYROONLY     No        No         Yes        No 
accmag     ACCMAG       Yes       Yes        No         No 
accgyro    ACCGYRO      Yes       No         Yes        No 
maggyro    MAGGYRO      No        Yes        Yes        No 
amg        AMG          No        Yes        Yes        No 
imu        IMU          Yes       No         Yes        Yes
comp       COMPASS      Yes       Yes        No         Yes
m4g        M4G          Yes       Yes        Yes        Yes
ndof_off   NDOF_FMC_OFF No        Yes        Yes        Yes
ndof       NDOF         Yes       Yes        Yes        Yes
========== ============ ========= ========== ========== ===========
                     
        """
        if mode not in _modes:
            raise ValueError
        self.mode = mode
        self.write_bytes(BNO055_OPR_MODE_ADDR, OPERATION_MODE_CONFIG)
        sleep(25) # p.21 switching time
        self.write_bytes(BNO055_OPR_MODE_ADDR, _modes[self.mode])
        sleep(15) # p.21 switching time

    def _sum_and_sign(self, lsb, msb):
        res = ((lsb | (msb << 8)) & 0xFFFF)
        if res > 32767:
            res -= 65536
        return res

    def _convert_in_lsb_msb(self, value):
        if value < 0:
            value += 65536
        if value > 65535:
            raise ValueError
        lsb = int(value) & 0x00FF
        msb = (int(value) >> 8) & 0x00FF
        return lsb,msb

    def _read_vector(self, lsb_addr, count = 3):
        data = self.write_read(lsb_addr, count*2)
        res = [0]*count
        for i in range(count):
            res[i] = self._sum_and_sign(data[i*2], data[i*2+1])
        return res

    def _write_vector(self, lsb_addr, data, count = 11):
        rawdata = [0x00]*count*2
        for i in range(count):
            rawdata[i*2],rawdata[i*2+1] = self._convert_in_lsb_msb(data[i])
        self.write_bytes(lsb_addr, *rawdata)

    def get_calibration_status(self):
        """

.. method:: get_calibration_status()

        Retrieves the current calibration status of the BNO055 main components:

        * System
        * Accelerometer
        * Gyroscope
        * Magnetometer

        .. note:: Read: 3 indicates fully calibrated; 0 indicates not calibrated.

        Returns [sys_cal_sts, acc_cal_sts, gyro_cal_sts, magn_cal_sts]

        """
        sts = self.write_read(BNO055_CALIB_STAT_ADDR, 1)[0]
        sys = (sts >> 6) & 0x03
        acc = (sts >> 2) & 0x03
        gyro = (sts >> 4) & 0x03
        magn = sts & 0x03
        return [sys, acc, gyro, magn]

    def get_calibration(self, raw=False):
        """

.. method:: get_calibration(raw=False)

        Retrieves the calibration values of the BMO055 main components (list of 11 elements):

        * Accelerometer Offset for X, Y, Z axes (values in m/s²) - list elements 0,1,2;
        * Magnetometer Offset for X, Y, Z axes (values in uT) - list elements 3,4,5;
        * Gyroscope Offset for X, Y, Z axes (values in Dps) - list elements 6,7,8;
        * Accelerometer Radius - list element 9;
        * Magnetometer Radius - list element 10.

        .. note:: If raw parameter is set to True, returns a list of 22 raw bytes. 

        Returns [list of calibration values]

        """
        self.write_bytes(BNO055_OPR_MODE_ADDR, OPERATION_MODE_CONFIG)
        sleep(25)
        if raw:
            cdata = self.write_read(ACCEL_OFFSET_X_LSB_ADDR, 22)
        else:
            cdata = self._read_vector(ACCEL_OFFSET_X_LSB_ADDR, 11)
            cdata[0] = cdata[0]/100
            cdata[1] = cdata[1]/100
            cdata[2] = cdata[2]/100
            cdata[3] = cdata[3]/16
            cdata[4] = cdata[4]/16
            cdata[5] = cdata[5]/16
            cdata[6] = cdata[6]/900
            cdata[7] = cdata[7]/900
            cdata[8] = cdata[8]/900
            cdata[9] = cdata[9]/1000
            cdata[10] = cdata[10]/960
        self.set_mode(self.mode)
        return cdata

    def set_calibration(self, data, raw=False):
        """

.. method:: set_calibration(data, raw=False)

        Sets the calibration values of the BNO055 main components.
        
        :param data: List of values (11 elements) representing the sensors offsets and radius.
 
        data list must follow this order:

        * Accelerometer Offset for X, Y, Z axes (values in m/s²) - list elements 0,1,2;
        * Magnetometer Offset for X, Y, Z axes (values in uT) - list elements 3,4,5;
        * Gyroscope Offset for X, Y, Z axes (values in Dps) - list elements 6,7,8;
        * Accelerometer Radius - list element 9;
        * Magnetometer Radius - list element 10.  

        .. note:: If raw parameter is set to True, following rules are required:

                    * data list must have 22 elements;
                    * each element must be a byte (value 0 to 255);
                    * data list must be a sequence of [lsb1, msb1, lsb2, msb2, ..., ...];
                    * data list order is the same described above (elem0 and elem1 of data list are respectively lsb and msb of accelerometer offset in x axis).

        """
        self.write_bytes(BNO055_OPR_MODE_ADDR, OPERATION_MODE_CONFIG)
        sleep(25)
        if raw:
            if len(data) != 22:
                raise ValueError
            for elem in data:
                if elem > 255:
                    raise ValueError
            self.write_bytes(ACCEL_OFFSET_X_LSB_ADDR, *data)
        else:
            if len(data) != 11:
                raise ValueError
            data[0] = data[0]*100
            data[1] = data[1]*100
            data[2] = data[2]*100
            data[3] = data[3]*16
            data[4] = data[4]*16
            data[5] = data[5]*16
            data[6] = data[6]*900
            data[7] = data[7]*900
            data[8] = data[8]*900
            data[9] = data[9]*1000
            data[10] = data[10]*960
            self._write_vector(ACCEL_OFFSET_X_LSB_ADDR, data)
        self.set_mode(self.mode)

    def get_acc(self):
        """

.. method:: get_acc()

        Retrieves the current absolute acceleration as a list of X, Y, Z values in m/s²

        Returns [acc_x, acc_y, acc_z]

        """
        xx = self._read_vector(BNO055_ACCEL_DATA_X_LSB_ADDR)
        xyz = [ i/100 for i in xx ]
        return xyz

    def get_gyro(self):
        """

.. method:: get_gyro()

        Retrieves the current gyroscope data reading as a list of X, Y, Z values in degrees per second 

        Returns [gyro_x, gyro_y, gyro_z]

        """
        xx  = self._read_vector(BNO055_GYRO_DATA_X_LSB_ADDR)
        xyz = [ i/900 for i in xx ]
        return xyz

    def get_magn(self):
        """

.. method:: get_magn()

        Retrieves the current magnetometer reading as a list of X, Y, Z values
        in micro-Teslas.

        Returns [value_magn_x, value_magn_y, value_magn_z]

        """
        xx = self._read_vector(BNO055_MAG_DATA_X_LSB_ADDR)
        xyz = [ i/16 for i in xx ]
        return xyz

    def get_euler(self):
        """

.. method:: get_euler()

        Retrieves the current orientation as a list of heading, roll,
        and pitch euler angles in degrees.

        Returns [abs_or_h, abs_or_r, abs_or_p]

        """
        xx = self._read_vector(BNO055_EULER_H_LSB_ADDR)
        hrp = [ i/16 for i in xx ]
        return hrp

    def get_lin_acc(self):
        """

.. method:: get_lin_acc()

        Retrieves the current linear acceleration (acceleration from movement,
        not from gravity) as a list of X, Y, Z values in m/s²

        Returns [lin_acc_x, lin_acc_y, lin_acc_z]

        """
        xx = self._read_vector(BNO055_LINEAR_ACCEL_DATA_X_LSB_ADDR)
        xyz = [ i/100 for i in xx ]
        return xyz

    def get_grav(self):
        """

.. method:: get_grav()

        Retrieves the current gravity acceleration as a list of X, Y, Z values in m/s²

        Returns [grav_x, grav_y, grav_z]

        """
        xx = self._read_vector(BNO055_GRAVITY_DATA_X_LSB_ADDR)
        xyz = [ i/100 for i in xx ]
        return xyz

    def get_quaternion(self):
        """

.. method:: get_quaternion()

        Retrieves the current orientation as a list of X, Y, Z, W quaternion
        values.

        Returns [w, x, y, z]

        """
        xx = self._read_vector(BNO055_QUATERNION_DATA_W_LSB_ADDR, 4)
        wxyz = [ i*(1.0 / (1<<14)) for i in xx ]
        return wxyz
        
    def get_temp(self):
        """

.. method:: get_temp()

        Retrieves the current temperature in Celtius.

        Returns temp

        """
        raw_t = self.write_read(BNO055_TEMP_ADDR, 1)[0]
        if raw_t > 127:
            return raw_t - 256
        else:
            return raw_t