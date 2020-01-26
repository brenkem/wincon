# sources
# datasheet: https://produktinfo.conrad.com/datenblaetter/500000-524999/505671-da-01-de-DIGITALER_FEUCHTE_TEMP_SENSOR_HYT221.pdf
# code: https://www.raspberrypi.org/forums/viewtopic.php?t=24765

# That's the I2C bus module we need
import smbus

# This is the address of the HYT221 sensor on I2C bus #1 (on Rev.2 boards)
# you can check the address with $ i2cdetect -y 1
add = 0x28

# This is our array for the sensor data
ansa = bytearray()

# Create a smbus object on I2C bus #1
hyt = smbus.SMBus(1)


# Init the array just to make sure we have 4 bytes available -
# I'm paranoid!
ansa.append(0x30)
ansa.append(0x31)
ansa.append(0x32)
ansa.append(0x33)

# Init HYT 221 for reading, ignore answer
ans = hyt.read_byte_data(add,0)

# Read 4 bytes (even more) of data from HYT221
ansa = hyt.read_i2c_block_data(add,4)

# Now we have all data from sensor in the first 4 bytes of 'ansa'
# Look for HYT221 doc to see how to
# extract temperature and humidity from bytes

# Calc humidity in rel.%
hum = ansa[0]<<8 | ansa[1]
hum = hum & 0x3FFF

HI = 100.0*hum/(2**14)

# Calc temperature
ansa[3] = ansa[3] & 0x3F
temp = ansa[2] << 6 | ansa[3]

TI = 165.0*temp/(2**14)-40

#Print result
#print "H =",HI,"T =",TI
print "T: %0.1f" % TI,"H: %0.1f" % HI
