#!/usr/bin/env python

import sys
import time
import smbus
from tentacle_pi.AM2315 import AM2315


def HYT221(addr, bus):
    hyt = smbus.SMBus(bus)
    ansa = bytearray()

    # Init HYT 221 for reading, ignore answer
    hyt.read_byte_data(addr,0)

    # Read 4 bytes (even more) of data from HYT221
    ansa = hyt.read_i2c_block_data(addr,4)

    # Calc humidity in rel.%
    hum = ansa[0]<<8 | ansa[1]
    hum = hum & 0x3FFF
    H = 100.0*hum/(2**14)

    if H < 0 or H > 100:
        return -0, -0, 0

    # Calc temperature
    ansa[3] = ansa[3] & 0x3F
    temp = ansa[2] << 6 | ansa[3]
    T = 165.0*temp/(2**14)-40

    if T < -40 or T > 125:
        return -0, -0, 0

    # return valid data
    return T, H, 1


########################################
# main
########################################

sens_A = AM2315(0x5c,"/dev/i2c-1")
sens_K = AM2315(0x5c,"/dev/i2c-3")

TA, HA, crcA_check = sens_A.sense()
TK, HK, crcK_check = sens_K.sense()
TI, HI, validI_check = HYT221(0x28,1)


#Print result
if crcA_check == 1 and crcK_check == 1 and validI_check == 1:
    print "A= T: %0.1f" % TA,"H: %0.1f" % HA, "crc_ok: %s" % (crcA_check == 1)
    print "K= T: %0.1f" % TK,"H: %0.1f" % HK, "crc_ok: %s" % (crcK_check == 1)
    print "I= T: %0.1f" % TI,"H: %0.1f" % HI, "crc_ok: %s" % (validI_check == 1)
