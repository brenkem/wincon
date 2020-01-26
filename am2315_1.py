import time
from tentacle_pi.AM2315 import AM2315
am = AM2315(0x5c,"/dev/i2c-1")

TA, HA, crcA_check = am.sense()
#print "temperature: %0.1f" % temperature
#print "humidity: %0.1f" % humidity
#print "crc_ok: %s" % (crc_check == 1)
#print "temperature: %0.1f" % TA," humidity: %0.1f" % HA, " crc_ok: %s" % (crcA_check == 1)
print "T: %0.1f" % TA,"H: %0.1f" % HA, "crc_ok: %s" % (crcA_check == 1)
