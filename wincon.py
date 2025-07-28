#!/usr/bin/env python

import bluepy.btle as btle
import datetime
import signal
import struct
#
import sys
import time
import smbus
import RPi.GPIO as GPIO
from tentacle_pi.AM2315 import AM2315

debug = 1

## display
# Zuordnung der GPIO Pins (ggf. anpassen)
LCD_RS = 4
LCD_E  = 23
LCD_DATA4 = 24
LCD_DATA5 = 25
LCD_DATA6 = 8
LCD_DATA7 = 7

LCD_WIDTH = 16          # Zeichen je Zeile
LCD_LINE_1 = 0x80       # Adresse der ersten Display Zeile
LCD_LINE_2 = 0xC0       # Adresse der zweiten Display Zeile
LCD_CHR = GPIO.HIGH
LCD_CMD = GPIO.LOW
E_PULSE = 0.0005
E_DELAY = 0.0005

def lcd_send_byte(bits, mode):
        # Pins auf LOW setzen
        GPIO.output(LCD_RS, mode)
        GPIO.output(LCD_DATA4, GPIO.LOW)
        GPIO.output(LCD_DATA5, GPIO.LOW)
        GPIO.output(LCD_DATA6, GPIO.LOW)
        GPIO.output(LCD_DATA7, GPIO.LOW)
        if bits & 0x10 == 0x10:
          GPIO.output(LCD_DATA4, GPIO.HIGH)
        if bits & 0x20 == 0x20:
          GPIO.output(LCD_DATA5, GPIO.HIGH)
        if bits & 0x40 == 0x40:
          GPIO.output(LCD_DATA6, GPIO.HIGH)
        if bits & 0x80 == 0x80:
          GPIO.output(LCD_DATA7, GPIO.HIGH)
        time.sleep(E_DELAY)
        GPIO.output(LCD_E, GPIO.HIGH)
        time.sleep(E_PULSE)
        GPIO.output(LCD_E, GPIO.LOW)
        time.sleep(E_DELAY)
        GPIO.output(LCD_DATA4, GPIO.LOW)
        GPIO.output(LCD_DATA5, GPIO.LOW)
        GPIO.output(LCD_DATA6, GPIO.LOW)
        GPIO.output(LCD_DATA7, GPIO.LOW)
        if bits&0x01==0x01:
          GPIO.output(LCD_DATA4, GPIO.HIGH)
        if bits&0x02==0x02:
          GPIO.output(LCD_DATA5, GPIO.HIGH)
        if bits&0x04==0x04:
          GPIO.output(LCD_DATA6, GPIO.HIGH)
        if bits&0x08==0x08:
          GPIO.output(LCD_DATA7, GPIO.HIGH)
        time.sleep(E_DELAY)
        GPIO.output(LCD_E, GPIO.HIGH)
        time.sleep(E_PULSE)
        GPIO.output(LCD_E, GPIO.LOW)
        time.sleep(E_DELAY)

def display_init():
        lcd_send_byte(0x33, LCD_CMD)
        lcd_send_byte(0x32, LCD_CMD)
        lcd_send_byte(0x28, LCD_CMD)
        lcd_send_byte(0x0C, LCD_CMD)
        lcd_send_byte(0x06, LCD_CMD)
        lcd_send_byte(0x01, LCD_CMD)

def lcd_message(message):
        message = message.ljust(LCD_WIDTH," ")
        for i in range(LCD_WIDTH):
          lcd_send_byte(ord(message[i]),LCD_CHR)

###########################################################
## indoor sensor
# https://airthings.com
# wave GEN1 readout via bluetooth low energy

SERIALNUMBER = 2900071413 # keller sensor from airthings

class Wave():

    DATETIME_UUID = btle.UUID(0x2A08)
    HUMIDITY_UUID = btle.UUID(0x2A6F)
    TEMPERATURE_UUID = btle.UUID(0x2A6E)
    RADON_STA_UUID = btle.UUID("b42e01aa-ade7-11e4-89d3-123b93f75cba")
    RADON_LTA_UUID = btle.UUID("b42e0a4c-ade7-11e4-89d3-123b93f75cba")

    def __init__(self, serial_number):
        self._periph = None
        self._datetime_char = None
        self._humidity_char = None
        self._temperature_char = None
        self._radon_sta_char = None
        self._radon_lta_char = None
        self.serial_number = serial_number
        self.mac_addr = None

    def is_connected(self):
        try:
            return self._periph.getState() == "conn"
        except Exception:
            return False

    def discover(self):
        scan_interval = 0.1
        timeout = 3
        scanner = btle.Scanner()
        for _count in range(int(timeout / scan_interval)):
            advertisements = scanner.scan(scan_interval)
            for adv in advertisements:
                if self.serial_number == _parse_serial_number(adv.getValue(btle.ScanEntry.MANUFACTURER)):
                    return adv.addr
        return None

    def connect(self, retries=1):
        tries = 0
        while (tries < retries and self.is_connected() is False):
            tries += 1
            if self.mac_addr is None:
                self.mac_addr = self.discover()
            try:
                self._periph = btle.Peripheral(self.mac_addr)
                self._datetime_char = self._periph.getCharacteristics(uuid=self.DATETIME_UUID)[0]
                self._humidity_char = self._periph.getCharacteristics(uuid=self.HUMIDITY_UUID)[0]
                self._temperature_char = self._periph.getCharacteristics(uuid=self.TEMPERATURE_UUID)[0]
                self._radon_sta_char = self._periph.getCharacteristics(uuid=self.RADON_STA_UUID)[0]
                self._radon_lta_char = self._periph.getCharacteristics(uuid=self.RADON_LTA_UUID)[0]
            except Exception:
                if tries == retries:
                    raise
                else:
                    pass

    def read(self):
        rawdata = self._datetime_char.read()
        rawdata += self._humidity_char.read()
        rawdata += self._temperature_char.read()
        rawdata += self._radon_sta_char.read()
        rawdata += self._radon_lta_char.read()
        return CurrentValues.from_bytes(rawdata)

    def disconnect(self):
        if self._periph is not None:
            self._periph.disconnect()
            self._periph = None
            self._datetime_char = None
            self._humidity_char = None
            self._temperature_char = None
            self._radon_sta_char = None
            self._radon_lta_char = None


class CurrentValues():

    def __init__(self, timestamp, humidity, temperature, radon_sta, radon_lta):
        self.timestamp = timestamp
        self.humidity = humidity
        self.temperature = temperature
        self.radon_sta = radon_sta  # Short term average
        self.radon_lta = radon_lta  # Long term average

    @classmethod
    def from_bytes(cls, rawdata):
        data = struct.unpack('<H5B4H', rawdata)
        timestamp = datetime.datetime(data[0], data[1], data[2], data[3], data[4], data[5])
        return cls(timestamp, data[6]/100.0, data[7]/100.0, data[8], data[9])

    def __str__(self):
        msg = "Timestamp: {}, ".format(self.timestamp)
        msg += "Humidity: {} %rH, ".format(self.humidity)
        msg += "Temperature: {} *C, ".format(self.temperature)
        msg += "Radon STA: {} Bq/m3, ".format(self.radon_sta)
        msg += "Radon LTA: {} Bq/m3".format(self.radon_lta)
        return msg


def _parse_serial_number(manufacturer_data):
    try:
        (ID, SN, _) = struct.unpack("<HLH", manufacturer_data)
    except Exception:  # Return None for non-Airthings devices
        return None
    else:  # Executes only if try-block succeeds
        if ID == 0x0334:
            return SN

###########################################################
def INIT_GPIOs():
  # check to see if the pin is already exported
  ## Gefrierraumfenster
  try:
    # check to see if the pin is already exported
    open('/sys/class/gpio/gpio5/direction').read()
  except:
    # it isn't, so export it
    open('/sys/class/gpio/export', 'w').write('5')

    # set gpiochip87 ID=0  on header pin 31 to output
    open('/sys/class/gpio/gpio5/direction', 'w').write('out')

    # set default value to 1 (Windows closed)
    open('/sys/class/gpio/gpio5/value', 'w').write('1')

  ## Kellerbuerofenster
  try:
    # check to see if the pin is already exported
    open('/sys/class/gpio/gpio6/direction').read()
  except:
    # it isn't, so export it
    open('/sys/class/gpio/export', 'w').write('6')

    # set pin to output
    open('/sys/class/gpio/gpio6/direction', 'w').write('out')

    # set default value to 1 (Windows closed)
    open('/sys/class/gpio/gpio6/value', 'w').write('1')

  ## Heizoelkellerfenster
  try:
    # check to see if the pin is already exported
    open('/sys/class/gpio/gpio13/direction').read()
  except:
    # it isn't, so export it
    open('/sys/class/gpio/export', 'w').write('13')

    # set gpiochip87 ID=0  on header pin 31 to output
    open('/sys/class/gpio/gpio13/direction', 'w').write('out')

    # set default value to 1 (Windows closed)
    open('/sys/class/gpio/gpio13/value', 'w').write('1')


  # Kartoffelkellerfenster
  try:
    # check to see if the pin is already exported
    open('/sys/class/gpio/gpio19/direction').read()
  except:
    # it isn't, so export it
    open('/sys/class/gpio/export', 'w').write('19')

    # set gpiochip87 ID=0  on header pin 31 to output
    open('/sys/class/gpio/gpio19/direction', 'w').write('out')

    # set default value to 1 (Windows closed)
    open('/sys/class/gpio/gpio19/value', 'w').write('1')



###########################################################
# main
###########################################################

# init wave sensor for indoor values
wave = Wave(SERIALNUMBER)


# INIT system
## INIT GPIOs
INIT_GPIOs()

## init 16x2 display
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup(LCD_E, GPIO.OUT)
GPIO.setup(LCD_RS, GPIO.OUT)
GPIO.setup(LCD_DATA4, GPIO.OUT)
GPIO.setup(LCD_DATA5, GPIO.OUT)
GPIO.setup(LCD_DATA6, GPIO.OUT)
GPIO.setup(LCD_DATA7, GPIO.OUT)
display_init()

## init sensors and data
sens_A = AM2315(0x5c,"/dev/i2c-1")
sens_K = AM2315(0x5c,"/dev/i2c-3")

while True:
  # read sensor data
  TA, HA, crcA_check = sens_A.sense()
  TK, HK, crcK_check = sens_K.sense()
  wave.connect(retries=3)
  indoor_val = wave.read()
  TI = indoor_val.temperature
  HI = indoor_val.humidity
  RS = indoor_val.radon_sta
  RL = indoor_val.radon_lta
#  print("I= T: %0.1f" % indoor_val.temperature, "H: %0.1f" % indoor_val.humidity, "Rs: %d" % indoor_val.radon_sta, "Rl: %d" % indoor_val.radon_lta)

  # check if data validity
  if crcA_check == 1 and crcK_check == 1:
    if debug == 1:
      #Print result
      print("A= T: %0.1f" % TA, "H: %0.1f" % HA, "crc_ok: %s" % (crcA_check == 1))
      print("I= T: %0.1f" % TI, "H: %0.1f" % HI)
      print("K= T: %0.1f" % TK, "H: %0.1f" % HK, "crc_ok: %s" % (crcK_check == 1))


    # prepare for humidity relation calculation
    if (TA > 0):
      aa = 7.5
      ba = 237.3
    else:
      aa = 9.5
      ba = 265.5

    if (TK > 0):
      ai = 7.5
      bi = 237.3
    else:
      ai = 9.5
      bi = 265.5

    # Wassermengenverhaeltnis nach http://www.wetterochs.de/wetter/feuchte.html
    VK  = ((10**(((ai*TK)/(bi+TI))-((aa*TA)/(ba+TA))))*HK*(TA+273.15))/(HA*(TK+273.15))
    if debug == 1:
      print("VK: %0.3f" % VK)

    ################### control Kartoffelkeller ############################
    # Kaftoffelkellerfensterstatus: 0 wenn geoeffnet; 1 wenn geschlossen
    statK = int(open('/sys/class/gpio/gpio19/value', 'r').read())

    # save log
    open('/dev/shm/wetterstation_HK', 'w').write("%.1f" % HK)
    open('/dev/shm/wetterstation_TK', 'w').write("%.1f" % TK)

    # check clima and control window
    if (statK) and (VK > 1.3) and (HK >= 80) and ( ((TK < 6) and (TA > (TK + 5))) or ((TK > 8) and (TA < (TK - 5))) ):
      open('/sys/class/gpio/gpio19/value', 'w').write("0") # open  Kartoffelkellerfenster
      open('/dev/shm/wincon.log', 'a').write(time.strftime("%d.%m.%Y %H:%M: ") + "Kartoffelkeller geoeffnet " + "TK:%.1f, HK:%.1f; TA:%.1f, HA:%.1f, VK:%.3f\n" % (TK, HK, TA, HA, VK))
    if (not statK) and ( (VK < 1.1) or (HK < 70) or ((TK <= (TA + 3)) and (TK >= (TA - 3))) or ((TK > 6.5) and (TA > TK)) or ((TK < 6.5) and (TA < TK)) ):
      open('/sys/class/gpio/gpio19/value', 'w').write("1") # close Kartoffelkellerfenster
      open('/dev/shm/wincon.log', 'a').write(time.strftime("%d.%m.%Y %H:%M: ") + "Kartoffelkeller geschlossen " + "TK:%.1f, HK:%.1f; TA:%.1f, HA:%.1f, VK:%.3f\n" % (TK, HK, TA, HA, VK))


    ###################### Kellerlueftung ####################################
    # Fensterstatus: 0 bei geoeffnet und 1 bei geschlossen
    stat = int(open('/sys/class/gpio/gpio5/value', 'r').read())

    # prepare for humidity relation calculation
    if (TA > 0):
      aa = 7.5
      ba = 237.3
    else:
      aa = 9.5
      ba = 265.5

    if (TI > 0):
      ai = 7.5
      bi = 237.3
    else:
      ai = 9.5
      bi = 265.5

    # Wassermengenverhaeltnis nach http://www.wetterochs.de/wetter/feuchte.html
    V  = ((10**(((ai*TI)/(bi+TI))-((aa*TA)/(ba+TA))))*HI*(TA+273.15))/(HA*(TI+273.15))
    if debug == 1:
      print("V: %0.3f" % V)

    # Zwischenspeichern der Temperatur- und Luftfeuchtigkeitswerte im SHM
    open('/dev/shm/wetterstation_HI', 'w').write("%0.1f" % HI)
    open('/dev/shm/wetterstation_HA', 'w').write("%0.1f" % HA)
    open('/dev/shm/wetterstation_TI', 'w').write("%0.1f" % TI)
    open('/dev/shm/wetterstation_TA', 'w').write("%0.1f" % TA)
    open('/dev/shm/wetterstation_VK', 'w').write("%0.3f" % VK)
    open('/dev/shm/wetterstation_V', 'w').write("%0.3f" % V)

    # print current data to display
    lcd_send_byte(LCD_LINE_1, LCD_CMD)
    lcd_message("A%2d%%" % HA + "  I%2d%%" % HI + "  K%2d%%" % HK)
    lcd_send_byte(LCD_LINE_2, LCD_CMD)
    lcd_message("%4.1f" % TA + " %5.1f" % TI  + " %5.1f" % TK)

    # Auswertung der Fensteransteuerung
    # open windows
    if (statK) and (stat) and (V > 1.2) and (TI > 19) and (HA < 80) and ((TA < TI) or (TI < 21)):
      open('/sys/class/gpio/gpio5/value', 'w').write("0") # open Gefrierraumfenster
      time.sleep(15)
      open('/sys/class/gpio/gpio6/value', 'w').write("0") # open Kellerbuerofenster
      time.sleep(15)
      open('/sys/class/gpio/gpio13/value', 'w').write("0") # open Heizoelkellerfenster
      time.sleep(15)
      open('/dev/shm/wincon.log', 'a').write(time.strftime("%d.%m.%Y %H:%M: Kellerfenster geoeffnet. " + "TI:%.1f, HI:%.1f; TA:%.1f, HA:%.1f, V:%.3f\n" % (TI, HI, TA, HA, V)))

    # close windows
    if ((not statK) and (not stat)) or ((not stat) and ((V <= 1.1) or (TI <= 18) or (HA >= 85) or ((TA > TI) and (TI >= 24)))):
      open('/sys/class/gpio/gpio5/value', 'w').write("1") # close Gefrierraumfenster
      time.sleep(15)
      open('/sys/class/gpio/gpio6/value', 'w').write("1") # close Kellerbuerofenster
      time.sleep(15)
      open('/sys/class/gpio/gpio13/value', 'w').write("1") # close Heizoelkellerfenster
      time.sleep(15)
      open('/dev/shm/wincon.log', 'a').write(time.strftime("%d.%m.%Y %H:%M: Kellerfenster geschlossen. " + "TI:%.1f, HI:%.1f; TA:%.1f, HA:%.1f, V:%.3f\n\n" % (TI, HI, TA, HA, V)))

  # error
  elif crcA_check != 1:
    open('/dev/shm/wetterstation_TA', 'w').write("-0") # error code
    open('/dev/shm/wetterstation_HA', 'w').write("-0") # error code
  elif crcK_check != 1:
    open('/dev/shm/wetterstation_TK', 'w').write("-0") # error code
    open('/dev/shm/wetterstation_HK', 'w').write("-0") # error code

  wave.disconnect()

  time.sleep(60) # wait to check again
