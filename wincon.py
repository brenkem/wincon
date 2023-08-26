#!/usr/bin/env python

import sys
import time
import smbus
import RPi.GPIO as GPIO
from tentacle_pi.AM2315 import AM2315

debug = 1

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
  TI, HI, validI_check = HYT221(0x28,1)

  # check if data validity
  if crcA_check == 1 and crcK_check == 1 and validI_check == 1:
    if debug == 1:
      #Print result
      print("A= T: %0.1f" % TA, "H: %0.1f" % HA, "crc_ok: %s" % (crcA_check == 1))
      print("I= T: %0.1f" % TI, "H: %0.1f" % HI, "crc_ok: %s" % (validI_check == 1))
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
#    if (statK) and (HK >= 80) and (HA <= 90) and ( ((TK < 5) and (TA > (TK+1))) or ((TK > 8) and (TA < (TK-3))) ):
    if (statK) and (VK > 1.3) and (HK >= 80) and ( ((TK < 6) and (TA > (TK + 5))) or ((TK > 8) and (TA < (TK - 5))) ):
      open('/sys/class/gpio/gpio19/value', 'w').write("0") # open  Kartoffelkellerfenster
      open('/dev/shm/wincon.log', 'a').write(time.strftime("%d.%m.%Y %H:%M: ") + "Kartoffelkeller geoeffnet " + "TK:%.1f, HK:%.1f; TA:%.1f, HA:%.1f, VK:%.3f\n" % (TK, HK, TA, HA, VK))
#    if (not statK) and ( (HA > 95) or (HK < 70) or ((TK <= (TA + 0.5)) and (TK >= (TA - 0.5))) or ((TK > 6.5) and (TA > TK)) or ((TK < 6.5) and (TA < TK)) ):
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
    open('/dev/shm/wetterstation_V', 'w').write("%0.3f" % V)

    # print current data to display
    lcd_send_byte(LCD_LINE_1, LCD_CMD)
    lcd_message("A%2d%%" % HA + "  I%2d%%" % HI + "  K%2d%%" % HK)
    lcd_send_byte(LCD_LINE_2, LCD_CMD)
    lcd_message("%4.1f" % TA + " %5.1f" % TI  + " %5.1f" % TK)

    # Auswertung der Fensteransteuerung
    # open windows
    #if (statK) and (stat) and (V > 1.3) and (TI > 19) and (HA < 80) and ((TA < TI) or (TI < 21)):
    if (statK) and (stat) and (V > 1.2) and (TI > 19) and (HA < 80) and ((TA < TI) or (TI < 21)):
      open('/sys/class/gpio/gpio5/value', 'w').write("0") # open Gefrierraumfenster
      time.sleep(15)
      open('/sys/class/gpio/gpio6/value', 'w').write("0") # open Kellerbuerofenster
      time.sleep(15)
      open('/sys/class/gpio/gpio13/value', 'w').write("0") # open Heizoelkellerfenster
      time.sleep(15)
      open('/dev/shm/wincon.log', 'a').write(time.strftime("%d.%m.%Y %H:%M: Kellerfenster geoeffnet.\n"))

    # close windows
    if ((not statK) and (not stat)) or ((not stat) and ((V <= 1.1) or (TI <= 18) or (HA >= 85) or ((TA > TI) and (TI >= 24)))):
      open('/sys/class/gpio/gpio5/value', 'w').write("1") # close Gefrierraumfenster
      time.sleep(15)
      open('/sys/class/gpio/gpio6/value', 'w').write("1") # close Kellerbuerofenster
      time.sleep(15)
      open('/sys/class/gpio/gpio13/value', 'w').write("1") # close Heizoelkellerfenster
      time.sleep(15)
      open('/dev/shm/wincon.log', 'a').write(time.strftime("%d.%m.%Y %H:%M: Kellerfenster geschlossen.\n\n"))

  # error
  elif crcA_check != 1:
    open('/dev/shm/wetterstation_TA', 'w').write("-0") # error code
    open('/dev/shm/wetterstation_HA', 'w').write("-0") # error code
  elif crcI_check != 1:
    open('/dev/shm/wetterstation_TI', 'w').write("-0") # error code
    open('/dev/shm/wetterstation_HI', 'w').write("-0") # error code
  elif validI_check != 1:
    open('/dev/shm/wetterstation_TK', 'w').write("-0") # error code
    open('/dev/shm/wetterstation_HK', 'w').write("-0") # error code

  time.sleep(60) # wait to check again
