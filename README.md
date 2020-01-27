
The windows control station is an embedded system with multiple sensors
to climatize different locations according there temerature and humidity.


## hardware setup
    SBC:
    ====
    i2c-1:
    ------
    am2315

    i2c-3:
    ------
    am2315

    gpios:
    ======
    Fensteransteuerungen(0/offen, 1/zu):
    ---------------------
    gpio5	Gefrierraum
    gpio6	Kellerbüro
    gpio13	Heizoelkeller
    gpio19	Kartoffelkeller




# set up OS
- install rasbian
- set root passwd and enable ssh and set root access

## set hostname
echo wcs > /etc/hostname

## rm user
deluser pi --remove-home

## fix language
export LANGUAGE=en_US.UTF-8
export LANG=en_US.UTF-8
export LC_ALL=en_US.UTF-8
locale-gen en_US.UTF-8
dpkg-reconfigure locales

## clean system (apt)
apt purge avahi-daemon bluez bluez-firmware crda ca-certificates cifs-utils ed firmware-atheros firmware-brcm80211 firmware-libertas firmware-misc-nonfree firmware-realtek freetype2-doc g++ gdb htop iw javascript-common nfs-common ntfs-3g pciutils pi-bluetooth rpi-eeprom rpi-eeprom-images rfkill sudo wireless-regdb wireless-tools wpasupplicant zlib1g-dev

apt autoremove
apt autoclean
apt clean

apt update
apt dist-upgrade
apt clean

## clean system (rm)
rmdir /home /srv /media

## install (apt)
apt install git i2c-tools libi2c-dev python-dev

## config DT
### source: https://www.instructables.com/id/Raspberry-PI-Multiple-I2c-Devices/
nano /boot/config.txt
...
# Additional overlays and parameters are documented /boot/overlays/README
dtoverlay=i2c-gpio,bus=3,i2c_gpio_delay_us=2,i2c_gpio_sda=22,i2c_gpio_scl=27

# Enable audio (loads snd_bcm2835)
#dtparam=audio=on

disable_splash=1 # no rainbow on boot
gpu_mem=16
...


# set up software
## git clone
mkdir git
cd git/
git clone https://github.com/lexruee/am2315.git
git clone https://github.com/brenkem/wincon.git

## set up LCD
## source: https://tutorials-raspberrypi.de/raspberry-pi-lcd-display-16x2-hd44780/
