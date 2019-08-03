/*
 * @author 	Alexander Rüedlinger <a.rueedlinger@gmail.com>
 * @date 	28.02.2015
 *
 * A C driver for the sensor AM2315.
 *
 */

#define AM_ADDRESS	0x5c // i2c address of all am2315 sensors

void *am2315_init(const char* i2c_device_filepath);

void am2315_close(void *_am);

float am2315_temperature(void *_am);

float am2315_humidity(void *_am);

int am2315_read_data(void *_am, float *temperature, float *humidity);
