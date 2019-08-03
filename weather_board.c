#include <stdio.h>
#include <unistd.h>
#include "bme280-i2c.h"
#include "si1132.h"

static int pressure;
static int temperature;
static int humidity;
static float altitude;

float SEALEVELPRESSURE_HPA = 1024.25;

int main(int argc, char **argv)
{
	int status = 0;
	int WBVersion = 2;
	char *bus_1 = "/dev/i2c-1";
	char *bus_5 = "/dev/i2c-5";

	bme280_begin(bus_5);
	si1132_begin(bus_5);

	printf("\e[2J"); // clear command line

	while (1) {
		printf("\e[H======== si1132 ========\n");
		printf("UV_index : %.2f\e[K\n", Si1132_readUV()/100.0);
		printf("Visible : %.0f Lux\e[K\n", Si1132_readVisible());
		printf("IR : %.0f Lux\e[K\n", Si1132_readIR());

		bme280_read_pressure_temperature_humidity(
					&pressure, &temperature, &humidity);

		printf("======== bme280 ========\n");
		printf("temperature : %.2lf 'C\e[K\n", (double)temperature/100.0);
		printf("humidity : %.2lf %%\e[K\n", (double)humidity/1024.0);
		printf("pressure : %.2lf hPa\e[K\n", (double)pressure/100.0);
		printf("altitude : %f m\e[K\n", bme280_readAltitude(pressure,
							SEALEVELPRESSURE_HPA));
		usleep(1000000);
	}

	return 0;
}
