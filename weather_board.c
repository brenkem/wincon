#include <stdio.h>
#include <unistd.h>
#include "si1132.h"
#include "bme280-i2c.h"
#include "am2315.h"

#define WINCON_CYCLE_TIME	1 // cycle time in seconds


static int pressure; // one location, same air pressure
static float altitude;

// sendor data indoor basement
static int Ti;	// temperature indoor (basement)
static int Hi;	// humidity indoor ( basement)

// sensor date outdoor
float Ta;	// temperature outdoor
float Ha;	// humidity outdoor

float SEALEVELPRESSURE_HPA = 1024.25;

int main(int argc, char **argv)
{
	char *bus_1 = "/dev/i2c-1";
	char *bus_5 = "/dev/i2c-5";

	si1132_begin(bus_5);
	bme280_begin(bus_5);
	void *am_a = am2315_init(bus_5); // Auszensensor

	while (1) {
		printf("\e[1;1H\e[2J");	// clean console
		sleep(WINCON_CYCLE_TIME); // rest a moment

		// read indoor sensor light data
		printf("======== si1132 ========\n");
		printf("UV_index : %.2f\e[K\n", Si1132_readUV()/100.0);
		printf("Visible :  %.0f Lux\e[K\n", Si1132_readVisible());
		printf("IR :       %.0f Lux\e[K\n", Si1132_readIR());


		// read indoor sensor clima data
		bme280_read_pressure_temperature_humidity(&pressure, &Ti, &Hi);

		if ( (Ti <= 0) || (Hi <= 0) || (pressure <= 0)) {
			puts("ERROR: insufficient data");
			continue;
		}

		printf("\n======== bme280 ========\n");
		printf("temperature : %.2lf °C\e[K\n", (double)Ti/100.0);
		printf("humidity :    %.2lf %%\e[K\n", (double)Hi/1024.0);
		printf("pressure :    %.2lf hPa\e[K\n", (double)pressure/100.0);
		printf("altitude :    %f m\e[K\n", bme280_readAltitude(pressure,
							SEALEVELPRESSURE_HPA));

		// read outdoor sensor clima data
		am2315_read_data(am_a, &Ta, &Ha);

		if ( (Ta <= 0) || (Ha <= 0)) {
			puts("ERROR: insufficient data");
			continue;
		}

		printf("\n======== am2315 ========\n");
		printf("temperature : %.2lf °C\e[K\n", Ta);
		printf("humidity :    %.2lf %%\e[K\n", Ha);

	}

	return 0;
}
