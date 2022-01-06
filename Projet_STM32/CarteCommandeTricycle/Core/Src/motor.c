#include "motor.h"
#include "main.h"

/// Gestion vitesse :
/// de 0 a 0xFFFFFF sur 12 bits, DAC OUT PA4 : de 0 a 3.3V
#define MAX_POWER 0xFFFFFF

void motor_init(void) {
	// start  DAC vitesse
	HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 0x000); // PA4 écriture 12 bits sur sortie
	HAL_DAC_Start(&hdac1, DAC_CHANNEL_1); // démarrage une fois
	// commande vitesse nulle DAC OUT
	HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 0x000); // PA4 vitesse nulle
}

void motor_set_power(float power) {
	if (power < 0) {
		power = 0;
	}
	if (power > 1) {
		power = 1;
	}
	uint32_t DAC_power = (uint32_t) (power * MAX_POWER);
	HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, DAC_power); // PA4 vitesse nulle 0x000000 à 0xFFFFFF / 0 à 3.3V
}
