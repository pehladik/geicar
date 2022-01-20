#include <stdint.h>

/**
 * Measure the distance to the first obstacle using the ultrasound sensor.
 * Method 1, requiring an additional timer (htim3).
 *
 * @returns the measured distance in cm
 */
double ultrasound_get_distance();

/**
 * Measure the distance to the first obstacle using the ultrasound sensor.
 * Method 2, not requiring an additional timer.
 *
 * @returns the measured distance in cm
 */
double ultrasound_get_distance_trig_input();