#ifndef CARTECOMMANDETRICYCLE_STEERING_H
#define CARTECOMMANDETRICYCLE_STEERING_H

/// Incrément bouton left et right du volant
/// PAS = 0.12° X increment_volant
#define increment_volant 5

/// Gestion volant :
/// angle volant centré 1500 :  [1000 1500 2000] pour[0° 60° 120°]
extern uint32_t angle_volant;

void steering_init(void);
void turn_right(void);
void turn_left(void);

#endif //CARTECOMMANDETRICYCLE_STEERING_H
