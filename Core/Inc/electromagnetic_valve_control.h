#ifndef __ELECTROMAGNETIC_VALVE_CONTROL_H__
#define __ELECTROMAGNETIC_VALVE_CONTROL_H__
#include "main.h"

typedef struct {
    uint8_t valve_id;
    uint16_t valve_pin;
} valve_parameter;

void valve_init(valve_parameter* valve, uint8_t id, uint16_t pin);
void open_electromagnetic_valve(uint16_t pin);
void close_electromagnetic_valve(uint16_t pin);
#endif
