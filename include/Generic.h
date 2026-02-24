#ifndef GENERIC_H
#define GENERIC_H

#include "Globals.h"
#include "esp_timer.h"

void status_LED_task(void *arg);
void telemetry_print_task(void *arg);
uint32_t telemetry_timestamp_ms(void);

#endif /* GENERIC_H */