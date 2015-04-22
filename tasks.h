#include "main.h"
#include "I2C.h"
#include "calculation.h"

// Tasks
void vTaskWriteUART (void *pvParameters);
void vTaskUpdateSpeed (void *pvParameters);
void vTaskGetTemp(void *pvParameters);