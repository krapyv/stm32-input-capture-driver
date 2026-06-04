#ifndef INPUT_CAPTURE_DRIVER_H
#define INPUT_CAPTURE_DRIVER_H

#include <stdint.h>
#include "stm32f411.h"

typedef enum
{
    IC_RE = 0,   // 00
    IC_FE,       // 01
    IC_RESERVED, // 10
    IC_BE        // 11
} IC_Polarity_t;

typedef struct
{
    uint32_t overflow_count;
    uint32_t first_timestamp;
    uint32_t second_timestamp;
    uint32_t delta;
} IC_Data_t;

typedef struct
{
    TIM_RegDef_t *Instance;
    GPIO_RegDef_t *Port;
    uint32_t pin;
    GPIO_AF_t af;
    uint32_t channel;
    IC_Polarity_t polarity;
    volatile IC_Data_t data; // tracking data
} IC_HandleTypeDef;

void input_capture_init(IC_HandleTypeDef *ic);
void nvic_interrupts_init(IC_HandleTypeDef *ic);

#endif