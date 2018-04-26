#include "LED.h"

GPIO_TypeDef *LED::type[4] = {LED1_GPIO_Port, LED2_GPIO_Port, LED3_GPIO_Port, LED4_GPIO_Port};
const uint16_t LED::pin[4] = {LED1_Pin, LED2_Pin, LED3_Pin, LED4_Pin};
