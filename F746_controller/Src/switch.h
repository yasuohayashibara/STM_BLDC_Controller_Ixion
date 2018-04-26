#ifndef SWITCH_H
#define SWITCH_H

#include "main.h"
#include "stm32f7xx_hal.h"

class Switch
{
public:
  Switch(int no) : _no(no) {}
    
  bool isOn() { return HAL_GPIO_ReadPin(type[_no], pin[_no]) == GPIO_PIN_RESET ? true : false; }
    
  int read() { return isOn() ? 1 : 0; }
  
  operator int() { return read(); }
    
private:
  int _no;
  static GPIO_TypeDef *type[2];
  static const uint16_t pin[2];
};

GPIO_TypeDef *Switch::type[2] = {SW1_GPIO_Port, SW2_GPIO_Port};
const uint16_t Switch::pin[2] = {SW1_Pin, SW2_Pin};

#endif
