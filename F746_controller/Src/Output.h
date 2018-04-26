#ifndef OUTPUT_H
#define OUTPUT_H

#include "main.h"
#include "stm32f7xx_hal.h"

class Output
{
public:
  Output(GPIO_TypeDef *type, uint16_t pin) : _type(type), _pin(pin) {}
  
  void setOn() { HAL_GPIO_WritePin(_type, _pin, GPIO_PIN_SET); }
  
  void setOff() { HAL_GPIO_WritePin(_type, _pin, GPIO_PIN_RESET); }
  
  bool isOn() { return HAL_GPIO_ReadPin(_type, _pin) == GPIO_PIN_SET ? true : false; }
  
  void write(int i) { i != 0 ? setOn() : setOff(); };
  
  int read() { return isOn() ? 1 : 0; }
  
  Output& operator= (int value) {
    write(value);
    return *this;
  }
  
  operator int() { return read(); }
  
private:
  GPIO_TypeDef *_type;
  uint16_t _pin;
};

#endif
