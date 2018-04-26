#ifndef LED_H
#define LED_H

#include "main.h"
#include "stm32f7xx_hal.h"

class LED
{
public:
  LED(int no) : _no(no) {}
  
  void setOn() { HAL_GPIO_WritePin(type[_no], pin[_no], GPIO_PIN_SET); }
  
  void setOff() { HAL_GPIO_WritePin(type[_no], pin[_no], GPIO_PIN_RESET); }
  
  bool isOn() { return HAL_GPIO_ReadPin(type[_no], pin[_no]) == GPIO_PIN_SET ? true : false; }
  
  void write(int i) { i != 0 ? setOn() : setOff(); };
  
  int read() { return isOn() ? 1 : 0; }
  
  LED& operator= (int value) {
    write(value);
    return *this;
  }
  
  operator int() { return read(); }
  
private:
  int _no;
  static GPIO_TypeDef *type[4];
  static const uint16_t pin[4];
};

#endif
