#ifndef PWM_H
#define PWM_H

#include "main.h"
#include "stm32f7xx_hal.h"

class PWM
{
public:
  PWM(TIM_HandleTypeDef *htim, uint32_t channel) :
    _duty_ratio(0), _htim(htim), _channel(channel)
  {
    HAL_TIM_PWM_Start(_htim, _channel);
  }
  
  void write(float duty) {
    _duty_ratio = (int)(duty * 100); 
    __HAL_TIM_SetCompare(_htim, _channel, _duty_ratio);
  }
  
  float read() { return (float)_duty_ratio / 100.0f; }
  
  PWM& operator= (float value) {
    write(value);
    return *this;
  }
  
  operator float() { return read(); }
  
private:
  int _duty_ratio;
  TIM_HandleTypeDef *_htim;
  uint32_t _channel;
};

#endif
