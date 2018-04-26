#ifndef ADCONV_H
#define ADCONV_H

#include "main.h"
#include "stm32f7xx_hal.h"

class ADConv
{
public:
  ADConv(ADC_HandleTypeDef *hadc1, ADC_HandleTypeDef *hadc2, ADC_HandleTypeDef *hadc3);
  
  float getVoltage(int ch) {
    return _voltage[ch];
  }
  
  bool sendStartMeasure();
  
  bool recvMeasuredVoltage();

private:
  ADC_HandleTypeDef *_hadc1, *_hadc2, *_hadc3;
  float _voltage[3];
  static const float MAX_VOLTAGE;
};

#endif
