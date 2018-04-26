#include "ADConv.h"

const float ADConv::MAX_VOLTAGE = 3.3f;

ADConv::ADConv(ADC_HandleTypeDef *hadc1, ADC_HandleTypeDef *hadc2, ADC_HandleTypeDef *hadc3) :
  _hadc1(hadc1), _hadc2(hadc2), _hadc3(hadc3)
{
  __HAL_ADC_ENABLE(_hadc1);
}

bool ADConv::sendStartMeasure()
{
  _hadc1->Instance->CR2 |= ADC_CR2_JSWSTART;
  return true;
}

bool ADConv::recvMeasuredVoltage()
{
  _voltage[0] = (float)HAL_ADCEx_InjectedGetValue(_hadc1, ADC_INJECTED_RANK_1) * MAX_VOLTAGE / 0x7FFF;
  _voltage[1] = (float)HAL_ADCEx_InjectedGetValue(_hadc2, ADC_INJECTED_RANK_1) * MAX_VOLTAGE / 0x7FFF;
  _voltage[2] = (float)HAL_ADCEx_InjectedGetValue(_hadc3, ADC_INJECTED_RANK_1) * MAX_VOLTAGE / 0x7FFF;
  return true;
}
