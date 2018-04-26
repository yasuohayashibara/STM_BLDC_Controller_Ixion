#include "AngleSensor.h"

AngleSensor *p_AngleSensor = NULL;

void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *huart)
{
  if (p_AngleSensor == NULL) return;
  if (huart->Instance == I2C2)
  {
    if (p_AngleSensor->_do_measure)
      if (!p_AngleSensor->receiveAngleRequest()) p_AngleSensor->_error = true;
  }
}

void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *huart)
{
  if (p_AngleSensor == NULL) return;
  if (huart->Instance == I2C2)
  {
    p_AngleSensor->receiveAngle();
    if (!p_AngleSensor->sendMeasureAngleRequest()) p_AngleSensor->_error = true;
  }
}

AngleSensor::AngleSensor(I2C_HandleTypeDef *hi2c, model_t model) :
  _do_measure(false), _error(false), counter(0),_model(model), _angle(0), _angle0(0), _hi2c(hi2c)
{
  p_AngleSensor = this;
  if (model == AS5600){
    SLAVE_ADRESS = 0x36;
    ANGLE_ADRESS = 0x0E;
  } else {
    SLAVE_ADRESS = 0x40;
    ANGLE_ADRESS = 0xFF;
  }
}

void AngleSensor::startMeasure()
{
  _do_measure = true;
  sendMeasureAngleRequest();
}

void AngleSensor::stopMeasure()
{
  _do_measure = false;
}

bool AngleSensor::sendMeasureAngleRequest()
{
  _tx_buf[0] = ANGLE_ADRESS;
  return HAL_I2C_Master_Transmit_IT(_hi2c, SLAVE_ADRESS << 1, _tx_buf, 1) == HAL_OK ? true : false;
}

bool AngleSensor::receiveAngleRequest()
{
  counter ++;
  return HAL_I2C_Master_Receive_IT(_hi2c, SLAVE_ADRESS << 1, _rx_buf, 2) == HAL_OK ? true : false;
}

void AngleSensor::receiveAngle()
{
  if (_model == AS5600) {
    _angle = maxPI(((_rx_buf[0] << 8) + _rx_buf[1]) * 0.087912087f * M_PI / 180.0f - _angle0);
  } else if (_model == AS5048B) {
    _angle = maxPI((((unsigned short)_rx_buf[0] << 6) + (_rx_buf[1] & 0x3f)) * 0.021973997f * M_PI / 180.0f - _angle0);
    _angle = -_angle;
  }
}

float AngleSensor::maxPI(float angle)
{
  while(angle >  M_PI) angle -= 2 * M_PI;
  while(angle < -M_PI) angle += 2 * M_PI;
  return angle;
}


