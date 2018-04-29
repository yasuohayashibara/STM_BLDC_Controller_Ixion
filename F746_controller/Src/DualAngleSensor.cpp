#include "DualAngleSensor.h"

DualAngleSensor *p_AngleSensor = NULL;

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

DualAngleSensor::DualAngleSensor(I2C_HandleTypeDef *hi2c) :
  _do_measure(false), _error(false), _read_joint_flag(true), _do_read_joint(false),
	_motor_read_counter(0), _motor_angle_rad(0), _motor_angle0_rad(0),
	_joint_angle_rad(0), _joint_angle0_rad(0), _hi2c(hi2c),
	MOTOR_SLAVE_ADDRESS(0x40), MOTOR_ANGLE_ADDRESS(0xFF), // AS5048B
	JOINT_SLAVE_ADDRESS(0x36), JOINT_ANGLE_ADDRESS(0x0E)  // AS5600
{
  p_AngleSensor = this;
}

void DualAngleSensor::startMeasure()
{
  _do_measure = true;
  sendMeasureAngleRequest();
}

void DualAngleSensor::stopMeasure()
{
  _do_measure = false;
}

bool DualAngleSensor::sendMeasureAngleRequest()
{
	if (_read_joint_flag) {
		_do_read_joint = true;
		_read_joint_flag = false;
	} else _do_read_joint = false;
	int slave_address = _do_read_joint ? JOINT_SLAVE_ADDRESS : MOTOR_SLAVE_ADDRESS;
  int angle_address = _do_read_joint ? JOINT_ANGLE_ADDRESS : MOTOR_ANGLE_ADDRESS;
	_tx_buf[0] = angle_address;
  return HAL_I2C_Master_Transmit_IT(_hi2c, slave_address << 1, _tx_buf, 1) == HAL_OK ? true : false;
}

bool DualAngleSensor::receiveAngleRequest()
{
	int slave_address = _do_read_joint ? JOINT_SLAVE_ADDRESS : MOTOR_SLAVE_ADDRESS;
  _motor_read_counter ++;
  return HAL_I2C_Master_Receive_IT(_hi2c, slave_address << 1, _rx_buf, 2) == HAL_OK ? true : false;
}

void DualAngleSensor::receiveAngle()
{
	if (_do_read_joint){
		// AS5600 joint
    _joint_angle_rad = maxPI(((_rx_buf[0] << 8) + _rx_buf[1]) * 0.087912087f * M_PI / 180.0f - _joint_angle0_rad);
	} else {
		// AS5048B motor
    _motor_angle_rad = maxPI((((unsigned short)_rx_buf[0] << 6) + (_rx_buf[1] & 0x3f)) * 0.021973997f * M_PI / 180.0f - _motor_angle0_rad);
    _motor_angle_rad = -_motor_angle_rad; // TODO: check this!
	}
}

float DualAngleSensor::maxPI(float angle)
{
  while(angle >  M_PI) angle -= 2 * M_PI;
  while(angle < -M_PI) angle += 2 * M_PI;
  return angle;
}
