#ifndef DUAL_ANGLE_SENSOR_H
#define DUAL_ANGLE_SENSOR_H

#include "main.h"
#include "stm32f7xx_hal.h"

#ifndef M_PI
#define M_PI           3.14159265358979323846f
#endif

class DualAngleSensor
{
public:

  DualAngleSensor(I2C_HandleTypeDef *hi2c);
  
  void setMotorOffsetAngleRad(float motor_angle_rad) { _motor_angle0_rad = motor_angle_rad; }
  
  void setJointOffsetAngleRad(float joint_angle_rad) { _motor_angle0_rad = joint_angle_rad; }

  void startMeasure();
  
  void stopMeasure();

  float getMotorAngleRad() { return _motor_angle_rad; }
  
  float getJointAngleRad() { return _joint_angle_rad; }
  
  int getError() { return _error; }

  void resetError() { _error = false; }

  float read() { return _motor_angle_rad; }

  void requestReadJointAngle() { _read_joint_flag = true; }

  bool sendMeasureAngleRequest();
  
  bool receiveAngleRequest();
  
  void receiveAngle();
	
	unsigned int getMotorReadCounter() { return _motor_read_counter; }
  
	bool _do_measure;
  
  bool _error;
  
private:

  bool _read_joint_flag;
  bool _do_read_joint;

	unsigned int _motor_read_counter;
  
  float _motor_angle_rad;
  float _motor_angle0_rad;
  float _joint_angle_rad;
  float _joint_angle0_rad;

  float maxPI(float angle);
  
  I2C_HandleTypeDef *_hi2c;

  static const int TX_BUF_SIZE = 16;
  static const int RX_BUF_SIZE = 16;
  unsigned char _tx_buf[TX_BUF_SIZE];
  unsigned char _rx_buf[RX_BUF_SIZE];
  
  int MOTOR_SLAVE_ADDRESS;
  int MOTOR_ANGLE_ADDRESS;
  int JOINT_SLAVE_ADDRESS;
  int JOINT_ANGLE_ADDRESS;
};

#endif
