#ifndef ANGLE_SENSOR_H
#define ANGLE_SENSOR_H

#include "main.h"
#include "stm32f7xx_hal.h"

#ifndef M_PI
#define M_PI           3.14159265358979323846f
#endif

class AngleSensor
{
public:
  enum model_t {
    AS5600,
    AS5048B
  };

  AngleSensor(I2C_HandleTypeDef *hi2c, model_t model);
  
  void setOffserAngleRad(float value) { _angle0 = value; }
  
  void startMeasure();
  
  void stopMeasure();

  float getAngleRad() { return _angle; }
  
  float getAngleDeg() { return _angle * 180.0f / M_PI; }

  int getError() { return _error; }

  void resetError() { _error = false; }

  float read() { return _angle; }

  bool sendMeasureAngleRequest();
  
  bool receiveAngleRequest();
  
  void receiveAngle();
  
  model_t getModel() { return _model; }
  
  bool _do_measure;
  
  bool _error;
  
  unsigned int counter;
  
private:
  model_t _model;
  float _angle;
  float _angle0;

  float maxPI(float angle);
  
  I2C_HandleTypeDef *_hi2c;

  static const int TX_BUF_SIZE = 16;
  static const int RX_BUF_SIZE = 16;
  unsigned char _tx_buf[TX_BUF_SIZE];
  unsigned char _rx_buf[RX_BUF_SIZE];
  
  int SLAVE_ADRESS;
  int ANGLE_ADRESS;

};

#endif
