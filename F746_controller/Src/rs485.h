#ifndef RS485_H
#define RS485_H

#include <stdarg.h>
#include "main.h"
#include "stm32f7xx_hal.h"

class RS485
{
public:
  enum {
    INPUT = 0,
    OUTPUT
  };

public:
  RS485(UART_HandleTypeDef *huart);

  void setDirection(int status);

  char putc(int c);
  
  int getc();
  
  int read(unsigned char *buf, unsigned int len);

  int readBufferLen();

  void resetRead();
  
  int write(const void* buffer, size_t length);
  
  int printf(const char* format, ...);

  UART_HandleTypeDef *_huart;

  int _rx_index;

  int _direction;
};

#endif
