#include "RS485.h"

RS485 *p_rs485 = NULL;

static const int RX_BUF_SIZE = 2048;
static const int TX_BUF_SIZE = 2048;
static uint8_t _rx_buf[RX_BUF_SIZE];
static uint8_t _tx_buf[TX_BUF_SIZE];

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  if (p_rs485 == NULL) return;
  if (huart->Instance == USART1)
  {
    HAL_UART_DMAStop(p_rs485->_huart);
    HAL_UART_Receive_DMA(p_rs485->_huart, (uint8_t *)_rx_buf, RX_BUF_SIZE);
    p_rs485->_rx_index = 0;
    p_rs485->setDirection(RS485::INPUT);
  }
}

RS485::RS485(UART_HandleTypeDef *huart) :
  _huart(huart), _rx_index(0), _direction(INPUT)
{
  setDirection(INPUT);
  p_rs485 = this;
  /* Disable the UART Parity Error Interrupt */
  __HAL_UART_DISABLE_IT(_huart, UART_IT_PE);
  /* Disable the UART Error Interrupt: (Frame error, noise error, overrun error) */
  __HAL_UART_DISABLE_IT(_huart, UART_IT_ERR);
  HAL_UART_Receive_DMA(_huart, (uint8_t *)_rx_buf, RX_BUF_SIZE);
}

void RS485::setDirection(int status) {
  GPIO_PinState pin_state = status == OUTPUT ? GPIO_PIN_SET : GPIO_PIN_RESET;
  HAL_GPIO_WritePin(RX485_REDE_GPIO_Port, RX485_REDE_Pin, pin_state);
  _direction = status;
}

char RS485::putc(int c)
{
  _tx_buf[0] = c;
  setDirection(OUTPUT);
  HAL_StatusTypeDef res = HAL_UART_Transmit_DMA(_huart, (uint8_t *)_tx_buf, 1);
  return res == HAL_OK ? c : EOF;
}

int RS485::getc()
{
  if (_direction == OUTPUT) return 0;
  size_t length = (RX_BUF_SIZE + (RX_BUF_SIZE - _huart->hdmarx->Instance->NDTR) - _rx_index) % RX_BUF_SIZE;
  int ret = EOF;
  if (length > 0){
    ret = _rx_buf[_rx_index ++];
    resetRead();
  }
  return ret;
}
  
int RS485::read(unsigned char *buf, unsigned int len)
{
  if (_direction == OUTPUT) return 0;
  size_t length = (RX_BUF_SIZE + (RX_BUF_SIZE - _huart->hdmarx->Instance->NDTR) - _rx_index) % RX_BUF_SIZE;
  length = (length > len) ? len : length;
  if (length > 0){
    if (length == _rx_buf[_rx_index]){
      for(int i = 0; i < length; i ++) {
        buf[i] = _rx_buf[_rx_index ++];
        _rx_index %= RX_BUF_SIZE;
      }
    }
  }
  return length;
}

int RS485::readBufferLen()
{
  size_t length = (RX_BUF_SIZE + (RX_BUF_SIZE - _huart->hdmarx->Instance->NDTR) - _rx_index) % RX_BUF_SIZE;
  return (int)length;
}

void RS485::resetRead()
{
  HAL_UART_DMAStop(p_rs485->_huart);
  HAL_UART_Receive_DMA(p_rs485->_huart, (uint8_t *)_rx_buf, RX_BUF_SIZE);
  _rx_index = 0;
  setDirection(RS485::INPUT);
}
  
int RS485::write(const void* buffer, size_t length)
{
  unsigned char *buf = (unsigned char *)buffer;
  unsigned char *tx_buf = (uint8_t *)_tx_buf;
  for(int i = 0; i < length; i ++){
    *tx_buf ++ = *buf ++;
  }
  setDirection(OUTPUT);
  while(HAL_UART_Transmit_DMA(_huart, (uint8_t *)_tx_buf, length) != HAL_OK);
  return length;
}
  
int RS485::printf(const char* format, ...)
{
  va_list arg;
  va_start(arg, format);
  char buf[256];
  int res = vsprintf(buf, format, arg);
  va_end(arg);
  write(buf, res);
  return res;
}
