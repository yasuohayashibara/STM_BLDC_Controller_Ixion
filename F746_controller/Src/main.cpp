/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/*
 * MCUTempLimit 0: AS5600(Motor), 1: AS5048B(Motor), 2: AS5600(Joint) + AS5048B(Motor)
 * MotorTempLimit 0: Normal, 1: Reverse
 * 
 */


/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f7xx_hal.h"

/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "LED.h"
#include "switch.h"
#include "RS485.h"
#include "DualAngleSensor.h"
#include "PWM.h"
#include "ADConv.h"
#include "Parser.h"
#include "b3m.h"
#include "STM_BLDCMotor.h"
#include "Flash.h"

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
ADC_HandleTypeDef hadc3;

I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

// version { year, month, day, no }
char version[4] = { 18, 6, 14, 1 };

#define GAIN 10.0
#define GAIN_I 0.0
#define PUNCH 0.0
#define DEAD_BAND_WIDTH 0.0
#define MAX_ANGLE 60.0
#define MIN_ANGLE -60.0
#define BAUDRATE 1500000
#define FLASH_ADDRESS 0x08010000

#ifndef M_PI
#define M_PI           3.14159265358979323846f
#endif

#define max(a, b) ((a) > (b) ? (a) : (b))
#define min(a, b) ((a) < (b) ? (a) : (b))

#define USE_WAKEUP_MODE

extern Property property;

static const unsigned int MAX_COMMAND_LEN = 2048; 
unsigned char command_data[MAX_COMMAND_LEN];
int command_len = 0;
const int LED_TOGGLE_COUNT = 500;
unsigned char send_buf[256];
unsigned char send_buf_len = 0;

const int stocked_number = 1000;
const int period_ms = 5;
short stocked_target_position[stocked_number];
short stocked_encoder_position[stocked_number];
short stocked_motor_position[stocked_number];
short stocked_pwm_duty[stocked_number];

struct RobotStatus {
  float target_angle_rad;
  float target_total_angle_rad;
  float initial_angle_rad;
  float target_torque_Nm;
  bool is_servo_on;
  bool change_target;
  bool isWakeupMode;
  int led_state;
  int led_count;
  float err_i;
  int pulse_per_rotate;
  int control_mode;
} status;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
static void MX_ADC3_Init(void);
static void MX_I2C2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM5_Init(void);

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
                                

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

volatile long time_ms = 0;
STM_BLDCMotor *p_motor = NULL;
int prev_hole_state = -1;
ADConv *p_adc = NULL;
int prev_read_buffer_len = 0;
RS485 *p_rs485_main = NULL;
Parser *p_parser = NULL;
int guard_counter = 0;
int command = 0;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if(htim->Instance == TIM4)      // PWM
  {
    if (p_motor != NULL){
      p_motor->update();
      int hole_state = p_motor->getHoleState();
      if (hole_state != prev_hole_state){
        p_motor->status_changed();
        prev_hole_state = hole_state;
      }
    }
  }
  else if(htim->Instance == TIM5) // 100us timer (RS485)
  {
    if (p_rs485_main != NULL &&  p_parser != NULL){
      int read_buffer_len = p_rs485_main->readBufferLen(); // guard time > 100us
      if (read_buffer_len != 0 && read_buffer_len == prev_read_buffer_len) {
        int command_len = p_rs485_main->read(command_data, MAX_COMMAND_LEN);
        int command_temp = p_parser->setCommand(command_data, command_len);
        if (command_temp != 0) {
          command = command_temp;
        }
        if (p_rs485_main->_direction == RS485::INPUT) p_rs485_main->resetRead();
        if (send_buf_len == 0) {
          send_buf_len = p_parser->getReply(send_buf);
        }
        guard_counter = 2;
      }
      if (send_buf_len > 0 && guard_counter == 0) {
        p_rs485_main->write(send_buf, send_buf_len);
        send_buf_len = 0;
      }
      if (guard_counter > 0) guard_counter --;
      prev_read_buffer_len = read_buffer_len;
    }
  }
  else if(htim->Instance == TIM3) // 1ms timer (main loop)
  {
    time_ms ++;
  }
}

float deg100_2rad(float deg){
  return deg * M_PI / 18000.0f;
}

float deg2rad(float deg){
  return deg * M_PI / 180.0f;
}

float rad2deg100(float rad){
  return rad * 18000.0f / M_PI;
}

void initialize(float angle)
{
  status.initial_angle_rad = status.target_angle_rad = angle;   // read angle
  status.target_total_angle_rad = angle;
  status.is_servo_on = false;
  status.led_state = 0;
  status.led_count = 0;
  status.change_target = false;
  status.isWakeupMode = false;
  status.err_i = 0.0;
  status.control_mode = B3M_OPTIONS_RUN_FREE << 8;
  
  memset((void *)&property, 0, sizeof(property));
  property.ID = 0;
  property.Baudrate = BAUDRATE;
  property.PositionMinLimit = MIN_ANGLE * 100;
  property.PositionMaxLimit = MAX_ANGLE * 100;
  property.PositionCenterOffset = rad2deg100(0);
  property.TorqueLimit = 100;
  property.DeadBandWidth = DEAD_BAND_WIDTH * 100;
  property.Kp0 = GAIN * 100;
  property.Ki0 = GAIN_I * 100;
  property.StaticFriction0 = PUNCH *100;
  property.FwVersion = (version[0] << 24) + (version[1] << 16) + (version[2] << 8) + version[3];
  property.InputVoltageMin = 0; // Dummy (Reference Servo No. 0:Disable, 1~:Servo No.)
  property.InputVoltageMax = rad2deg100(0); // Dummy (Reference Servo Angle)
}

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  initialize(0);
  memcpy((void *)&property, (void *)FLASH_ADDRESS, sizeof(property));

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_ADC3_Init();
  MX_I2C2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  MX_TIM5_Init();

  /* USER CODE BEGIN 2 */
  LED led1(0), led2(1), led3(2), led4(3);
  RS485 rs485(&huart1);

  p_rs485_main = &rs485;
  DualAngleSensor angle_sensor(&hi2c2);
  float dir = (property.MotorTempLimit == 0) ? 1.0 : -1.0;
  HAL_TIM_Base_Start_IT(&htim3);
  HAL_TIM_Base_Start_IT(&htim4);
  HAL_TIM_Base_Start_IT(&htim5);
  ADConv adc(&hadc1, &hadc2, &hadc3);
  p_adc = &adc;
  STM_BLDCMotor motor(&htim4, &angle_sensor);
  p_motor = &motor;
  Parser commnand_parser;
  p_parser = &commnand_parser;
  Flash flash;

  bool is_status_changed = false;
  int time_from_last_update = 0;
  int stocked_count = stocked_number;
  int sub_count = period_ms;
  
  HAL_Delay(100);
  angle_sensor.startMeasure();
  HAL_Delay(100);
  if (property.dummy1 != 100) initialize(0);
  property.dummy1 = 100;  // saved rom flag
  
  motor.setHoleStateInitAngle(deg100_2rad(property.CurrentLimit));
  property.FwVersion = (version[0] << 24) + (version[1] << 16) + (version[2] << 8) + version[3];
  status.pulse_per_rotate = property.MCUTempLimit;
  if (status.pulse_per_rotate <= 0) status.pulse_per_rotate = 2000.0f;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  long prev_time_ms = time_ms;
  motor.servoOn();//  motor = 0.1;
  float prev_integrated_angle = dir * motor.getIntegratedAngleRad();
  int prev_angle_sensor_counter = 0;
  float velocity_rad = 0.0f;
	short prev_goal_angle_position = 0, start_angle_position = 0, step_goal_angle_position = 0;
  for(long count = 0; ; count ++)
  {
    
    status.led_count ++;
    if (status.led_count > LED_TOGGLE_COUNT){
      status.led_state ^= 1;
      led1 = status.led_state;
      status.led_count = 0;
    } 
#ifdef USE_WAKEUP_MODE
    status.isWakeupMode = (count < 5000) ? true : false;
#endif
    
    if (command == B3M_CMD_WRITE || command == B3M_CMD_READ){
      led2 = led2 ^ 1;
    } else if (command == B3M_CMD_SAVE){
      flash.write(FLASH_ADDRESS, (uint8_t *)&property, sizeof(property));
    } else if (command == B3M_CMD_LOAD){
      memcpy((void *)&property, (void *)FLASH_ADDRESS, sizeof(property));
    } else if (command == B3M_CMD_RESET){      
      initialize(0);
      led2 = led3 = led4 = 1;
      HAL_Delay(1000);
      led2 = led3 = led4 = 0;
    } else if (command == B3M_CMD_DATA_STOCK){
      stocked_count = 0;
    } else if (command == B3M_CMD_DATA_PLAY){
      led4 = 1;
      for(int i = 0; i < stocked_number; i ++){
        rs485.printf("%d, %d, %d, %d\r\n", 
          stocked_target_position[i], stocked_encoder_position[i],
          stocked_motor_position[i], stocked_pwm_duty[i]);
        HAL_Delay(10);
      }
      led4 = 0;
    }
    command = 0;
    
    int address, data;
    int com_num = commnand_parser.getNextCommand(&address, &data);
    if (com_num > 0){
      switch(address){
        case B3M_SYSTEM_ID:
          property.ID = data;
          break;
        case B3M_SYSTEM_POSITION_MIN:
          property.PositionMinLimit = data;
          break;
        case B3M_SYSTEM_POSITION_MAX:
          property.PositionMaxLimit = data;
          break;
        case B3M_SYSTEM_POSITION_CENTER:
          property.PositionCenterOffset = data;
          break;
        case B3M_SYSTEM_MCU_TEMP_LIMIT:
          property.MCUTempLimit = data;
          break;
        case B3M_SYSTEM_MOTOR_TEMP_LIMIT:
          property.MotorTempLimit = data;
          break;
        case B3M_SYSTEM_CURRENT_LIMIT:
          property.CurrentLimit = data;
          break;
        case B3M_SYSTEM_DEADBAND_WIDTH:
          property.DeadBandWidth = data;
          break;
        case B3M_SYSTEM_TORQUE_LIMIT:
          property.TorqueLimit = data;
          break;
        case B3M_SERVO_DESIRED_POSITION:
//        data = max(min(data, property.PositionMaxLimit), property.PositionMinLimit);
          status.target_angle_rad = deg100_2rad(data) + deg100_2rad(property.PositionCenterOffset);
          property.DesiredPosition = rad2deg100(status.target_angle_rad);
          is_status_changed = true;
          break;
        case B3M_SERVO_DESIRED_TORQUE:
          status.target_torque_Nm = (double)data / 1000.0f;
          property.DesiredTorque = data;
          break;
        case B3M_CONTROL_KP0:
          property.Kp0 = data;
          break;
        case B3M_CONTROL_KD0:
          property.Kd0 = data;
          break;
        case B3M_CONTROL_KI0:
          property.Ki0 = data;
          status.err_i = 0;
          break;
        case B3M_CONTROL_STATIC_FRICTION0:
          property.StaticFriction0 = data;
          break;
        case B3M_CONTROL_KP1:
          property.Kp1 = data;
          break;
        case B3M_SERVO_SERVO_MODE:
          status.is_servo_on = (data == B3M_OPTIONS_CONTROL_POSITION || data == B3M_OPTIONS_CONTROL_VELOCITY  || data == B3M_OPTIONS_CONTROL_TORQUE || data == B3M_OPTIONS_CONTROL_FFORWARD) ? true : false;
          led3 = (status.is_servo_on) ? 1 : 0;
          if (status.is_servo_on) {
            if (data == B3M_OPTIONS_CONTROL_VELOCITY) {
              status.control_mode = B3M_OPTIONS_CONTROL_VELOCITY;
              status.target_total_angle_rad = dir * motor.getIntegratedAngleRad();
            } else if (data == B3M_OPTIONS_CONTROL_TORQUE) {
              status.control_mode = B3M_OPTIONS_CONTROL_TORQUE;
            } else if (data == B3M_OPTIONS_CONTROL_FFORWARD) {
              status.control_mode = B3M_OPTIONS_CONTROL_FFORWARD;
            }
          }
          if (data == B3M_OPTIONS_RUN_HOLD) status.control_mode = B3M_OPTIONS_RUN_HOLD;
          status.initial_angle_rad = status.target_angle_rad = angle_sensor.getJointAngleRad();
          if (angle_sensor.getError()) break;
          property.DesiredPosition = rad2deg100(status.target_angle_rad);
          break;
        case B3M_SYSTEM_INPUT_VOLTAGE_MIN:
          property.InputVoltageMin = data;	// Reference Servo No.
      }
    }

    short goal_angle_position = property.InputVoltageMax;  // temp
    if (goal_angle_position != prev_goal_angle_position){
      step_goal_angle_position = 0;
      start_angle_position = prev_goal_angle_position;
      prev_goal_angle_position = goal_angle_position;
    }
    short current_reference_angle_position = (float)(goal_angle_position - start_angle_position) * step_goal_angle_position / 10 + start_angle_position;
    step_goal_angle_position ++;
    if (step_goal_angle_position > 10) step_goal_angle_position = 10;
    property.PreviousPosition = property.CurrentPosition;
    short current_position = rad2deg100(angle_sensor.getJointAngleRad()) + current_reference_angle_position;
    angle_sensor.requestReadJointAngle();

    property.CurrentPosition = current_position;
    float period = 0.001f;
    float velocity_rad_raw = (dir * motor.getIntegratedAngleRad() - prev_integrated_angle) / period;
    velocity_rad = (1.0f - 0.1f) * velocity_rad + 0.1f * velocity_rad_raw;
    prev_integrated_angle = dir * motor.getIntegratedAngleRad();
    property.CurrentVelocity = rad2deg100(velocity_rad / 10.0f);
    property.DesiredVelosity = rad2deg100(status.target_angle_rad);
    short joint_angle = rad2deg100(status.target_angle_rad) - property.PositionCenterOffset - current_reference_angle_position;
    status.target_angle_rad = deg100_2rad(max(min(joint_angle, property.PositionMaxLimit), property.PositionMinLimit) + property.PositionCenterOffset + current_reference_angle_position);
    status.target_total_angle_rad += status.target_angle_rad * 10 * period;
    float error = deg100_2rad(property.CurrentPosition) - status.target_angle_rad;
//    float error = status.target_total_angle - dir * motor.getIntegratedAngleRad();
    status.err_i += error * 0.001f;
    status.err_i = max(min(status.err_i, 0.001f), -0.001f); 
    
    float gain = property.Kp0 / 100.0f;
    float gain1 = property.Kp1 / 100.0f;
    float gain_d = property.Kd0 / 100.0f;
    float gain_i = property.Ki0 / 100.0f;
    float punch = property.StaticFriction0 / 100.0f;
    float pwm = gain_i * status.err_i;
//    pwm += gain_d * (status.target_angle_rad * 10 - velocity_rad);
    pwm += gain_d * velocity_rad;
    float margin = deg100_2rad(property.DeadBandWidth);
    if (fabs(error) > margin){
      if (error > 0){
        error -= margin;
        pwm += gain * error + punch;
      } else {
        error += margin;
        pwm += gain * error - punch;
      }
    } else {
        pwm += gain1 * error;
    }
    const float ANGULAR_VEL_TO_PWM = 1.0f / 10.83f * 0.8f; // 80% avoid vibration
    pwm -= ANGULAR_VEL_TO_PWM * velocity_rad;
    
    if (status.control_mode == B3M_OPTIONS_CONTROL_TORQUE) {
      const float TORQUE_COEF_TO_PWM = 1.0f / 34.8f;
      pwm -= TORQUE_COEF_TO_PWM * status.target_torque_Nm;
    }
    if (status.control_mode == B3M_OPTIONS_CONTROL_FFORWARD) {    // Direct PWM Control
      pwm = status.target_angle_rad / M_PI;
    }
    float max_torque = property.TorqueLimit / 100.0f;
    float val = max(min(pwm, max_torque), -max_torque);
    if (status.isWakeupMode) val *= 0.3f;
    
    if (status.is_servo_on) motor = - val * dir;
    else motor = 0;
    property.PwmDuty = abs(motor) * 10000;
    
    if ((is_status_changed)||(time_from_last_update >= 10)){
      motor.status_changed();
      is_status_changed = false;
      time_from_last_update = 0;
    }
    time_from_last_update ++;
    
    if (stocked_count < stocked_number){
      sub_count --;
      if (sub_count <= 0){
        sub_count = period_ms;
        float angle = angle_sensor.read();
        if (angle_sensor.getError()) angle = 0;
        stocked_target_position[stocked_count] = property.DesiredPosition - property.PositionCenterOffset;
        stocked_encoder_position[stocked_count] = rad2deg100(angle) - property.PositionCenterOffset;
        stocked_motor_position[stocked_count] = property.CurrentPosition - property.PositionCenterOffset;
        stocked_pwm_duty[stocked_count] = property.PwmDuty;
        stocked_count ++;
        led4 = 1;
      }
    } else {
      led4 = 0;
    }
    if (angle_sensor.getMotorReadCounter() == prev_angle_sensor_counter){
      led4 = 1;
      HAL_Delay(10);  // min 10
      HAL_I2C_DeInit(&hi2c2);
      HAL_Delay(10);  // min 10
      __HAL_RCC_I2C2_FORCE_RESET();
      HAL_Delay(2);
      __HAL_RCC_I2C2_RELEASE_RESET();
      HAL_Delay(10);  // min 10
      HAL_I2C_Init(&hi2c2);
      HAL_Delay(10);  // min 10
      angle_sensor.startMeasure();
      prev_angle_sensor_counter = angle_sensor.getMotorReadCounter();
      HAL_Delay(10);  // min 10
      led4 = 0;
    } else {
      prev_angle_sensor_counter = angle_sensor.getMotorReadCounter();
    }

    motor.setHoleStateInitAngle(deg100_2rad(property.CurrentLimit));
    if (status.control_mode == B3M_OPTIONS_RUN_HOLD) {
      p_rs485_main = NULL;
			int hole = 0;
			for(count = 0; ; count ++){
        if (count % 200 == 0){
          float ratio = motor;
          float integrated_angle = dir * motor.getIntegratedAngleRad();
          float rot = (integrated_angle - prev_integrated_angle) / (2.0f * M_PI) / 0.2f * 32.0f;
          prev_integrated_angle = integrated_angle;
          char buf[100];
          sprintf(buf, "%f %f %f %f %d %d\r\n", ratio, rot, motor.getHoleStateInitAngle(), angle_sensor.getMotorAngleRad(), prev_hole_state, hole);
          int c = rs485.getc();
          if (c == 'a' && motor <  0.5f) motor = motor + 0.1f;
          if (c == 'z' && motor > -0.5f) motor = motor - 0.1f;
          if (c == 'q') motor.setHoleStateInitAngle(motor.getHoleStateInitAngle() + 0.001f);
          if (c == 'w') motor.setHoleStateInitAngle(motor.getHoleStateInitAngle() - 0.001f);
          if (c == 'h') {
            hole ++;
            if (hole >= 6) hole = 0;
            motor.controlHole(hole,0.2);
          } if (c == 'g') {
            hole --;
            if (hole < 0) hole = 5;
            motor.controlHole(hole,0.2);
          }
          rs485.write(buf, strlen(buf));
        }
        if (count % 500 == 0) led4 = led4 ^ 1;
        while(time_ms == prev_time_ms);
        prev_time_ms = time_ms;
      }
    }
    while(time_ms == prev_time_ms);
    prev_time_ms = time_ms;
    
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 216;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Activate the Over-Drive mode 
    */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_I2C2;
  PeriphClkInitStruct.Usart1ClockSelection = RCC_USART1CLKSOURCE_SYSCLK;
  PeriphClkInitStruct.I2c2ClockSelection = RCC_I2C2CLKSOURCE_HSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* ADC1 init function */
static void MX_ADC1_Init(void)
{

  ADC_MultiModeTypeDef multimode;
  ADC_ChannelConfTypeDef sConfig;
  ADC_InjectionConfTypeDef sConfigInjected;

    /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
    */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the ADC multi-mode 
    */
  multimode.Mode = ADC_TRIPLEMODE_REGSIMULT_INJECSIMULT;
  multimode.DMAAccessMode = ADC_DMAACCESSMODE_DISABLED;
  multimode.TwoSamplingDelay = ADC_TWOSAMPLINGDELAY_5CYCLES;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configures for the selected ADC injected channel its corresponding rank in the sequencer and its sample time 
    */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_10;
  sConfigInjected.InjectedRank = 1;
  sConfigInjected.InjectedNbrOfConversion = 1;
  sConfigInjected.InjectedSamplingTime = ADC_SAMPLETIME_3CYCLES;
  sConfigInjected.ExternalTrigInjecConvEdge = ADC_EXTERNALTRIGINJECCONVEDGE_RISING;
  sConfigInjected.ExternalTrigInjecConv = ADC_EXTERNALTRIGINJECCONV_T2_TRGO;
  sConfigInjected.AutoInjectedConv = DISABLE;
  sConfigInjected.InjectedDiscontinuousConvMode = ENABLE;
  sConfigInjected.InjectedOffset = 0;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* ADC2 init function */
static void MX_ADC2_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;
  ADC_InjectionConfTypeDef sConfigInjected;

    /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
    */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.ScanConvMode = DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_11;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configures for the selected ADC injected channel its corresponding rank in the sequencer and its sample time 
    */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_11;
  sConfigInjected.InjectedRank = 1;
  sConfigInjected.InjectedNbrOfConversion = 1;
  sConfigInjected.InjectedSamplingTime = ADC_SAMPLETIME_3CYCLES;
  sConfigInjected.AutoInjectedConv = DISABLE;
  sConfigInjected.InjectedDiscontinuousConvMode = ENABLE;
  sConfigInjected.InjectedOffset = 0;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc2, &sConfigInjected) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* ADC3 init function */
static void MX_ADC3_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;
  ADC_InjectionConfTypeDef sConfigInjected;

    /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
    */
  hadc3.Instance = ADC3;
  hadc3.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc3.Init.Resolution = ADC_RESOLUTION_12B;
  hadc3.Init.ScanConvMode = DISABLE;
  hadc3.Init.ContinuousConvMode = DISABLE;
  hadc3.Init.DiscontinuousConvMode = DISABLE;
  hadc3.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc3.Init.NbrOfConversion = 1;
  hadc3.Init.DMAContinuousRequests = DISABLE;
  hadc3.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_12;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configures for the selected ADC injected channel its corresponding rank in the sequencer and its sample time 
    */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_12;
  sConfigInjected.InjectedRank = 1;
  sConfigInjected.InjectedNbrOfConversion = 1;
  sConfigInjected.InjectedSamplingTime = ADC_SAMPLETIME_3CYCLES;
  sConfigInjected.AutoInjectedConv = DISABLE;
  sConfigInjected.InjectedDiscontinuousConvMode = ENABLE;
  sConfigInjected.InjectedOffset = 0;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc3, &sConfigInjected) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* I2C2 init function */
static void MX_I2C2_Init(void)
{

  hi2c2.Instance = I2C2;
if (property.MCUTempLimit == 0) {
  hi2c2.Init.Timing = 0x00200105;
} else {
  hi2c2.Init.Timing = 0x00610611;
}
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Analogue filter 
    */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Digital filter 
    */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM2 init function */
static void MX_TIM2_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 108-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 10-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM3 init function */
static void MX_TIM3_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 108-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1000-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM4 init function */
static void MX_TIM4_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 54-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 100-1;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim4);

}

/* TIM5 init function */
static void MX_TIM5_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 108-1;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 100-1;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART1 init function */
static void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 1500000;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_DMADISABLEONERROR_INIT;
  huart1.AdvancedInit.DMADisableonRxError = UART_ADVFEATURE_DMA_DISABLEONRXERROR;
  if (HAL_RS485Ex_Init(&huart1, UART_DE_POLARITY_HIGH, 16, 16) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);
  /* DMA2_Stream7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream7_IRQn);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, LED1_Pin|LED2_Pin|LED3_Pin|LED4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, L1_Pin|L2_Pin|L3_Pin|RX485_REDE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LED1_Pin LED2_Pin LED3_Pin LED4_Pin */
  GPIO_InitStruct.Pin = LED1_Pin|LED2_Pin|LED3_Pin|LED4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : L1_Pin L2_Pin L3_Pin RX485_REDE_Pin */
  GPIO_InitStruct.Pin = L1_Pin|L2_Pin|L3_Pin|RX485_REDE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : SW1_Pin SW2_Pin */
  GPIO_InitStruct.Pin = SW1_Pin|SW2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : HOLE1_Pin HOLE2_Pin HOLE3_Pin */
  GPIO_InitStruct.Pin = HOLE1_Pin|HOLE2_Pin|HOLE3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : RX485_DE_INT_Pin */
  GPIO_InitStruct.Pin = RX485_DE_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(RX485_DE_INT_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler_Debug */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
