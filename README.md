# STM_BLDC_Controller_Ixion

This BLDC Controller is developed for an original controller with STM F746.

### Environment

Microsoft Windows  
Arm Keil MDK Version 5  
STMicroelectronics STM32CubeMX  

### Build

```
- STM32CubeMX  
open F746_controller.ioc  
Generate Code  

- Keil  
F746_controller.uvprojx  
remove main.c from project
remove one of system_stm32f7xx.c from project (duplicated files)  
Build Target  
```
