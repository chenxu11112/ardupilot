# ArduPilot 开发 #
本项目在Ardupilot基础上进行二次开发，目前基于`Copter-4.4`进行开发

## 编译下载 ##

* **编译 TSDPilot-S4-FCU**
  ```sh
  ./waf configure --board=TSDPilot-S4-FCU
  ./waf copter 
  ```
* **上传 TSDPilot-S4-FCU**
  ```sh
  ./waf targe=bin/arducopter --upload 
  或者
  ./waf copter --upload #编译同时上传
  ```

* **编译 TSDPilot-S4-PDU**
  ```sh
  ./waf configure --board=TSDPilot-S4-PDU
  ./waf AP_Periph 
  ```
* **上传 TSDPilot-S4-PDU**  
  通过`TSDPilot-S4-FCU`飞控板的CAN接口进行上传，通过`MissionPlanner`地面站的`DroneCAN`界面进行固件上传

## 软件说明 ##

* **遥控器通道控制ESC开关(TSDPilot-S4-FCU)**  
  ```cmake
  define UAVCAN_ESC_CONTROL             #使能ESC开关控制
  define UAVCAN_ESC_CONTROL_RC_CH CH_7  #设置用于开关ESC的遥控器通道
  define HAL_GPIO_ESC_OPEN_NUM 123456   
  define HAL_GPIO_ESC_CLOSE_NUM 654321  #设置开关ESC的变量值
  ```

* **遥控器通道控制ESC开关(TSDPilot-S4-PDU)**  
  ```cmake
  define HAL_GPIO_ESC_ENABLED           #使能ESC开关控制
  define HAL_GPIO_ESC_PIN 13            
  PB5 ESC_GPIO  OUTPUT GPIO(13)         #设置用于开关ESC的GPIO
  define HAL_GPIO_ESC_OPEN 1            #设置用于开ESC的GPIO的电平
  define HAL_GPIO_ESC_CLOSE 0           #设置用于开ESC的GPIO的电平
  define HAL_GPIO_ESC_CLOSE_NUM 654321  #设置开关ESC的变量值
  define HAL_GPIO_ESC_OPEN_NUM 123456   
  define HAL_GPIO_ESC_CLOSE_NUM 654321  #设置开关ESC的变量值，和TSDPilot-S4-FCU对应
  ```

* **航行灯设置(TSDPilot-S4-PDU)**  
  ```cmake
  define HAL_RGB_FlIGHT_STAT  #使能航行灯
  ```
