# ArduPilot Project

## 双旋翼轮式机器人项目 ##

- 设置串口  
  设置串口功能:45，波特率默认115200

- 设置电机  
  servo5: k_speedMotorRightWheel: 156  
  servo6: k_speedMotorLeftWheel:  157  
  servo7: k_tiltMotorRightJoint:  158  
  servo8: k_tiltMotorLeftJoint:   159  

- 串口协议
  | Bytes      | Content  | default   |
  | --------   | -------- |--------   |
  | 2          | header   | 0xAA 0xAF |
  | 1          | len      | 15        |
  | 4          | timestamp| 0         |
  | 2          | motor0   | 1000      |
  | 2          | motor1   | 1000      |
  | 2          | servo0   | 1500      |
  | 2          | servo1   | 1500      |
