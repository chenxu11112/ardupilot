# ArduPilot Project

## 双旋翼轮式机器人项目 ##

- 设置串口  
  设置串口功能:45，波特率默认115200

- 设置电机  
  servo5: k_speedMotorLeftWheel:  159  
  servo6: k_speedMotorRightWheel: 158  
  servo7: k_tiltMotorLeftJoint:   157  
  servo8: k_tiltMotorRightJoint:  156  

- 串口协议
  | Bytes      | Content  | default   |
  | --------   | -------- |--------   |
  | 2          | header   | 0xAA 0xAF |
  | 1          | len      | 15        |
  | 4          | timestamp| 0         |
  | 2          | motor1   | 1500      |
  | 2          | motor2   | 1500      |
  | 2          | motor3   | 1500      |
  | 2          | motor4   | 1500      |
