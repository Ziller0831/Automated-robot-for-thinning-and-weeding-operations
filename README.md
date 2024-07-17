# 智慧除草機器人
該專案執行人為高科大同步工程設計實驗室團隊。
![CAD](https://github.com/user-attachments/assets/5b1d2555-ca4f-4bdb-a027-cc5b6369b484)

## 關於Delta機械手臂
這台除草機器人的Delta手臂參考了IMWI公司的開源項目https://github.com/deltaxrobot/Delta-X-Firmware.git ，並更改了一些設計，規格如下：
| Parameter   | Spectifications |
| ----------- | ----------- |
| Working space | D = 595.81 mm, H = 209.69 mm  |
| Power supply   | 12VDC       |
| Controller Board | Arduino Mega 2560, RAMPS 1.4 |

更改後的程式放在另一個Repository: [Ziller0831/Delta-X-Firmware](https://github.com/Ziller0831/Delta-X-Firmware.git)

![image](https://github.com/user-attachments/assets/25fc2953-217c-4682-bfc0-f6abb7078b16)

## 關於底盤控制
底盤設計為2 Wheel Drive - 2 Wheel Steer，即2輪驅動轉向設計，驅動馬達為13 inch的BLDC輪鼓馬達，轉向馬達為57型閉迴路步進馬達，控制器使用NodeMCU-32S單晶片。

ESP32程式放在另一個Repository: [Ziller0831/4WD4WS](https://github.com/Ziller0831/ROS2_4WS/tree/3db7d8452c3a13d7f19f9634dbf2a732f82ec4f9/4WD4WS)

## 關於上位機
本機器人的上位機採用LattePanda Delta，規格如下：
* Intel® Celeron® N4100, Quad-Core 1.1-2.4GHz
* Intel® UHD Graphics 600
* Dual-Band 2.5GHz/5GHz Wi-Fi & Bluetooth 5.0
* USB3.0 x3，USB Type-C x1
* Gigabit Ethernet
* 2 x M.2 PCIe (Support B&M Key and A&E Key)
* Support Windows 10 & Linux OS
* Integrated Arduino Coprocessor ATMEL 32U4
  
作業系統使用Linux架構的Ubuntu22.04，並配合ROS2 humble進行資料溝通。

### ROS2
目前專案使用的package如下：
``` bash
src
├── customize_interface
├── delta_robot
├── gamepad_controller
├── ldlidar_stl_ros2
├── ros2.repos
├── uros
└── weeding_robot
```
#### 關於Delta手臂的上位控制程式
手臂的運作架構如下：

![繪圖1](https://github.com/user-attachments/assets/11db749c-05c4-42e9-8849-4ee6a3657f4f)
上下位機溝通使用Pyserial進行傳輸，由上位機整合好的GCode命令通過Serial Port的形式發送至下位機。

## 操作
### 驅動機器人底盤
於command window中輸入`ros2 launch weeding_robot joycontrol_launch.xml`，會跑出下面畫面：
![image](https://github.com/user-attachments/assets/7c124030-006b-4f7e-a28f-f99daf97d9e0)
這表示ROS2正在等待Joy與ESP32的MicroROS的接通，以下一項項處理。
1. Joy：
  這邊手把使用Xbox Series X控制器並使用藍牙連接，如果要用其他產品，則需要到`src/weeding_robot/launch/joycontrol_launch.xml‵中進行更改，連上方式就於Setting的Bluetooth中進行配對即可。
2. ESP32：
	ESP32則是需要按壓Reset按鈕即可。
> 若使用到一半發現Joy無法控制底盤時，則重新執行launch即可。
### 驅動Delta手臂
於command window中輸入`ros2 launch delta_robot delta_robot_launch.xml`，手臂就會從`"/home/ced/Image_recognition_ws/output/keypoints_3d.csv`中提取目標點座標並編排成一套流程。
> 該程式還未設計自動停止，因此會一直重複執行流程，若想執行一遍，則需要在手臂歸零時，終止launch檔的運作
