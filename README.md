## 说明
这是一个ros的node，旨在为turtlebot添加一些额外的设备，比如IMU/sonar/tof-ranger等等。
主要以串口输入，或许是挂载arduino上的，不依赖于rosserial。

### extended_imu.py
在做SLAM的过程中，姿态估计需要一个精度以及帧率更高的IMU，因此购买了一个超核电子的IMU模块(Hi219M).
[淘宝链接](https://item.taobao.com/item.htm?spm=a230r.1.14.20.ebb2eb2biL3Ai&id=26915396185&ns=1&abbucket=18#detail)
已经做了加速度到linear acceleration的转换。
