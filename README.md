# ZJU_21
 浙江大学智能小车21组
  
## 版本说明
- vehicleA: 两辆小车路线（vehicle.h)
- vehicle_msg: 增加通信模块（message.h）
  - 正确的开机顺序：先把两车都关机，然后先开A车，再开B车。
- vehicle_sound：增加B车发声模块（sound.h)
  - sound.h是gb格式，尽量用记事本打开。在Arduino乱码是正常情况，不要管警告，直接编译/上传。
  - 两车初始化完成后会播报“准备出发”，才能按启动按钮。
- vehicle_QRcode：增加摄像头模块（camera.h;camera.py），实现扫码和颜色播报
- vehicle_identify：增加识别模块（camera.h）

## 流程：
### 启动
 1. 两车关机
 2. A车开机
 3. B车开机
 4. 依次播报“网络已连接”，“TCP已连接”，“准备出发”
 5. 按启动按钮
 6. 播报“出发”
 7. 两车开始运动
###  扫码
###  拾取物料A
###  拾取物料B
###  装配
