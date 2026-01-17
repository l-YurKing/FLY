# WHEELTEC 四旋翼飞控系统

## 📋 项目概述
基于 **STM32F405** + **FreeRTOS** 的完整四旋翼无人机飞控系统。采用串级PID控制架构，集成IMU、光流、激光雷达等多传感器融合，支持定点悬停、避障、跟随等高级功能。

**平台**: WHEELTEC QR-X1 四旋翼
**主控**: STM32F405 (ARM Cortex-M4, 168MHz)
**工程**: Keil MDK-ARM v5

---

## 🏗️ 系统架构

```
┌─────────────────────────────────────────────┐
│         FreeRTOS 任务调度 (优先级)           │
├──────────┬──────────┬──────────┬───────────┤
│balance   │SensorHandle│ShowTask │BluetoothAPP│
│_task     │_task       │         │_task       │
│200Hz     │事件驱动    │         │事件驱动    │
└──────┬───┴──────┬────┴─────────┴───────────┘
       │          │
    PID控制      数据队列 (Queue + EventGroup)
       │          │
       └──────┬───┘
              ▼
         BSP驱动层
    ┌────┬────┬────┬────┐
    │IMU │电机│光流│雷达│
    └────┴────┴────┴────┘
```

### 🔄 数据流向

```
【输入】
  - 遥控器/APP指令 → g_xQueueFlyControl
  - IMU数据(200Hz) → balance_task
  - 光流数据(50Hz) → getOpticalFlowResult_Callback
  - 高度数据 → g_readonly_distance
  - 雷达数据 → CoordinateHandleTask

【处理】
  balance_task(200Hz)
    ├─ IMU校准 (开机5s)
    ├─ 姿态解算 (Mahony互补滤波)
    ├─ 位置融合 (IMU加速度 + 光流速度)
    ├─ 串级PID计算
    └─ 电机指令输出

【输出】
  DShot电机协议 → 4个电机
```

---

## 📁 核心文件说明

### 应用层 (`WHEELTEC_APP/`)

| 文件 | 功能 | 频率 | 说明 |
|-----|------|------|------|
| **balance_task.c** | 主飞控任务 | 200Hz | 串级PID、姿态控制、位置保持 |
| **pid.c** | PID控制器 | - | 位置式PID + 积分分离 + 微分滤波 |
| **sensorhandle_task.c** | 传感器数据处理 | 事件驱动 | 接收STP23L和STL06N数据 |
| **bluetoothControl_task.c** | 蓝牙APP控制 | 事件驱动 | 解析APP指令，优先级管理 |
| **lidar_task.c** | 激光雷达处理 | 事件驱动 | 避障和跟随模式 |


### 驱动层 (`WHEELTEC_BSP/`)

| 文件 | 设备 | 接口 | 说明 |
|-----|------|------|------|
| **bsp_imu.c** | ICM20948 | I2C | 9轴IMU，采样220Hz，Mahony解算 |
| **bsp_dshot.c** | 无刷电机 | TIM8 PWM+DMA | DShot300协议，4路电机 |
| **bsp_LC307.c** | 光流传感器 | UART3 | 50Hz数据，XY速度 |
| **bsp_stp23L.c** | 高度传感器 | UART5 | 测距传感器 |
| **bsp_stl06n.c** | 激光雷达 | UART2 | 360°扫描，500个数据点 |
| **bsp_led.c** | LED指示灯 | GPIO | 状态提示 |
| **bsp_buzzer.c** | 蜂鸣器 | GPIO | 声音提示 |

---

## 🎮 控制模式

### 事件标志位 (`balance_task.h`)

```c
enum {
    UNUSE_HeightMode_Event      = (1<<0),  // 非定高模式（手动油门）
    IMU_CalibZeroDone_Event     = (1<<1),  // IMU校准完成
    StartFly_Event              = (1<<2),  // 起飞指令
    LowPower_Event              = (1<<3),  // 低电压警告
    TestMotorMode_Event         = (1<<14), // 电机测试模式
    FlyMode_HeadLessMode_Event  = (1<<15), // 无头模式
    lidar_follow_mode           = (1<<16), // 激光雷达跟随
    lidar_avoid_mode            = (1<<17), // 激光雷达避障
};
```

### 飞控指令优先级

```c
enum {
    IDLECmd = 0,  // 空闲
    APPCmd  = 1,  // APP控制（优先）
    PS2Cmd  = 2,  // 遥控器
    AvoidCmd= 3,  // 避障
};
```

---

## 🔧 PID 控制系统

### 串级结构
```
【外环】              【内环】
Roll目标角    →  RollPID    →  RollRatePID    →  电机油门
Pitch目标角   →  PitchPID   →  PitchRatePID   →  电机油门
Yaw目标角     →  YawPID     →  YawRatePID     →  电机油门
Height目标值  →  HeightPID  →  HeightSpeedPID →  电机油门
```

### PID参数 (`pid.c`)

| 环路 | Kp | Ki | Kd | 描述 |
|-----|----|----|----|----|
| RollRate | 85.0 | 0 | 0 | 滚转速率 |
| RollAngle | 4.2 | 0 | 0 | 滚转角度 |
| PitchRate | 85.0 | 0 | 0 | 俯仰速率 |
| PitchAngle | 4.2 | 0 | 0 | 俯仰角度 |
| YawRate | 200.0 | 0 | 0 | 偏航速率 |
| YawAngle | 6.0 | 0 | 0 | 偏航角度 |
| HeightSpeed | 60.0 | 0.02 | 80.0 | 上升速度 |
| Height | 3.5 | 0 | 0.5 | 高度保持 |

### 关键特性
- ✅ **积分分离**: 误差超过阈值时停止积分
- ✅ **微分滤波**: 一阶低通滤波微分项 (α=0.8~1.0)
- ✅ **输出限制**: 防止饱和
- ✅ **积分限制**: 防止积分崩溃

---

## 📡 传感器融合

### IMU姿态解算 (Mahony互补滤波)
```
输入: 陀螺仪 + 加速度计 + 磁力计 (9轴)
处理:
  1. 四元数微分方程更新
  2. 加速度计校正 (捕捉漂移)
  3. 磁力计校正 (防止陀螺仪积分漂移)
输出: Roll, Pitch, Yaw (欧拉角)
频率: 200Hz
延迟: ~5ms
```

### 位置融合 (定点悬停)
```
IMU加速度(200Hz) ──┐
                  ├─ 坐标变换 ──┐
光流速度(50Hz)   ──┤            ├─ 速度融合
                  │            │
陀螺仪角速度────────┘            ├─ 积分 ─→ 位置反馈
                                 │
                        位置PD控制
                                 │
                            pitch/roll目标值
```

---

## ⚡ 启动流程

```
main()
  ├─ HAL_Init() / SystemClock_Config()
  ├─ 硬件初始化
  │  ├─ GPIO/UART/I2C/TIM/ADC/DMA
  │  ├─ OLED屏幕
  │  └─ I2C总线复位 (BUSY异常恢复)
  │
  ├─ IMU初始化 (ICM20948)
  │  └─ 初始化失败自动复位 (NVIC_SystemReset)
  │
  ├─ 光流模块初始化 (LC307)
  │
  ├─ Flash参数读取 (pitch/roll微调值)
  │
  ├─ FreeRTOS初始化
  │  ├─ osKernelInitialize()
  │  └─ MX_FREERTOS_Init()
  │
  └─ osKernelStart() ← 任务调度开始
```

### FreeRTOS 任务创建 (`freertos.c: MX_FREERTOS_Init()`)

```c
1. defaultTask           // USB初始化 + DMA重启
2. ShowTask             // OLED显示
3. SensorHandleTask     // 传感器处理
4. balance_task         // 主飞控 (200Hz)
5. BluetoothAPPControl_task  // 蓝牙控制
6. CoordinateHandleTask // 雷达处理
```

---

## 🚁 电机与推进

### DShot300 协议 (`bsp_dshot.c`)
```
比特率: 300kbps
帧格式: [11bit油门 | 1bit遥测 | 4bit CRC] = 16bit
油门范围: 48 ~ 2047
传输: TIM8 4个PWM通道 + DMA

逻辑1: 420/560 (75% PWM)
逻辑0: 210/560 (37.5% PWM)
```

### 电机分布 (X型)
```
    C(顺)     B(逆)
      ◇       ◇
      |\     /|
      | \   / |
      |  X   |
      | /   \ |
      |/     \|
      ◇       ◇
    D(逆)     A(顺)

g_esc_cmd[0] = 电机A
g_esc_cmd[1] = 电机B
g_esc_cmd[2] = 电机C
g_esc_cmd[3] = 电机D
```

---

## 📊 重要全局变量

| 变量 | 类型 | 说明 |
|-----|------|------|
| `g_readonly_BalanceTaskFreq` | float | 飞控任务实际频率 |
| `g_readonly_distance` | float | 当前高度 (米) |
| `g_robotVOL` | float | 电池电压 (V) |
| `g_lost_pos_dev` | uint8_t | 光流异常标志 |
| `g_userparam_pitchzero` | float | Pitch微调值 (Flash) |
| `g_userparam_rollzero` | float | Roll微调值 (Flash) |
| `appshow` | APPShowType_t | 发送给APP的数据 |
| `LidarFollowRegion` | LidarNearestDenseRegion | 激光雷达跟随目标 |

---

## ⚠️ 关键参数与阈值

```c
// balance_task.c
#define ROBOT_VOL_LIMIT       10.0f   // 起飞最低电压
#define DefalutHeight         1.0f    // 定高模式默认高度
#define HeightMode_MAX        1.5f    // 定高最大高度
#define HeightMode_Min       -0.5f    // 定高最小高度
#define UNHeightMode_MAX      5.0f    // 非定高最大目标
#define UNHeightMode_Min     -0.5f    // 非定高最小目标
#define stop_angle            angle_to_rad(30)  // 30°倾斜自动降落

// lidar_task.c
#define ANGLE_RESOLUTION      0.72f   // 雷达角度分辨率
#define WINDOW_ANGLE          30.0f   // 跟随窗口大小
#define MIN_VALID_DISTANCE    50.0f   // 最小有效距离 (mm)
#define MAX_VALID_DISTANCE    10000.0f // 最大有效距离 (mm)
```

---

## 🐛 常见问题排查

| 症状 | 可能原因 | 排查方法 |
|-----|--------|--------|
| 无法起飞 | IMU校准失败 | 检查LED1是否点亮 |
| 飘移明显 | PID参数不合适 | 调整Kp/Ki/Kd |
| 高度震荡 | 高度环不稳定 | 降低HeightSpeedPID.kp |
| 光流异常 | 光线不足/表面纹理差 | 检查LC307_DEBUG日志 |
| 低电压警告 | 电池快没了 | 充电或更换电池 |
| 雷达数据乱 | 遮挡/强光干扰 | 检查环境 |

---

## 📝 开发注意事项

1. **浮点运算**: 使用 `portTASK_USES_FLOATING_POINT()` 声明浮点任务
2. **临界区**: 关键操作用 `taskENTER_CRITICAL()` 保护
3. **内存**: FreeRTOS堆大小在 `FreeRTOSConfig.h` 中配置
4. **UART**: 中断+DMA接收，使用 `HAL_UARTEx_ReceiveToIdle_DMA`
5. **I2C总线**: 启动时检查BUSY状态，异常时复位（见main.c）

---

## 🔗 相关资源

- STM32F405 数据手册: `Drivers/CMSIS/Device/ST/STM32F4xx/`
- ICM20948 寄存器定义: `icm20948_reg.h`
- FreeRTOS 配置: `Core/Inc/FreeRTOSConfig.h`
- Keil工程: `MDK-ARM/WHEELTEC.uvprojx`

---

**最后更新**: 2025-01-17
**维护者**: l-YurKing
**GitHub**: https://github.com/l-YurKing/FLY
