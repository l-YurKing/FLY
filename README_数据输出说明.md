# 飞行数据输出功能使用说明

## 📊 功能概述

已为您的四旋翼飞控系统添加了**实时飞行数据输出**功能，可以将飞行过程中的关键参数通过USART1串口输出为CSV格式，方便保存和分析。

---

## 🛠️ 实现细节

### 修改的文件

1. **`WHEELTEC_APP/Inc/balance_task.h`**
   - 新增宏定义 `ENABLE_FLIGHT_DATA_OUTPUT` 用于开关数据输出功能

2. **`WHEELTEC_APP/balance_task.c`**
   - 新增 `OutputFlightData()` 函数实现CSV格式数据输出
   - 在主循环中每10个周期(20Hz)调用一次输出函数

---

## 📡 输出数据列表

每行CSV数据包含**21个字段**：

| 序号 | 字段名 | 单位 | 说明 |
|-----|--------|------|------|
| 1-3 | Roll, Pitch, Yaw | 度(°) | 姿态角 |
| 4-6 | GyroX, GyroY, GyroZ | rad/s | 三轴角速度 |
| 7-9 | AccelX, AccelY, AccelZ | m/s² | 三轴加速度 |
| 10 | Height | 米(m) | 当前高度 |
| 11-12 | PosX, PosY | 米(m) | 当前位置 |
| 13-14 | TargetX, TargetY | 米(m) | 目标位置 |
| 15-16 | SpeedX, SpeedY | m/s | 当前速度 |
| 17 | Voltage | 伏特(V) | 电池电压 |
| 18-21 | MotorA/B/C/D | - | 四个电机的油门值 |

**输出频率**: 20Hz (balance_task运行在200Hz，每10次输出一次)

**串口配置**:
- 端口: USART1 (调试串口)
- 波特率: 115200
- 格式: 8N1

---

## 🚀 使用方法

### 1️⃣ 开启/关闭数据输出

编辑 `WHEELTEC_APP/Inc/balance_task.h`:

```c
// 设置为1开启数据输出，设置为0关闭
#define ENABLE_FLIGHT_DATA_OUTPUT 1
```

### 2️⃣ 编译并烧录到STM32

使用Keil MDK编译工程并烧录到飞控板。

### 3️⃣ 使用Python脚本接收数据

已为您创建了 `flight_data_logger.py` 脚本。

**使用步骤**:

1. **安装依赖**:
```bash
pip install pyserial
```

2. **修改串口配置**:
打开 `flight_data_logger.py`，修改第12行：
```python
SERIAL_PORT = 'COM3'  # Windows系统改为实际端口号，如COM5
                      # Linux系统改为/dev/ttyUSB0等
```

3. **运行脚本**:
```bash
python flight_data_logger.py
```

4. **启动飞行器**，脚本会自动接收并保存数据到 `flight_logs/` 目录

5. **停止录制**: 按 `Ctrl+C`

---

## 📁 数据保存格式

### 文件命名
```
flight_logs/flight_data_20260117_153025.csv
                      ^^^^^^^^^^^^^^^^
                      日期时间戳
```

### CSV文件示例

```csv
Roll(deg),Pitch(deg),Yaw(deg),GyroX(rad/s),GyroY(rad/s),...
0.523,-1.245,45.678,0.05,-0.03,...
0.612,-1.198,45.712,0.06,-0.02,...
...
```

---

## 🔧 高级配置

### 调整输出频率

在 `balance_task.c` 第745行修改分频系数：

```c
if( ++dataOutputCounter >= 10 )  // 10 = 200Hz / 20Hz
```

例如：
- 改为 `>= 5` → 输出频率40Hz
- 改为 `>= 20` → 输出频率10Hz

### 自定义输出字段

修改 `OutputFlightData()` 函数 (balance_task.c:1140-1195)，可以：
- 删除不需要的字段
- 添加其他全局变量（如PID输出值）
- 修改输出精度

---

## 📊 数据分析示例

使用Python/Matlab/Excel打开CSV文件进行分析。

**Python示例**:
```python
import pandas as pd
import matplotlib.pyplot as plt

# 读取数据
df = pd.read_csv('flight_logs/flight_data_20260117_153025.csv')

# 绘制姿态角曲线
plt.figure(figsize=(12, 4))
plt.plot(df['Roll(deg)'], label='Roll')
plt.plot(df['Pitch(deg)'], label='Pitch')
plt.plot(df['Yaw(deg)'], label='Yaw')
plt.legend()
plt.xlabel('Sample')
plt.ylabel('Angle (deg)')
plt.title('Attitude Angles')
plt.grid(True)
plt.show()
```

---

## ⚠️ 注意事项

1. **串口冲突**: USART1同时用于printf调试和数据输出，数据量大时可能影响其他调试信息
2. **性能影响**: 数据输出使用了互斥锁，若锁被占用会跳过本次输出，不影响飞控主循环
3. **存储空间**: 长时间飞行会生成大量数据，注意定期清理 `flight_logs/` 目录
4. **APP功能**: 不影响UART4的APP显示功能，两者可以同时使用

---

## 🐛 故障排查

| 问题 | 可能原因 | 解决方法 |
|-----|---------|---------|
| Python脚本无法打开串口 | 串口号错误或被占用 | 检查设备管理器确认端口号 |
| 接收到乱码 | 波特率不匹配 | 确认STM32和脚本都是115200 |
| 没有数据输出 | 宏开关未开启 | 检查 `ENABLE_FLIGHT_DATA_OUTPUT` |
| 数据不完整 | 缓冲区溢出 | 降低输出频率或增加串口波特率 |

---

## 📞 技术支持

如有问题，请检查：
1. 编译是否通过
2. 串口配置是否正确
3. Python依赖是否安装

---

**创建时间**: 2026-01-17
**作者**: Claude Code
