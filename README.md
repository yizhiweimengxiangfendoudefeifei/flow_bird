# ESP32 多传感器数据采集示例

基于 ESP-IDF 框架，集成 BMP388 气压传感器、PMW3901 光流传感器和 VL53L1X 激光测距传感器，实现温度、气压、海拔高度估计、相对高度、水平位移估计等功能。

---

## 功能说明

### 1. BMP388 气压/温度/海拔（`i2c_bmp3_task`）

- **接口**：I2C（I2C_NUM_0，SDA=GPIO21，SCL=GPIO22）
- **I2C 地址**：`0x77`（SDO 接 VDD）或 `0x76`（SDO 接 GND）
- **采样率**：100 Hz，2x 过采样
- **功能**：
  - 读取温度（°C）和气压（Pa）
  - 利用国际气压公式将气压转换为海拔高度（m）：
    ```
    altitude = 44330 × (1 - (P / 101325)^(1/5.255))
    ```
- **串口输出示例**：
  ```
  Data[0]  T: 25.30 deg C, P: 101200.00 Pa, Alt: 10.50 m
  ```

---

### 2. PMW3901 光流水平位移估计（`pmw3901_position_task`）

- **接口**：SPI（VSPI_HOST）
- **SPI 引脚**：MOSI=GPIO13，MISO=GPIO15，CLK=GPIO26，CS=GPIO27
- **采样率**：~10 Hz
- **功能**：
  - 读取光流传感器的像素位移量（deltaX、deltaY）
  - 通过几何转换将像素位移转换为线速度（rad/s → m/s）
  - 对速度进行 IIR 低通滤波（α=0.7），再积分得到 X/Y 方向位移（m）
  - 包含异常值剔除（|dpixel| ≥ 100 丢弃）
  - 仅在运动标志有效（`motion == 0xB0`）时处理数据
- **串口输出示例**：
  ```
  X:  0.0123 m | Y: -0.0045 m | Vx:  0.012 m/s | Vy: -0.004 m/s | dX:    3 px | dY:   -1 px
  ```
- **注意**：当前使用固定高度 `DEFAULT_HEIGHT_M = 0.1 m` 进行速度转换，需根据实际安装高度调整。

---

### 3. VL53L1X 激光测距高度估计（`vl53l1x_height_task`）

- **接口**：I2C（I2C_NUM_1，SDA=GPIO16，SCL=GPIO17，独立总线）
- **I2C 地址**：`0x29`
- **采样率**：10 Hz（测量预算 50ms，测量周期 100ms）
- **距离模式**：Short 模式（最大 1.3m，抗环境光干扰强）
- **功能**：
  - 连续测距，读取传感器到目标的距离（mm）
  - 异常值剔除（range = 0 或 > 5000 mm 丢弃）
  - IIR 滤波输出高度估计（α=0.90，参考 Crazyflie position_estimator_altitude.c）：
    ```
    estimatedZ = 0.90 × estimatedZ + 0.10 × measurement
    ```
- **串口输出示例**：
  ```
  [ToF] Range: 152 mm | Height(est): 0.152 m
  ```

---

## 硬件接线汇总

| 传感器   | 接口 | ESP32 引脚 | 传感器引脚 |
|:---:|:---:|:---:|:---:|
| BMP390   | I2C（I2C_NUM_0） | GPIO21 (SDA) | SDA |
| BMP390   | I2C（I2C_NUM_0） | GPIO22 (SCL) | SCL |
| PMW3901  | SPI（VSPI） | GPIO13 (MOSI) | MOSI |
| PMW3901  | SPI（VSPI） | GPIO15 (MISO) | MISO |
| PMW3901  | SPI（VSPI） | GPIO26 (CLK)  | CLK |
| PMW3901  | SPI（VSPI） | GPIO27 (CS)   | CS  |
| VL53L1X  | I2C（I2C_NUM_1） | GPIO16 (SDA) | SDA |
| VL53L1X  | I2C（I2C_NUM_1） | GPIO17 (SCL) | SCL |

> **VL53L1X 供电注意**：AVDD 需 2.8V，IOVDD 支持 1.8V 或 2.8V，不可直接接 3.3V/5V。

---

## 文件结构

```
ESP32/
├── main/
│   ├── main.c          # 主程序，三个 FreeRTOS 任务入口
│   ├── bmp3.c/.h       # BMP390 Bosch 官方驱动
│   ├── bmp3_defs.h     # BMP390 寄存器和类型定义
│   ├── common.c/.h     # I2C/SPI 总线初始化及读写适配层
│   ├── pmw3901.c/.h    # PMW3901 光流传感器 SPI 驱动
│   ├── vl53l1x.c/.h    # VL53L1X ToF 传感器 I2C 驱动（ULD 版本）
│   └── CMakeLists.txt
├── CMakeLists.txt
├── sdkconfig
└── README.md
```

---

## 编译与烧录

### 环境准备

```bash
# 激活 ESP-IDF 环境（根据实际安装路径调整）
export IDF_PATH=/home/hope/esp/esp-idf
source $IDF_PATH/export.sh
```

### 编译

```bash
cd ESP32
idf.py build
```

### 烧录

```bash
# 替换 /dev/ttyUSB0 为实际串口设备
idf.py -p /dev/ttyUSB0 flash
```

### 编译 + 烧录 + 监视串口（一键）

```bash
idf.py -p /dev/ttyUSB0 flash monitor
```

### 仅监视串口

```bash
idf.py -p /dev/ttyUSB0 monitor
```

> 退出串口监视：按 `Ctrl+]`
