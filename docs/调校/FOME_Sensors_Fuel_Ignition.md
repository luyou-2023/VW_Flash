
# FOME ECU 中文文档（传感器 / 燃油 / 点火 部分）

## 1. 传感器与仪表 (Sensors and Gauges)

### 驾驶控制与反馈
- 油门踏板（Accelerator Pedal）：测量油门开度，控制发动机燃油喷射量。
- CAN 显示仪表（CAN Gauge）：通过 CAN 总线输出车辆状态数据。
- 刹车踏板传感器（Brake Pedal Sensor）：用于安全与制动逻辑。
- 离合踏板传感器（Clutch Pedal Sensor）：用于手动挡车辆控制。
- 转速表（Rev Counter）：监控发动机转速。
- 电子节气门位置传感器（ETB TPS）：用于节气门控制。

### 燃油传感器 (Fuel Sensors)
- 弹性燃油传感器（Flex Fuel Sensor）：监控燃油类型与酒精比例。
- 燃油液位（Fuel Level）：油箱燃油剩余量。
- 燃油压力（Fuel Pressure）：保证燃油供给压力。
- 宽带氧传感器（WBO2）：空气燃油比测量，支持闭环燃油控制。

### 通用传感器 (General Sensors)
- 模拟输入 (Analogue Input Settings)：用于接入各种模拟信号。
- 凸轮轴传感器 (Cam Sensor)：用于顺序喷油和精准点火。
- 曲轴传感器 (Crank Sensor)：监控发动机位置。
- 节气门位置传感器 (TPS)：监控油门开度。
- 涡轮转速传感器 (Turbo Speed Sensor)：监控涡轮转速。
- 车速传感器 (VSS)：监控车速。

### 压力传感器 (Pressure Sensors)
- 大气压力 (Barometric)：用于高度补偿。
- 进气歧管绝对压力 (MAP)：用于燃油与点火控制。
- 油压 (Oil Pressure)：发动机润滑监控。

### 温度传感器 (Temperature Sensors)
- 冷却液温度 (Coolant Temperature)
- 进气温度 (Intake Air Temperature)
- 辅助温度输入 (Aux Temperature Sensors)

## 2. 燃油系统 (Fuel)

### 燃油控制算法 (Fuel Algorithms)
- AlphaN：基于节气门开度与转速控制燃油。
- 质量空气流量 (MAF)：空气流量输入控制燃油。
- 速度密度 (Speed Density)：基于 MAP 和 IAT 推算空气流量。

### 主要燃油设置 (Fuel Settings)
- 空燃比设置 (AFR Settings)
- 冷却液修正 (Coolant Multiplier)
- 每缸燃油修正 (Per-Cylinder Fuel Trim)
- 减速燃油切断 (Deceleration Fuel Cut Off)
- 可变燃油模式 (Flex Fuel)
- 喷油模式 (Fuel Injection Mode)
- TPS 修正 (TPS Multiplier)
- 进气温度修正 (IAT Multiplier)
- 喷油器死区时间 (Injector Deadtime Settings)
- 小脉宽修正 (Small Pulse Width Correction)

## 3. 点火系统 (Ignition)

### 点火硬件 (Ignition Hardware)
- 智能线圈 vs 普通线圈 (Smart vs Dumb Coils)
- 点火驱动器 (Ignition Drivers)

### 点火设置 (Ignition Settings)
- 点火保持时间 (Dwell)
- 点火顺序 (Firing Order)
- 进气温度点火附加 (IAT Ignition Adder)
- 点火模式 (Ignition Mode)
- 点火提前表 (Ignition Advance Table)
- 冷却液点火修正 (Ignition Coolant Correction)
- 每缸点火修正 (Per-Cylinder Trim)

### 点火类型 (Ignition Types)
- 单独线圈 (Individual Coils)
- 顺序点火 (Sequential Ignition)
- 废火花 (Wasted Spark)
- 单线圈/分电器 (Single Coil/Distributor)
