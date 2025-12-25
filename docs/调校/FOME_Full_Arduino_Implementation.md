
# FOME ECU 中文文档（技术实现版 + Arduino 实现示例）

## 1. Sensors and Gauges（传感器与仪表实现）

### 1.1 油门踏板 (Accelerator Pedal)
目标：检测油门开度，作为发动机负载输入。  
实现方法：两路信号冗余，ADC 转换，低通滤波平滑，输出负载百分比，异常偏差触发安全模式。  
**Arduino 实现**：模拟输入 `analogRead()`，双通道冗余，用 `map()` 转为百分比，可用低通滤波平滑信号。

### 1.2 曲轴传感器 (Crank Sensor)
目标：提供曲轴角度与转速。  
实现方法：中断处理每齿信号，计算瞬时 RPM，用于点火、喷油触发及空气量计算，异常触发熄火保护。  
**Arduino 实现**：`attachInterrupt()` 捕捉齿信号，硬件计数器计算 RPM。

### 1.3 凸轮轴传感器 (Cam Sensor)
目标：确定气缸相序，实现顺序喷油和精准点火。  
实现方法：边沿检测，状态机生成喷油触发时间，支持单/双凸轮，高速过零检测。异常报警或降功率保护。  
**Arduino 实现**：中断 + 状态机判断气缸顺序，输出喷油触发。

### 1.4 MAP 传感器 (Manifold Absolute Pressure)
目标：测量进气歧管压力，用于 Speed Density 算法计算空气量。  
实现方法：ADC 电压读取，转换 kPa，计算空气流量 VE*MAP/RT*k，输出喷油量参考。  
**Arduino 实现**：模拟输入读取 MAP 电压，线性或查表换算压力。

### 1.5 IAT & CLT（进气温度 & 冷却液温度）
目标：燃油与点火修正，热补偿。  
实现方法：热敏电阻 ADC 转换，查表或公式得 °C，燃油修正：Fuel = BaseFuel * CoolantMultiplier * IATMultiplier。  
**Arduino 实现**：读取 ADC 电压，Steinhart-Hart 公式换算温度，查表或简单公式修正燃油/点火角。

### 1.6 Wideband O2（宽带氧传感器）
目标：实时空燃比测量，闭环控制。  
实现方法：泵电流测 Lambda，AFR=Lambda*AFR_Stoichiometric，闭环 PID 调整喷油。  
**Arduino 实现**：使用 Wideband 模块输出模拟/PWM 信号，采样后 PID 计算喷油修正。

### 1.7 车速传感器 (VSS)
目标：监控车速，用于动态燃油与点火调整。  
实现方法：脉冲信号计算车速，触发怠速控制、减速燃油切断和转速限制。  
**Arduino 实现**：中断捕捉脉冲，累积计数计算速度，滤波防抖。

### 1.8 信号异常检测与保护逻辑
TPS、MAP、CLT、IAT 超范围或异常 -> ECU 安全模式；曲轴/凸轮信号异常 -> 限制转速或关闭喷油；Wideband AFR异常 -> 开环 AFR 表替代。  
**Arduino 实现**：中断 + 条件判断，异常立即切换安全模式。

## 2. Fuel（燃油系统实现）

### 2.1 燃油控制算法
- Alpha-N：TPS + RPM 查表得到 VE 值计算喷油脉宽。  
- Speed Density：MAP + IAT 推算空气量，结合 VE 表计算喷油量。  
- MAF：直接读取空气流量，转换喷油量。  
**Arduino 实现**：表格存储在 EEPROM/Flash，定时循环计算喷油脉宽。

### 2.2 空燃比与喷油量计算
- AFR 目标表查找 -> Lambda 计算  
- 基础喷油量：FuelBase = VE_table*AirDensity*CorrectionFactors  
- 修正：冷却液温度、进气温度、TPS、每缸微调、Flex Fuel  
- 喷油脉宽 = FuelBase / (喷油器流量 * 电压校正) + Deadtime  
**Arduino 实现**：使用定时器输出精确 PWM 或继电器/MOSFET 控制喷油器脉宽。

### 2.3 燃油控制功能
- 冷却液修正、每缸修正、减速燃油切断、Flex Fuel、小脉宽修正、闭环 PID 控制。  
**Arduino 实现**：软件乘法系数实现修正，闭环 PID 调整喷油量。

## 3. Ignition（点火系统实现）

### 3.1 点火硬件
- 智能线圈 vs 普通线圈  
- 点火驱动器  
**Arduino 实现**：数字输出 + MOSFET 驱动线圈，Dwell 控制充能时间。

### 3.2 点火设置
- Dwell、Firing Order、IAT 附加、Ignition Mode、Ignition Advance Table、冷却液点火修正、每缸修正  
**Arduino 实现**：定时器中断输出精确触发信号，查表计算提前角，加修正系数。

### 3.3 点火类型
- 单独线圈、顺序点火、废火花、单线圈/分电器  
**Arduino 实现**：根据曲轴 + 凸轮状态机计算输出时序。

### 3.4 点火控制逻辑
曲轴角度 + 凸轮轴相序 -> 顺序喷油与点火时机  
点火提前表 + 负载/RPM/温度修正 -> 输出触发信号  
异常保护：过压、过热、传感器异常 -> 限制点火或熄火

---

## 4. 实现建议
1. 硬件：Arduino Mega / Due / Teensy 32-bit MCU，足够 ADC 和定时器资源。  
2. 实时性：喷油和点火需微秒级精度，使用硬件定时器 + 中断。  
3. 信号滤波：模拟信号建议 RC 或数字滤波。  
4. 表格存储：VE 表、点火表、AFR 表存 EEPROM/Flash 或 SD 卡。  
5. 调试工具：串口打印或 OLED 显示实时数据，逻辑分析仪验证触发精度。

---

## 📌 注释
本文档为 FOME ECU 核心技术实现中文整理版，涵盖 **传感器信号处理、燃油计算、点火控制、Arduino 实现示例及安全逻辑**，便于调校、二次开发和 MCU 仿真。
