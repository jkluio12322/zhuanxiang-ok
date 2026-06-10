# PID 改进方案

## 对比分析: bluecar vs 平衡车测试程序-V1.0

### 当前 bluecar PID (pid.c)
- 增量式 PID，已有: 抗积分饱和、微分先行、前馈、目标归零清零积分、输出限幅
- **缺少**: 输出死区补偿、积分死区、输出变化率限制、非对称限幅

### 参考程序 PID (平衡车测试程序-V1.0)
- 关键特性: **OutOffset** — 输出>0则+偏移，输出<0则-偏移，补偿电机死区
- IOut 独立累积并用 OutMax/OutMin 直接钳位
- 非对称 OutMax/OutMin
- 串级 PID: 速度环→角度环 级联

### 数据反映的问题 (VOFA 日志)
```
目标=-89.1, 实际=-69.1, 输出=-35.2  ← 追踪偏差大
目标=-89.1, 实际=-214.5, 输出=0.8   ← 严重过冲, 输出骤降
目标=-89.1, 实际=-7.3, 输出=-83.5   ← 过度刹车, 反向过冲
```
→ **输出变化太剧烈**（0.8 → -83.5 相邻周期），需要限幅

---

## 计划改动 (仅 pid.h / pid.c)

| # | 改动 | 说明 | 来源 |
|---|------|------|------|
| 1 | **输出死区偏移 `out_offset`** | 电机有死区（TB6612 低占空比不转），输出>0时+offset，<0时-offset | 参考程序 |
| 2 | **积分死区 `deadband`** | 绝对值误差 < deadband 时不累计积分，防止目标附近积分震荡 | 新增 |
| 3 | **输出变化率限制 `output_rate_limit`** | 单次 update 输出变化量上限，防止相邻周期输出跳变 (0=不限) | 新增 |
| 4 | **非对称输出限幅 `out_max/out_min`** | 替代原来的单一 MAX_OUTPUT，正负方向可不同限幅 | 参考程序 |

### pid.h 结构体变更
```c
// 新增字段
float out_max, out_min;       // 替代 MAX_OUTPUT (非对称)
float out_offset;             // 死区补偿, 正值
float deadband;               // 积分死区阈值
float output_rate_limit;      // 输出变化率上限
float last_output;            // 上次输出 (计算变化量用)
```

### pid.c 算法变更
```c
// 1. 积分死区: 误差很小时不累积分
if (fabsf(error) > pid->deadband) {
    pid->error.integral += error;  // 原逻辑保留
}

// 2. 输出变化率限制: |out_new - out_old| 不超过 rate_limit
if (output_rate_limit > 0) {
    // clamp delta
}

// 3. 死区补偿: 输出>0 加 offset, <0 减 offset
if (pid->output > 0) pid->output += pid->out_offset;
else if (pid->output < 0) pid->output -= pid->out_offset;
// 之后再限幅
```

### main.c 调用变更
```c
// 旧: initPID(&SpeedPID, 100.0f, 50.0f);
// 新:
initPID(&SpeedPID, 100.0f, -100.0f, 50.0f);  // out_max, out_min, integral_limit
SpeedPID.out_offset = 5.0f;          // 5% 死区补偿 (需实测调整)
SpeedPID.deadband = 2.0f;            // 2RPM 内不积分
SpeedPID.output_rate_limit = 30.0f;  // 每周期最多变30%
```

### 不动的文件
- pid_tune.c / pid_tune.h — 串口调参命令不变（只调 Kp/Ki/Kd）
- motor.c / motor.h — 不动
- 其他文件不动
