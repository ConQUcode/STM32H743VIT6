# 遥控器控制映射

## 宏切换

在 `APPLICATION/remote_logic_profile.h` 中切换：

```c
#define REMOTE_LOGIC_PROFILE_CURRENT 0
#define REMOTE_LOGIC_PROFILE_ALT     1

#ifndef REMOTE_LOGIC_PROFILE
#define REMOTE_LOGIC_PROFILE REMOTE_LOGIC_PROFILE_CURRENT
#endif
```

默认 `REMOTE_LOGIC_PROFILE=0`，也就是当前比赛逻辑；不显式配置时不改变现有行为。

## 底盘

两套 profile 共用同一套底盘逻辑，不受 `REMOTE_LOGIC_PROFILE` 影响。

| 遥控器状态 | 控制内容 |
|---|---|
| 右摇杆 Y | 底盘前后 `vx` |
| 右摇杆 X | 底盘左右平移 `vy` |
| 左摇杆 X | 底盘旋转 `vw` |
| `SC=2` | 默认中位，不触发航向预设 |
| `SC: 2 -> 1` | 底盘以当前航向为基准左转 90 度，只触发一次 |
| `SC: 2 -> 3` | 底盘以当前航向为基准右转 90 度，只触发一次 |

说明：`SC` 必须先回到 `2`，下一次再从 `2` 拨到 `1` 或 `3` 才会再次触发 90 度转向。

## Profile 0：当前比赛逻辑

总模式：

| 遥控器状态 | 控制内容 |
|---|---|
| `SA=1` | 进入 catch 区域 |
| `SA=2` | 进入 arm 区域 |

### Catch

有效前提：`SA=1`。

| 遥控器状态 | 控制内容 |
|---|---|
| `six_pos=1` | 默认空档，不触发 catch 动作 |
| `six_pos=2` | 飞特夹爪闭合抓取，3508 上抬到 `21000` |
| `six_pos=3` | 达妙转到 `level_pos`，3508 下行堵转检测，飞特张开后重新闭合抓取 |
| `six_pos=4` | 释放流程：3508 上升/下降，PC2 拉低后飞特大张开 |
| `six_pos=5/6` | 不触发 catch 动作，按空档处理 |
| `SD: 1 -> 2` | 触发 USB 屏幕下一张 |
| `six_pos=3, SE: 1 -> 2` | 达妙目标在当前微调基础上增加 0.5 度 |
| `six_pos=3, SF: 1 -> 2` | 达妙目标在当前微调基础上减少 0.5 度 |

说明：`six_pos=3` 的达妙微调只允许在原 90 度目标附近 `-2.5` 到 `+2.5` 度范围内调整，也就是限制在 87.5 到 92.5 度区间。离开 `six_pos=3` 后微调清零，下次重新进入 `six_pos=3` 仍先回到原本 90 度目标，再重新微调。

### Arm

有效前提：`SA=2`。

进入 arm 区域时，如果 `six_pos` 不是 `1`，arm 处于锁定状态，不执行六档动作；必须先拨回 `six_pos=1` 解锁。

| 遥控器状态 | 控制内容 |
|---|---|
| `six_pos=1` | 解锁位/空档，保持当前 J2 目标，不应用新预设 |
| `six_pos=2` | 原 ARM J2 任务的第 1 档预设 |
| `six_pos=3` | 原 ARM J2 任务的第 2 档预设 |
| `six_pos=4` | 原 ARM J2 任务的第 3 档预设 |
| `six_pos=5` | 执行 `Arm_ActionPreset2()` 的 J2/大机械臂部分 |
| `six_pos=6` | 执行旧 `SB=2, SC=3` 中保留下来的 J2/大机械臂部分 |
| `SD=1` | arm/J2 气泵关闭 |
| `SD=2` | arm/J2 气泵开启 |

说明：profile 0 的 ARM 模式不使用 `SB/SC` 选择任务，也不主动下发 J1 电机目标；小机械臂气泵遥控控制暂时移除并默认关闭。

## Profile 1：第二套逻辑

总模式：

| 遥控器状态 | 控制内容 |
|---|---|
| `SA=1` | 进入 catch 区域 |
| `SA=2` | 进入 arm 区域 |

### Catch

有效前提：`SA=1`，并带进入保护：

| 进入场景 | 解锁条件 |
|---|---|
| 上电后首次进入 catch | `six_pos=1` |
| 从 arm 返回 catch | `six_pos=2` |

从 arm 返回 catch 且 `six_pos=2` 解锁后，会立即执行 `six_pos=2` 的夹爪动作。

| 遥控器状态 | 控制内容 |
|---|---|
| `six_pos=1` | 整套复位：3508 回 `5000`，达妙回 `init_pos`，PC2 置高，飞特张开 |
| `six_pos=2` | 只执行飞特夹爪闭合、电流检测和保护后保持力矩；不驱动 3508 上抬 |
| `six_pos=3` | 只让达妙转到 `level_pos`，也就是正方向 90 度目标；不启用 SE/SF 微调 |
| `six_pos=4` | 3508 角度环运动到 `20000`；不执行 release 流程 |
| `six_pos=5/6` | 按复位/保持处理，避免残留状态 |
| `SD: 1 -> 2` | 触发 USB 屏幕下一张 |

说明：profile 1 仍走当前 `LiftInit()`，但 3508 初始化完成后的默认目标由 profile 0 的 `18000` 改为 `5000`。

### Arm

有效前提：`SA=2`。

进入 arm 区域时，如果 `six_pos` 不是 `1`，arm 处于锁定状态，不执行六档动作；必须先拨回 `six_pos=1` 解锁。

| 遥控器状态 | 控制内容 |
|---|---|
| `six_pos=1` | 解锁位/空档，保持当前 J1 目标，不应用新预设 |
| `six_pos=2` | 旧 `SB=2, SC=1` 的 J1 目标和 H1/H2 舵机预设 |
| `six_pos=3` | 旧 `SB=2, SC=2` 的 J1 目标和 H1/H2 舵机预设 |
| `six_pos=4` | 旧 `SB=2, SC=3` 的 J1 目标和 H1/H2 舵机预设 |
| `six_pos=5/6` | 不执行动作，保持当前 J1 目标 |
| `SD=1` | J1 气泵关闭，J2 气泵保持关闭 |
| `SD=2` | J1 气泵开启，J2 气泵保持关闭 |

说明：profile 1 的 ARM 模式严格跳过 J2，不调用 J2 预设函数，不下发 `motor_j2` 新目标。当前代码按现有接线命名假定 J1 气泵对应 `Arm_AirModule1Set()`，J2 气泵对应 `Arm_AirModule2Set()`。

## 当前需要 Watch 的变量

```text
remote_boxer.sa
remote_boxer.sb
remote_boxer.sc
remote_boxer.sd
remote_boxer.six_pos
remote_boxer.se
remote_boxer.sf
catch_remote_mode
catch_remote_six_pos
catch_remote_se
catch_remote_sf
catch_dm4310_level_adjust_deg
catch_dm4310_level_target_pos
arm_debug.mode
arm_debug.six_pos
arm_debug.six_pos_unlocked
arm_debug.air_pc8_on
arm_debug.air_pd3_on
chassis_debug.cmd_vx
chassis_debug.cmd_vy
chassis_debug.cmd_vw
```
