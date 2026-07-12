# 遥控器控制映射

## 总模式

| 遥控器状态 | 控制内容 |
|---|---|
| `SA=1` | 进入 catch 区域，只执行 catch 任务相关控制 |
| `SA=2` | 进入 arm 区域，只执行 arm 任务相关控制 |

## 底盘

| 遥控器状态 | 控制内容 |
|---|---|
| 右摇杆 Y | 底盘前后 `vx` |
| 右摇杆 X | 底盘左右平移 `vy` |
| 左摇杆 X | 底盘旋转 `vw` |
| `SC=2` | 默认中位，不触发航向预设 |
| `SC: 2 -> 1` | 底盘以当前航向为基准左转 90 度，只触发一次 |
| `SC: 2 -> 3` | 底盘以当前航向为基准右转 90 度，只触发一次 |

说明：`SC` 必须先回到 `2`，下一次再从 `2` 拨到 `1` 或 `3` 才会再次触发 90 度转向。

## Catch 区域

有效前提：`SA=1`。

| 遥控器状态 | 控制内容 |
|---|---|
| `six_pos=1` | 默认空档，不触发 catch 动作 |
| `six_pos=2` | 飞特夹爪闭合抓取，3508 上抬 |
| `six_pos=3` | 达妙转到 `level_pos`，3508 下行堵转检测，飞特张开后重新闭合抓取 |
| `six_pos=4` | 释放流程：3508 上升/下降，PC2 拉低后飞特大张开 |
| `six_pos=5/6` | 不触发 catch 动作，按空档处理 |
| `SD: 1 -> 2` | 触发 USB 屏幕下一张 |
| `six_pos=3, SE: 1 -> 2` | 达妙目标在当前微调基础上增加 0.5 度 |
| `six_pos=3, SF: 1 -> 2` | 达妙目标在当前微调基础上减少 0.5 度 |

说明：`six_pos=3` 的达妙微调只允许在原 90 度目标附近 `-2.5` 到 `+2.5` 度范围内调整，也就是限制在 87.5 到 92.5 度区间。离开 `six_pos=3` 后微调清零，下次重新进入 `six_pos=3` 仍先回到原本 90 度目标，再重新微调。

## Arm 区域

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
| `SD=1` | arm 气泵关闭 |
| `SD=2` | arm 气泵开启 |

说明：ARM 模式下不再使用 `SB/SC` 选择任务，也不主动下发 J1 电机目标；小机械臂气泵遥控控制暂时移除并默认关闭。

## 当前需要 Watch 的变量

```text
remote_boxer.sa
remote_boxer.sb
remote_boxer.sc
remote_boxer.sd
remote_boxer.six_pos
catch_remote_mode
catch_remote_six_pos
catch_remote_se
catch_remote_sf
catch_dm4310_level_adjust_deg
catch_dm4310_level_target_pos
arm_debug.mode
arm_debug.six_pos
arm_debug.six_pos_unlocked
chassis_debug.cmd_vx
chassis_debug.cmd_vy
chassis_debug.cmd_vw
```
