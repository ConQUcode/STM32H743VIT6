# STM32H743VIT6 机器人主控工程

本工程基于 STM32H743VIT6，使用 STM32CubeMX/HAL、Keil MDK-ARM 和 FreeRTOS，实现机器人主控层的底盘控制、遥控器接收、USB 视觉通信、机械臂控制、抓取机构控制以及多类电机驱动封装。

这份 README 用来说明当前工程状态、模块划分、任务调度、硬件接口和关键调试注意事项。历史开发记录统一放在文档末尾，避免影响快速接手工程。

## 当前功能状态

| 模块 | 当前状态 | 说明 | 主要文件 |
| --- | --- | --- | --- |
| FDCAN / DJI 电机 | 已封装 | 支持 M3508、M2006、GM6020 的 PID 控制与 CAN 通信 | `MOUDLE/motor/dji_motor.c` |
| 舵轮底盘 | 基本可用 | 四轮舵轮运动学，支持遥控器输入、IMU 航向修正、GM6020 上电保护 | `APPLICATION/chasis.c` |
| 遥控器 | 已测试 | USART6 + DMA ReceiveToIdle + RTOS 任务通知，并处理 STM32H7 D-Cache 一致性 | `MOUDLE/remote/remote.c` |
| USB 视觉通信 | 已测试 | USB CDC 与视觉部分通信 | `MOUDLE/usb/usb.c` |
| 抓取机构 | 基本可用 | 飞特舵机夹具 + DJI 电机升降/旋转，包含机械归零逻辑 | `APPLICATION/catch.c` |
| 机械臂 | 基本调通 | 达妙电机 + 飞特/幻尔舵机，当前以遥控控制和关节跟随为主 | `APPLICATION/arm.c` |
| FreeRTOS | 已接入 | 底盘、电机、USB、遥控器、抓取、机械臂任务分离 | `Core/Src/freertos.c` |
| FreeRTOS V6 修复脚本 | 已加入 | 用于修复 FreeRTOS 在 AC6/V6 编译器环境下的兼容问题 | `fix_freertos_ac6.bat`, `fix_freertos_ac6.ps1` |

## 开发环境

- MCU: STM32H743VIT6
- IDE: Keil MDK-ARM
- 配置入口: `STM32H743VIT6-main/VIT6.ioc`
- Keil 工程: `STM32H743VIT6-main/MDK-ARM/VIT6.uvprojx`
- RTOS: FreeRTOS / CMSIS-RTOS2
- USB: STM32 USB Device CDC
- 依赖: STM32 HAL、CMSIS、FreeRTOS、USB Device Library 已随工程放入仓库，工程具备较好的迁移性

## 工程目录

| 目录 | 作用 |
| --- | --- |
| `Core/` | CubeMX 生成的主程序、外设初始化、FreeRTOS 任务入口 |
| `APPLICATION/` | 应用层控制逻辑，包括底盘、机械臂、抓取机构 |
| `MOUDLE/` | 自定义模块，包括电机、遥控器、USB、IMU、算法等 |
| `BSP/` | 板级支持，如 FDCAN、DWT |
| `USB_DEVICE/` | USB CDC 设备栈 |
| `Drivers/` | STM32 HAL、CMSIS、BSP 依赖 |
| `Middlewares/` | FreeRTOS 和 ST USB Device Library 等中间件 |
| `MDK-ARM/` | Keil 工程文件、启动文件、scatter 文件 |

## FreeRTOS 任务划分

| 任务 | 优先级 | 栈大小 | 功能 |
| --- | --- | --- | --- |
| `Chassis_Task` | `osPriorityAboveNormal` | `512 * 4` | 周期调用 `ChassisTask()`，完成底盘控制 |
| `DJIMotor_Task` | `osPriorityHigh` | `1024 * 4` | 周期发送电机控制指令，目前调用 `FeiteMotorControl()` |
| `usb_Task` | `osPriorityAboveNormal` | `512 * 4` | 周期调用 `USB_ProcessTask()`，处理 USB 通信 |
| `Remot_Task` | `osPriorityNormal` | `128 * 4` | 等待遥控器 DMA 接收事件并解析遥控器数据 |
| `Catch_Task` | `osPriorityBelowNormal` | `768 * 4` | 抓取机构任务入口，目前 `CatchTask()` 调用处保留 |
| `Arm_Task` | `osPriorityBelowNormal` | `512 * 4` | 周期调用 `Arm_Task()`，完成机械臂控制 |

## 硬件与通信接口

| 接口 | 用途 |
| --- | --- |
| FDCAN1 | 抓取机构 DJI 电机 |
| FDCAN2 | 底盘 DJI 电机、达妙电机 |
| USART1 | 飞特舵机总线 |
| USART2 | 幻尔舵机 |
| USART6 | 遥控器接收 |
| USB CDC | 与视觉上位机通信 |
| BMI088 | 底盘 IMU 姿态/航向修正 |
| GPIOE 相关引脚 | 抓取机构传感器、使能与气路控制 |

## 主要模块说明

### 底盘控制

底盘使用四轮舵轮结构，行走电机和转向电机均通过 DJI 电机封装进行控制。`ChassisTask()` 从遥控器读取 `vx`、`vy`、`vw` 指令，随后调用 `SteeringWheelKinematics()` 完成四轮速度和角度分解。

当前底盘包含以下关键逻辑：

- 四轮舵轮运动学解算。
- 轮组最小转角选择，必要时反向轮速，避免转向电机绕远路。
- BMI088 IMU 姿态读取和 EKF 姿态解算。
- 直行航向保持、旋转跟踪、混合修正三种 IMU 修正模式。
- CAN2 ID=3 的 GM6020 单独上电保护，避免反馈未初始化时直接追零导致疯转。
- CAN2 ID=3 的 GM6020 单独输出反向开关 `CHASSIS_GM6020_ID3_OUTPUT_REVERSE`。

### 遥控器接收

遥控器模块使用 `HAL_UARTEx_ReceiveToIdle_DMA`，由 USART6 DMA 后台接收数据帧，空闲中断到来后只唤醒 RTOS 任务，实际解析放在 `RemoteControlTask()` 中完成。

该设计的重点是：

- DMA 后台搬运数据，降低 CPU 占用。
- 中断只做轻量通知，避免影响实时控制。
- 环形缓冲区和帧头查找提高数据解析容错性。
- 针对 STM32H7 D-Cache 做 32 字节对齐和 Cache 失效处理。

### USB 视觉通信

USB 模块基于 USB CDC，与视觉部分进行数据收发。底层接收由 `usbd_cdc_if.c` 回调进入 USB 模块，周期处理由 `USB_ProcessTask()` 完成。

### 抓取机构

抓取机构由飞特舵机夹具和 DJI 电机升降/旋转机构组成。`CatchTask()` 内部包含机械归零、遥控器模式判断、夹取/张开、升降台动作和气路控制等逻辑。

当前控制流程重点：

- 飞特舵机控制三爪开合。
- DJI M3508 控制升降台。
- DJI M2006 控制夹爪旋转。
- 通过电流堵转检测完成升降机构机械归零。
- 遥控器 `switch_left == 2` 时进入抓取模式。

### 机械臂

机械臂模块包含达妙电机、飞特舵机和幻尔舵机控制。当前主要通过遥控器控制 J1，并让幻尔舵机按 J1 目标角度反向跟随。

当前注意点：

- J1 使用达妙 DM4340。
- J3 使用飞特舵机。
- 幻尔舵机通过 USART2 控制。
- 上电后会锁定 J1 零点，避免切入控制时目标突变。
- 机械臂运动学已有基础代码，但实际解算与机构仍存在一定偏差，需要继续结合实车修正。

### 电机封装

工程中封装了多类电机：

- DJI 电机: M3508、M2006、GM6020。
- 达妙电机: DM4340。
- 飞特舵机: HLS/SCS 系列。
- 幻尔舵机。

电机控制层尽量通过统一配置结构体完成初始化，应用层只保存电机实例并设置目标值。

## 调试与注意事项

- 遥控器接收使用 DMA，STM32H7 平台必须注意 D-Cache 一致性，否则可能出现接收到的数据不更新或偶发错帧。
- CAN2 ID=3 的 GM6020 已加入单独上电保护。该电机反馈未初始化时会保持停止；反馈就绪后先抱住当前角度若干控制周期，再进入正常闭环。
- `CHASSIS_GM6020_ID3_OUTPUT_REVERSE` 当前为 `1U`，只作用于 CAN2 ID=3 的 GM6020。如果实车验证方向不对，可以改为 `0U` 再试。
- 达妙电机通过上位机修改 ID 后必须写入保存，否则重新上电可能恢复旧 ID。
- 底盘当前遥控器旋转通道 `vw` 暂时置零。如果需要恢复旋转控制，需要在 `ChassisTask()` 中重新映射右摇杆或拨轮通道。
- 抓取机构中部分流程仍保留实车调试痕迹，修改前应先确认当前机械限位、传感器电平和气路动作方向。
- 工程目录名中存在 `MOUDLE`、`chasis.c` 等历史拼写，当前不建议为了命名统一而大范围改名，以免破坏 Keil 工程引用。

## 已知问题与待完善

- 底盘当前暂未完整使用遥控器旋转通道，`vw` 映射需要结合实际遥控器拨轮或右摇杆补齐。
- 机械臂运动学解算与实际机构仍有偏差，需要继续实车标定。
- 抓取机构气路和时序逻辑仍需根据最终机械结构做整理。
- 部分任务入口已创建但实际调用被注释或保留，需要根据最终功能状态决定是否恢复。
- 历史调试代码和命名问题较多，后续可以在功能稳定后分批整理。

## 历史开发记录

### 初始移植阶段

- 完成 H7 主控库代码从 0 到 1 的移植。
- 对 FDCAN 进行封装。
- 实现 DJI 电机控制逻辑。
- 加入 FreeRTOS 在 V6 编译器报错的自动化修复脚本。

### 2026/4/5

- 新增 USB 通信，与视觉部分对接，待测试。

### 2026/4/6

- 视觉部分测试完毕。

### 2026/4/11

- 添加遥控器部分接收程序，待测试。
- 遥控器接收模块重构为“非阻塞硬件 DMA + RTOS 任务通知”架构。
- 使用 `HAL_UARTEx_ReceiveToIdle_DMA`，由 USART6 DMA 后台搬运定长 18 字节数据帧。
- 空闲中断触发后仅发送任务唤醒信号，实际数据解算交给独立 FreeRTOS 任务完成。
- 应用初始化层调用 `RemoteControlInit()`，FreeRTOS 解算任务中调用 `RemoteControlTask()`。
- 调试时可将 `test_remote` 或 `remote_dev` 加入 Watch 变量窗口，观察摇杆、拨轮、开关实时数据。

### 2026/4/12

- 遥控器测试完成，并进行了优化。
- 底层使用 DMA 搬运接收数据，并处理 D-Cache 一致性问题。
- 中断只发送信号量唤醒任务，应用层 RTOS 任务通过双指针环形缓冲区完成帧头寻址、大端解包与数据校验。

### 2026/4/28

- 修复 FreeRTOS 在不同环境下的报错。
- 将 STM32Cube 相关库复制进工程，工程迁移性提升。

### 2026/4/30

- 给 2026/4/28 的修改打补丁。
- 封装 IMU。
- 重新写入底盘解算。

### 2026/5/1

- 新增长杆抓取部分代码。
- 封装飞特舵机，待测试。

### 2026/5/3

- 长杆抓取与底盘基本调完。
- 底盘暂时未使用 IMU。
- 转向遥控器缺少拨轮。
- 长杆抓取气路暂未加。
- 基本功能已测试完毕。

### 2026/5/4

- 重写达妙电机控制逻辑。
- 注意：上位机修改达妙电机 ID 后一定要写入保存。
- 写入基本机械臂运动学逆解算代码。

### 2026/5/6

- 机械臂基本调完。
- 当前解算与实际情况仍有一定出入，待修改。

### 2026/5/11

- 整车基本调完。
- 后续根据视觉需求进行联调。

### 2026/5/17

- 处理 CAN2 ID=3 的 GM6020 上电疯转问题。
- 在 `APPLICATION/chasis.c` 中给 CAN2 ID=3 的转向 GM6020 增加单独上电保护 `ChassisSteeringId3StartupGuard()`。
- 该电机反馈未初始化时保持停止；反馈就绪后先抱住当前角度若干控制周期，再进入正常转向闭环，避免上电直接大电流追零。
- 新增 `CHASSIS_GM6020_ID3_OUTPUT_REVERSE` 开关，目前为 `1U`。
- 该开关只作用于 CAN2 ID=3 的 GM6020，用于修正该电机最终电流输出方向和软件默认方向相反的问题；若实车验证方向不对，可改为 `0U` 再试。
- 在 `MOUDLE/motor/motor_def.h` 中为 DJI 电机控制设置增加 `output_reverse_flag`，并在 `MOUDLE/motor/dji_motor.c` 中于 PID 计算完成后反向最终输出电流。
- `output_reverse_flag` 默认为 0，不影响其它电机。
- 修正上电保护过度锁车的问题：保护逻辑不再要求四个 6020 全部反馈就绪，也不再提前 `return` 整个底盘解算。
- 现在遥控器仍会驱动其它正常电机；当 ID=3 未 ready 时，仅暂时屏蔽对应左前驱动轮速度，避免未受控转向下强行行走。
- 给 `DJIMotorEnable()`、`DJIMotorStop()`、`DJIMotorSetRef()` 增加空指针保护，提高电机初始化异常时的容错性。
