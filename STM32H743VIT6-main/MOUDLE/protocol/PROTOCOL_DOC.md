> 生成时间：2026-03-29T21:35:27+08:00
# MCU ↔ ROS 串口通信协议文档

> **Auto-generated** — 由 `scripts/codegen.py` 根据 `config/protocol.yaml` 生成，请勿手动修改。

---

## 全局参数

| 参数 | 值 |
| :--- | :--- |
| 波特率 | `115200` |
| 帧头字节 1 | `0x5a` |
| 帧头字节 2 | `0xa5` |
| 校验算法 | `CRC8` |
| 强制握手 | `是` |
| 协议哈希（握手用）| `0x0A622076` |

---

## 帧格式

每帧结构如下（小端序）：

| 字节位置 | 字段 | 说明 |
| :------: | :--- | :--- |
| 0 | Header1 | 固定 `0x5a` |
| 1 | Header2 | 固定 `0xa5` |
| 2 | ID | 消息 ID，见下表 |
| 3 | Len | 数据段字节数 |
| 4 … 4+Len-1 | Data | 各字段按结构体内存布局排列 |
| 4+Len | Checksum | CRC8，覆盖 ID + Len + Data，多项式 `0x31` |

---

## 电控 → ROS（电控主动发送）

### `Handshake` — ID `0xff`

- **ROS 话题**：`/task/handshake`
- **ROS 消息类型**：`std_msgs/msg/UInt32`
- **数据段字节数（Len）**：`4`
- **注意事项**：上电后 ROS 主动发起握手，电控收到后用相同的 protocol_hash 原样回复。握手通过后方可发送其他数据帧，且整个连接周期内只需执行一次。protocol_hash 值见【全局参数】。
- **默认生成行为**：`on_receive_Handshake()` 在收到匹配 `PROTOCOL_HASH` 的握手包后会自动调用 `send_Handshake(pkt)` 回包。

| 字节偏移 | 字段名 | C 类型 | 字节数 |
| :------: | :----- | :----- | :----: |
| 0 | `protocol_hash` | `uint32_t` | 4 |
| **4** | *(CRC8)* | `uint8_t` | 1 |

### `GenericStatusRx` — ID `0x20`

- **ROS 话题**：`/task/generic_status_rx`
- **ROS 消息类型**：`std_msgs/msg/Float32MultiArray`
- **数据段字节数（Len）**：`3`
- **注意事项**：统一状态 RX 话题，映射逻辑: [task_id, rx_id, rx_status]。约定: 结束=0, 开始=1。

| 字节偏移 | 字段名 | C 类型 | 字节数 |
| :------: | :----- | :----- | :----: |
| 0 | `task_id` | `uint8_t` | 1 |
| 1 | `rx_id` | `uint8_t` | 1 |
| 2 | `rx_status` | `uint8_t` | 1 |
| **3** | *(CRC8)* | `uint8_t` | 1 |

---

## ROS → 电控（电控被动接收）

### `Heartbeat` — ID `0xfe`

- **ROS 话题**：`/task/heartbeat`
- **ROS 消息类型**：`std_msgs/msg/UInt32`
- **数据段字节数（Len）**：`4`
- **注意事项**：握手完成后，ROS 侧以固定周期下发心跳。电控收到后必须尽快原样回同一个 count 作为确认，但不再独立主动发送心跳。只有与 ROS 最近一次发送值一致的回包才算有效确认。
- **默认生成行为**：`on_receive_Heartbeat()` 会自动调用 `send_Heartbeat(pkt)`，按原样回同一个 `count` 作为 ACK。

| 字节偏移 | 字段名 | C 类型 | 字节数 |
| :------: | :----- | :----- | :----: |
| 0 | `count` | `uint32_t` | 4 |
| **4** | *(CRC8)* | `uint8_t` | 1 |

### `Handshake` — ID `0xff`

- **ROS 话题**：`/task/handshake`
- **ROS 消息类型**：`std_msgs/msg/UInt32`
- **数据段字节数（Len）**：`4`
- **注意事项**：上电后 ROS 主动发起握手，电控收到后用相同的 protocol_hash 原样回复。握手通过后方可发送其他数据帧，且整个连接周期内只需执行一次。protocol_hash 值见【全局参数】。
- **默认生成行为**：`on_receive_Handshake()` 在收到匹配 `PROTOCOL_HASH` 的握手包后会自动调用 `send_Handshake(pkt)` 回包。

| 字节偏移 | 字段名 | C 类型 | 字节数 |
| :------: | :----- | :----- | :----: |
| 0 | `protocol_hash` | `uint32_t` | 4 |
| **4** | *(CRC8)* | `uint8_t` | 1 |

### `CmdVel` — ID `0x00`

- **ROS 话题**：`/cmd_vel`
- **ROS 消息类型**：`geometry_msgs/msg/Twist`
- **数据段字节数（Len）**：`12`

| 字节偏移 | 字段名 | C 类型 | 字节数 |
| :------: | :----- | :----- | :----: |
| 0 | `linear_x` | `float` | 4 |
| 4 | `linear_y` | `float` | 4 |
| 8 | `angular_z` | `float` | 4 |
| **12** | *(CRC8)* | `uint8_t` | 1 |

### `GripperControlGoal` — ID `0x01`

- **ROS 话题**：`/task/gripper_control/_public_goal`
- **ROS 消息类型**：`std_msgs/msg/Bool`
- **数据段字节数（Len）**：`1`
- **注意事项**：GripperControl.action 的 Goal 映射。data=1 表示 task_complete=true。任务状态回传统一走 GenericStatusRx。

| 字节偏移 | 字段名 | C 类型 | 字节数 |
| :------: | :----- | :----- | :----: |
| 0 | `task_complete` | `uint8_t` | 1 |
| **1** | *(CRC8)* | `uint8_t` | 1 |

### `WeaponDockGoal` — ID `0x02`

- **ROS 话题**：`/task/weapon_docking/_public_goal`
- **ROS 消息类型**：`std_msgs/msg/Float32MultiArray`
- **数据段字节数（Len）**：`12`
- **注意事项**：武器对接 TX 映射。data[0]=x_error(相机坐标系 X 轴偏差), data[1]=y_error(Y 轴偏差), data[2]=pitch_error(Pitch 角度偏差)。任务状态回传统一走 GenericStatusRx。

| 字节偏移 | 字段名 | C 类型 | 字节数 |
| :------: | :----- | :----- | :----: |
| 0 | `x_error` | `float` | 4 |
| 4 | `y_error` | `float` | 4 |
| 8 | `pitch_error` | `float` | 4 |
| **12** | *(CRC8)* | `uint8_t` | 1 |

### `MlControlTx` — ID `0x03`

- **ROS 话题**：`/task/ml_control/_public_request`
- **ROS 消息类型**：`std_msgs/msg/Int32MultiArray`
- **数据段字节数（Len）**：`2`
- **注意事项**：MlControl.action 请求映射。data[0]=task_type(0=上台阶, 1=下台阶)，data[1]=stage_status(0=phase1 ... 4=phase5)。2.3 的 current_stage 不再来自独立 RX 包，而是由 0x20 GenericStatusRx 的状态转换控制。

| 字节偏移 | 字段名 | C 类型 | 字节数 |
| :------: | :----- | :----- | :----: |
| 0 | `task_type` | `uint8_t` | 1 |
| 1 | `stage_status` | `uint8_t` | 1 |
| **2** | *(CRC8)* | `uint8_t` | 1 |

### `MerlinPickGoal` — ID `0x04`

- **ROS 话题**：`/task/merlin_pick/_public_goal`
- **ROS 消息类型**：`geometry_msgs/msg/Pose`
- **数据段字节数（Len）**：`16`
- **注意事项**：MeilinPick.action 的 Goal 映射。orientation.z 复用为 yaw。任务状态回传统一走 GenericStatusRx。

| 字节偏移 | 字段名 | C 类型 | 字节数 |
| :------: | :----- | :----- | :----: |
| 0 | `x` | `float` | 4 |
| 4 | `y` | `float` | 4 |
| 8 | `z` | `float` | 4 |
| 12 | `yaw` | `float` | 4 |
| **16** | *(CRC8)* | `uint8_t` | 1 |

### `GridPlaceGoal` — ID `0x05`

- **ROS 话题**：`/task/grid_place/_public_goal`
- **ROS 消息类型**：`geometry_msgs/msg/Point`
- **数据段字节数（Len）**：`12`
- **注意事项**：GridPlace.action 的 Goal 映射。任务状态回传统一走 GenericStatusRx。

| 字节偏移 | 字段名 | C 类型 | 字节数 |
| :------: | :----- | :----- | :----: |
| 0 | `x` | `float` | 4 |
| 4 | `y` | `float` | 4 |
| 8 | `z` | `float` | 4 |
| **12** | *(CRC8)* | `uint8_t` | 1 |

### `GridAttackGoal` — ID `0x06`

- **ROS 话题**：`/task/grid_attack/_public_goal`
- **ROS 消息类型**：`geometry_msgs/msg/Point`
- **数据段字节数（Len）**：`12`
- **注意事项**：GridAttack.action 的 Goal 映射。任务状态回传统一走 GenericStatusRx。

| 字节偏移 | 字段名 | C 类型 | 字节数 |
| :------: | :----- | :----- | :----: |
| 0 | `x` | `float` | 4 |
| 4 | `y` | `float` | 4 |
| 8 | `z` | `float` | 4 |
| **12** | *(CRC8)* | `uint8_t` | 1 |

### `GenericStatusTx` — ID `0x19`

- **ROS 话题**：`/task/generic_status_tx`
- **ROS 消息类型**：`std_msgs/msg/Float32MultiArray`
- **数据段字节数（Len）**：`3`
- **注意事项**：统一状态 TX 话题，映射逻辑: [task_id, tx_id, tx_status]。约定: 结束=0, 开始=1。

| 字节偏移 | 字段名 | C 类型 | 字节数 |
| :------: | :----- | :----- | :----: |
| 0 | `task_id` | `uint8_t` | 1 |
| 1 | `tx_id` | `uint8_t` | 1 |
| 2 | `tx_status` | `uint8_t` | 1 |
| **3** | *(CRC8)* | `uint8_t` | 1 |

---

*文档由构建系统自动生成，版本以协议哈希为准。*
