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
| 协议哈希（握手用）| `0x22FFBA3E` |

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
| 4+Len | CRC8 | 覆盖 ID + Len + Data，多项式 `0x31` |

---

## 电控 → ROS（电控主动发送）

### `Heartbeat` — ID `0x00`

- **ROS 话题**：`/task/heartbeat`
- **ROS 消息类型**：`std_msgs/msg/UInt32`
- **数据段字节数（Len）**：`4`
- **注意事项**：握手完成后，电控应以固定周期（建议 100ms）持续发送心跳，ROS 侧也会同频回应。count 字段单调递增，从 0 开始。

| 字节偏移 | 字段名 | C 类型 | 字节数 |
| :------: | :----- | :----- | :----: |
| 0 | `count` | `uint32_t` | 4 |
| **4** | *(CRC8)* | `uint8_t` | 1 |

### `Handshake` — ID `0xff`

- **ROS 话题**：`/task/handshake`
- **ROS 消息类型**：`std_msgs/msg/UInt32`
- **数据段字节数（Len）**：`4`
- **注意事项**：上电后 ROS 主动发起握手，电控收到后用相同的 protocol_hash 原样回复。握手通过后方可发送其他数据帧，且整个连接周期内只需执行一次。protocol_hash 值见【全局参数】。

| 字节偏移 | 字段名 | C 类型 | 字节数 |
| :------: | :----- | :----- | :----: |
| 0 | `protocol_hash` | `uint32_t` | 4 |
| **4** | *(CRC8)* | `uint8_t` | 1 |

### `GenericStatus` — ID `0x19`

- **ROS 话题**：`/task/generic_status`
- **ROS 消息类型**：`std_msgs/msg/Float32MultiArray`
- **数据段字节数（Len）**：`20`

| 字节偏移 | 字段名 | C 类型 | 字节数 |
| :------: | :----- | :----- | :----: |
| 0 | `task_index` | `float` | 4 |
| 4 | `tx_id` | `float` | 4 |
| 8 | `tx_status` | `float` | 4 |
| 12 | `rx_id` | `float` | 4 |
| 16 | `rx_status` | `float` | 4 |
| **20** | *(CRC8)* | `uint8_t` | 1 |

---

## ROS → 电控（电控被动接收）

### `Heartbeat` — ID `0x00`

- **ROS 话题**：`/task/heartbeat`
- **ROS 消息类型**：`std_msgs/msg/UInt32`
- **数据段字节数（Len）**：`4`
- **注意事项**：握手完成后，电控应以固定周期（建议 100ms）持续发送心跳，ROS 侧也会同频回应。count 字段单调递增，从 0 开始。

| 字节偏移 | 字段名 | C 类型 | 字节数 |
| :------: | :----- | :----- | :----: |
| 0 | `count` | `uint32_t` | 4 |
| **4** | *(CRC8)* | `uint8_t` | 1 |

### `Handshake` — ID `0xff`

- **ROS 话题**：`/task/handshake`
- **ROS 消息类型**：`std_msgs/msg/UInt32`
- **数据段字节数（Len）**：`4`
- **注意事项**：上电后 ROS 主动发起握手，电控收到后用相同的 protocol_hash 原样回复。握手通过后方可发送其他数据帧，且整个连接周期内只需执行一次。protocol_hash 值见【全局参数】。

| 字节偏移 | 字段名 | C 类型 | 字节数 |
| :------: | :----- | :----- | :----: |
| 0 | `protocol_hash` | `uint32_t` | 4 |
| **4** | *(CRC8)* | `uint8_t` | 1 |

### `CmdVel` — ID `0x01`

- **ROS 话题**：`/cmd_vel`
- **ROS 消息类型**：`geometry_msgs/msg/Twist`
- **数据段字节数（Len）**：`12`

| 字节偏移 | 字段名 | C 类型 | 字节数 |
| :------: | :----- | :----- | :----: |
| 0 | `linear_x` | `float` | 4 |
| 4 | `linear_y` | `float` | 4 |
| 8 | `angular_z` | `float` | 4 |
| **12** | *(CRC8)* | `uint8_t` | 1 |

### `GenericStatus` — ID `0x19`

- **ROS 话题**：`/task/generic_status`
- **ROS 消息类型**：`std_msgs/msg/Float32MultiArray`
- **数据段字节数（Len）**：`20`

| 字节偏移 | 字段名 | C 类型 | 字节数 |
| :------: | :----- | :----- | :----: |
| 0 | `task_index` | `float` | 4 |
| 4 | `tx_id` | `float` | 4 |
| 8 | `tx_status` | `float` | 4 |
| 12 | `rx_id` | `float` | 4 |
| 16 | `rx_status` | `float` | 4 |
| **20** | *(CRC8)* | `uint8_t` | 1 |

### `WeaponDockFeedback` — ID `0x02`

- **ROS 话题**：`/task/weapon_docking/_public_feedback`
- **ROS 消息类型**：`geometry_msgs/msg/Vector3`
- **数据段字节数（Len）**：`12`

| 字节偏移 | 字段名 | C 类型 | 字节数 |
| :------: | :----- | :----- | :----: |
| 0 | `x_error` | `float` | 4 |
| 4 | `y_error` | `float` | 4 |
| 8 | `pitch_error` | `float` | 4 |
| **12** | *(CRC8)* | `uint8_t` | 1 |

### `StairPoseGoal` — ID `0x03`

- **ROS 话题**：`/perception/stair_pose/_public_goal`
- **ROS 消息类型**：`geometry_msgs/msg/Vector3`
- **数据段字节数（Len）**：`8`

| 字节偏移 | 字段名 | C 类型 | 字节数 |
| :------: | :----- | :----- | :----: |
| 0 | `target_dist` | `float` | 4 |
| 4 | `yaw` | `float` | 4 |
| **8** | *(CRC8)* | `uint8_t` | 1 |

### `StairType` — ID `0x04`

- **ROS 话题**：`/perception/stair_type`
- **ROS 消息类型**：`std_msgs/msg/Int32`
- **数据段字节数（Len）**：`4`

| 字节偏移 | 字段名 | C 类型 | 字节数 |
| :------: | :----- | :----- | :----: |
| 0 | `task_type` | `int32_t` | 4 |
| **4** | *(CRC8)* | `uint8_t` | 1 |

### `MerlinPickGoal` — ID `0x05`

- **ROS 话题**：`/task/merlin_pick/_public_goal`
- **ROS 消息类型**：`geometry_msgs/msg/Pose`
- **数据段字节数（Len）**：`16`

| 字节偏移 | 字段名 | C 类型 | 字节数 |
| :------: | :----- | :----- | :----: |
| 0 | `x` | `float` | 4 |
| 4 | `y` | `float` | 4 |
| 8 | `z` | `float` | 4 |
| 12 | `yaw` | `float` | 4 |
| **16** | *(CRC8)* | `uint8_t` | 1 |

### `GridPlaceGoal` — ID `0x06`

- **ROS 话题**：`/task/grid_place/_public_goal`
- **ROS 消息类型**：`geometry_msgs/msg/Point`
- **数据段字节数（Len）**：`12`

| 字节偏移 | 字段名 | C 类型 | 字节数 |
| :------: | :----- | :----- | :----: |
| 0 | `x` | `float` | 4 |
| 4 | `y` | `float` | 4 |
| 8 | `z` | `float` | 4 |
| **12** | *(CRC8)* | `uint8_t` | 1 |

### `GridAttackGoal` — ID `0x07`

- **ROS 话题**：`/task/grid_attack/_public_goal`
- **ROS 消息类型**：`geometry_msgs/msg/Point`
- **数据段字节数（Len）**：`12`

| 字节偏移 | 字段名 | C 类型 | 字节数 |
| :------: | :----- | :----- | :----: |
| 0 | `x` | `float` | 4 |
| 4 | `y` | `float` | 4 |
| 8 | `z` | `float` | 4 |
| **12** | *(CRC8)* | `uint8_t` | 1 |

---

*文档由构建系统自动生成，版本以协议哈希为准。*
