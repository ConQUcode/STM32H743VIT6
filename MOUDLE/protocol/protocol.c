#include "protocol.h"
#include <string.h>

/* USER CODE BEGIN Includes */
/* USER CODE END Includes */


// 解析器状态定义
typedef enum {
    STATE_WAIT_HEADER1,
    STATE_WAIT_HEADER2,
    STATE_WAIT_ID,
    STATE_WAIT_LEN,
    STATE_WAIT_DATA,
    STATE_WAIT_CRC
} State;

static State rx_state = STATE_WAIT_HEADER1;
static uint8_t rx_buffer[256]; // 定义的最大包长
static uint16_t rx_cnt = 0;
static uint8_t rx_data_len = 0;
static uint8_t rx_id = 0;
static uint8_t rx_crc = 0;

// CRC8 计算函数 (查表法)
uint8_t calculate_crc8(const uint8_t* data, uint8_t len, uint8_t initial_crc) {
    uint8_t crc = initial_crc;
    for (uint8_t i = 0; i < len; i++) {
        crc = CRC8_TABLE[crc ^ data[i]];
    }
    return crc;
}

/* USER CODE BEGIN Private_Variables */
/* USER CODE END Private_Variables */


// 用户需要实现的回调函数
__attribute__((weak)) void on_receive_Heartbeat(const Packet_Heartbeat* pkt) {
/* USER CODE BEGIN on_receive_Heartbeat */
/* USER CODE END on_receive_Heartbeat */
}
__attribute__((weak)) void on_receive_Handshake(const Packet_Handshake* pkt) {
/* USER CODE BEGIN on_receive_Handshake */
/* USER CODE END on_receive_Handshake */
}
__attribute__((weak)) void on_receive_CmdVel(const Packet_CmdVel* pkt) {
/* USER CODE BEGIN on_receive_CmdVel */
/* USER CODE END on_receive_CmdVel */
}
__attribute__((weak)) void on_receive_GenericStatus(const Packet_GenericStatus* pkt) {
/* USER CODE BEGIN on_receive_GenericStatus */
/* USER CODE END on_receive_GenericStatus */
}
__attribute__((weak)) void on_receive_WeaponDockFeedback(const Packet_WeaponDockFeedback* pkt) {
/* USER CODE BEGIN on_receive_WeaponDockFeedback */
/* USER CODE END on_receive_WeaponDockFeedback */
}
__attribute__((weak)) void on_receive_StairPoseGoal(const Packet_StairPoseGoal* pkt) {
/* USER CODE BEGIN on_receive_StairPoseGoal */
/* USER CODE END on_receive_StairPoseGoal */
}
__attribute__((weak)) void on_receive_StairType(const Packet_StairType* pkt) {
/* USER CODE BEGIN on_receive_StairType */
/* USER CODE END on_receive_StairType */
}
__attribute__((weak)) void on_receive_MerlinPickGoal(const Packet_MerlinPickGoal* pkt) {
/* USER CODE BEGIN on_receive_MerlinPickGoal */
/* USER CODE END on_receive_MerlinPickGoal */
}
__attribute__((weak)) void on_receive_GridPlaceGoal(const Packet_GridPlaceGoal* pkt) {
/* USER CODE BEGIN on_receive_GridPlaceGoal */
/* USER CODE END on_receive_GridPlaceGoal */
}
__attribute__((weak)) void on_receive_GridAttackGoal(const Packet_GridAttackGoal* pkt) {
/* USER CODE BEGIN on_receive_GridAttackGoal */
/* USER CODE END on_receive_GridAttackGoal */
}

/* USER CODE BEGIN Code_0 */
/* USER CODE END Code_0 */


/**
 * @brief 协议解析状态机，在串口中断或轮询中调用此函数处理每个接收到的字节
 * @param byte 接收到的单个字节
 */
void protocol_fsm_feed(uint8_t byte) {
    switch (rx_state) {
        case STATE_WAIT_HEADER1:
            if (byte == FRAME_HEADER1) {
                rx_state = STATE_WAIT_HEADER2;
                rx_crc = 0; // CRC 重置，校验不包含 Frame Header
            }
            break;
            
        case STATE_WAIT_HEADER2:
            if (byte == FRAME_HEADER2) {
                rx_state = STATE_WAIT_ID;
            } else {
                rx_state = STATE_WAIT_HEADER1; // 重置
            }
            break;
            
        case STATE_WAIT_ID:
            rx_id = byte;
            rx_crc = CRC8_TABLE[0 ^ rx_id]; // 开始计算 CRC，校验包含 ID
            rx_state = STATE_WAIT_LEN;
            break;
            
        case STATE_WAIT_LEN:
            rx_data_len = byte;
            rx_crc = CRC8_TABLE[rx_crc ^ rx_data_len]; // CRC 计算，校验包含 Len
            rx_cnt = 0;
            if (rx_data_len > 0) {
                rx_state = STATE_WAIT_DATA;
            } else {
                rx_state = STATE_WAIT_CRC; // 数据长度为0的情况
            }
            break;
            
        case STATE_WAIT_DATA:
            rx_buffer[rx_cnt++] = byte;
            rx_crc = CRC8_TABLE[rx_crc ^ byte]; // CRC 计算，校验包含 Data
            if (rx_cnt >= rx_data_len) {
                rx_state = STATE_WAIT_CRC;
            }
            break;
            
        case STATE_WAIT_CRC:
            if (byte == rx_crc) {
                // 校验通过，分发数据
                switch (rx_id) {
                    case PACKET_ID_HEARTBEAT:
                        if (rx_data_len == sizeof(Packet_Heartbeat)) {
                            on_receive_Heartbeat((Packet_Heartbeat*)rx_buffer);
                        }
                        break;
                    case PACKET_ID_HANDSHAKE:
                        if (rx_data_len == sizeof(Packet_Handshake)) {
                            on_receive_Handshake((Packet_Handshake*)rx_buffer);
                        }
                        break;
                    case PACKET_ID_CMDVEL:
                        if (rx_data_len == sizeof(Packet_CmdVel)) {
                            on_receive_CmdVel((Packet_CmdVel*)rx_buffer);
                        }
                        break;
                    case PACKET_ID_GENERICSTATUS:
                        if (rx_data_len == sizeof(Packet_GenericStatus)) {
                            on_receive_GenericStatus((Packet_GenericStatus*)rx_buffer);
                        }
                        break;
                    case PACKET_ID_WEAPONDOCKFEEDBACK:
                        if (rx_data_len == sizeof(Packet_WeaponDockFeedback)) {
                            on_receive_WeaponDockFeedback((Packet_WeaponDockFeedback*)rx_buffer);
                        }
                        break;
                    case PACKET_ID_STAIRPOSEGOAL:
                        if (rx_data_len == sizeof(Packet_StairPoseGoal)) {
                            on_receive_StairPoseGoal((Packet_StairPoseGoal*)rx_buffer);
                        }
                        break;
                    case PACKET_ID_STAIRTYPE:
                        if (rx_data_len == sizeof(Packet_StairType)) {
                            on_receive_StairType((Packet_StairType*)rx_buffer);
                        }
                        break;
                    case PACKET_ID_MERLINPICKGOAL:
                        if (rx_data_len == sizeof(Packet_MerlinPickGoal)) {
                            on_receive_MerlinPickGoal((Packet_MerlinPickGoal*)rx_buffer);
                        }
                        break;
                    case PACKET_ID_GRIDPLACEGOAL:
                        if (rx_data_len == sizeof(Packet_GridPlaceGoal)) {
                            on_receive_GridPlaceGoal((Packet_GridPlaceGoal*)rx_buffer);
                        }
                        break;
                    case PACKET_ID_GRIDATTACKGOAL:
                        if (rx_data_len == sizeof(Packet_GridAttackGoal)) {
                            on_receive_GridAttackGoal((Packet_GridAttackGoal*)rx_buffer);
                        }
                        break;

                    default:
                        break;
                }
            }
            // 无论校验成功与否，都重置状态
            rx_state = STATE_WAIT_HEADER1;
            break;
            
        default:
            rx_state = STATE_WAIT_HEADER1;
            break;
    }
}

// --- 发送函数 ---
// 外部依赖：用户必须实现 void serial_write(const uint8_t* data, uint16_t len);
extern void serial_write(const uint8_t* data, uint16_t len);

void send_Heartbeat(const Packet_Heartbeat* pkt) {
    // Header(4) + Data(sizeof) + CRC(1)
    uint8_t buffer[4 + sizeof(Packet_Heartbeat) + 1];
    uint16_t idx = 0;
    
    // 1. Prepare Header
    buffer[idx++] = FRAME_HEADER1;
    buffer[idx++] = FRAME_HEADER2;
    buffer[idx++] = PACKET_ID_HEARTBEAT;
    buffer[idx++] = sizeof(Packet_Heartbeat);
    
    // 2. Copy Data
    memcpy(&buffer[idx], pkt, sizeof(Packet_Heartbeat));
    idx += sizeof(Packet_Heartbeat);
    
    // 3. Calculate CRC (ID + Len + Data)
    // ID is at buffer[2]
    // Count = 1(ID) + 1(Len) + sizeof(Data)
    uint8_t crc = 0;
    for(uint16_t i = 2; i < idx; i++) {
        crc = CRC8_TABLE[crc ^ buffer[i]];
    }
    buffer[idx++] = crc;
    
    // 4. Send Buffer
    serial_write(buffer, idx);
}
void send_Handshake(const Packet_Handshake* pkt) {
    // Header(4) + Data(sizeof) + CRC(1)
    uint8_t buffer[4 + sizeof(Packet_Handshake) + 1];
    uint16_t idx = 0;
    
    // 1. Prepare Header
    buffer[idx++] = FRAME_HEADER1;
    buffer[idx++] = FRAME_HEADER2;
    buffer[idx++] = PACKET_ID_HANDSHAKE;
    buffer[idx++] = sizeof(Packet_Handshake);
    
    // 2. Copy Data
    memcpy(&buffer[idx], pkt, sizeof(Packet_Handshake));
    idx += sizeof(Packet_Handshake);
    
    // 3. Calculate CRC (ID + Len + Data)
    // ID is at buffer[2]
    // Count = 1(ID) + 1(Len) + sizeof(Data)
    uint8_t crc = 0;
    for(uint16_t i = 2; i < idx; i++) {
        crc = CRC8_TABLE[crc ^ buffer[i]];
    }
    buffer[idx++] = crc;
    
    // 4. Send Buffer
    serial_write(buffer, idx);
}
void send_CmdVel(const Packet_CmdVel* pkt) {
    // Header(4) + Data(sizeof) + CRC(1)
    uint8_t buffer[4 + sizeof(Packet_CmdVel) + 1];
    uint16_t idx = 0;
    
    // 1. Prepare Header
    buffer[idx++] = FRAME_HEADER1;
    buffer[idx++] = FRAME_HEADER2;
    buffer[idx++] = PACKET_ID_CMDVEL;
    buffer[idx++] = sizeof(Packet_CmdVel);
    
    // 2. Copy Data
    memcpy(&buffer[idx], pkt, sizeof(Packet_CmdVel));
    idx += sizeof(Packet_CmdVel);
    
    // 3. Calculate CRC (ID + Len + Data)
    // ID is at buffer[2]
    // Count = 1(ID) + 1(Len) + sizeof(Data)
    uint8_t crc = 0;
    for(uint16_t i = 2; i < idx; i++) {
        crc = CRC8_TABLE[crc ^ buffer[i]];
    }
    buffer[idx++] = crc;
    
    // 4. Send Buffer
    serial_write(buffer, idx);
}
void send_GenericStatus(const Packet_GenericStatus* pkt) {
    // Header(4) + Data(sizeof) + CRC(1)
    uint8_t buffer[4 + sizeof(Packet_GenericStatus) + 1];
    uint16_t idx = 0;
    
    // 1. Prepare Header
    buffer[idx++] = FRAME_HEADER1;
    buffer[idx++] = FRAME_HEADER2;
    buffer[idx++] = PACKET_ID_GENERICSTATUS;
    buffer[idx++] = sizeof(Packet_GenericStatus);
    
    // 2. Copy Data
    memcpy(&buffer[idx], pkt, sizeof(Packet_GenericStatus));
    idx += sizeof(Packet_GenericStatus);
    
    // 3. Calculate CRC (ID + Len + Data)
    // ID is at buffer[2]
    // Count = 1(ID) + 1(Len) + sizeof(Data)
    uint8_t crc = 0;
    for(uint16_t i = 2; i < idx; i++) {
        crc = CRC8_TABLE[crc ^ buffer[i]];
    }
    buffer[idx++] = crc;
    
    // 4. Send Buffer
    serial_write(buffer, idx);
}
void send_WeaponDockFeedback(const Packet_WeaponDockFeedback* pkt) {
    // Header(4) + Data(sizeof) + CRC(1)
    uint8_t buffer[4 + sizeof(Packet_WeaponDockFeedback) + 1];
    uint16_t idx = 0;
    
    // 1. Prepare Header
    buffer[idx++] = FRAME_HEADER1;
    buffer[idx++] = FRAME_HEADER2;
    buffer[idx++] = PACKET_ID_WEAPONDOCKFEEDBACK;
    buffer[idx++] = sizeof(Packet_WeaponDockFeedback);
    
    // 2. Copy Data
    memcpy(&buffer[idx], pkt, sizeof(Packet_WeaponDockFeedback));
    idx += sizeof(Packet_WeaponDockFeedback);
    
    // 3. Calculate CRC (ID + Len + Data)
    // ID is at buffer[2]
    // Count = 1(ID) + 1(Len) + sizeof(Data)
    uint8_t crc = 0;
    for(uint16_t i = 2; i < idx; i++) {
        crc = CRC8_TABLE[crc ^ buffer[i]];
    }
    buffer[idx++] = crc;
    
    // 4. Send Buffer
    serial_write(buffer, idx);
}
void send_StairPoseGoal(const Packet_StairPoseGoal* pkt) {
    // Header(4) + Data(sizeof) + CRC(1)
    uint8_t buffer[4 + sizeof(Packet_StairPoseGoal) + 1];
    uint16_t idx = 0;
    
    // 1. Prepare Header
    buffer[idx++] = FRAME_HEADER1;
    buffer[idx++] = FRAME_HEADER2;
    buffer[idx++] = PACKET_ID_STAIRPOSEGOAL;
    buffer[idx++] = sizeof(Packet_StairPoseGoal);
    
    // 2. Copy Data
    memcpy(&buffer[idx], pkt, sizeof(Packet_StairPoseGoal));
    idx += sizeof(Packet_StairPoseGoal);
    
    // 3. Calculate CRC (ID + Len + Data)
    // ID is at buffer[2]
    // Count = 1(ID) + 1(Len) + sizeof(Data)
    uint8_t crc = 0;
    for(uint16_t i = 2; i < idx; i++) {
        crc = CRC8_TABLE[crc ^ buffer[i]];
    }
    buffer[idx++] = crc;
    
    // 4. Send Buffer
    serial_write(buffer, idx);
}
void send_StairType(const Packet_StairType* pkt) {
    // Header(4) + Data(sizeof) + CRC(1)
    uint8_t buffer[4 + sizeof(Packet_StairType) + 1];
    uint16_t idx = 0;
    
    // 1. Prepare Header
    buffer[idx++] = FRAME_HEADER1;
    buffer[idx++] = FRAME_HEADER2;
    buffer[idx++] = PACKET_ID_STAIRTYPE;
    buffer[idx++] = sizeof(Packet_StairType);
    
    // 2. Copy Data
    memcpy(&buffer[idx], pkt, sizeof(Packet_StairType));
    idx += sizeof(Packet_StairType);
    
    // 3. Calculate CRC (ID + Len + Data)
    // ID is at buffer[2]
    // Count = 1(ID) + 1(Len) + sizeof(Data)
    uint8_t crc = 0;
    for(uint16_t i = 2; i < idx; i++) {
        crc = CRC8_TABLE[crc ^ buffer[i]];
    }
    buffer[idx++] = crc;
    
    // 4. Send Buffer
    serial_write(buffer, idx);
}
void send_MerlinPickGoal(const Packet_MerlinPickGoal* pkt) {
    // Header(4) + Data(sizeof) + CRC(1)
    uint8_t buffer[4 + sizeof(Packet_MerlinPickGoal) + 1];
    uint16_t idx = 0;
    
    // 1. Prepare Header
    buffer[idx++] = FRAME_HEADER1;
    buffer[idx++] = FRAME_HEADER2;
    buffer[idx++] = PACKET_ID_MERLINPICKGOAL;
    buffer[idx++] = sizeof(Packet_MerlinPickGoal);
    
    // 2. Copy Data
    memcpy(&buffer[idx], pkt, sizeof(Packet_MerlinPickGoal));
    idx += sizeof(Packet_MerlinPickGoal);
    
    // 3. Calculate CRC (ID + Len + Data)
    // ID is at buffer[2]
    // Count = 1(ID) + 1(Len) + sizeof(Data)
    uint8_t crc = 0;
    for(uint16_t i = 2; i < idx; i++) {
        crc = CRC8_TABLE[crc ^ buffer[i]];
    }
    buffer[idx++] = crc;
    
    // 4. Send Buffer
    serial_write(buffer, idx);
}
void send_GridPlaceGoal(const Packet_GridPlaceGoal* pkt) {
    // Header(4) + Data(sizeof) + CRC(1)
    uint8_t buffer[4 + sizeof(Packet_GridPlaceGoal) + 1];
    uint16_t idx = 0;
    
    // 1. Prepare Header
    buffer[idx++] = FRAME_HEADER1;
    buffer[idx++] = FRAME_HEADER2;
    buffer[idx++] = PACKET_ID_GRIDPLACEGOAL;
    buffer[idx++] = sizeof(Packet_GridPlaceGoal);
    
    // 2. Copy Data
    memcpy(&buffer[idx], pkt, sizeof(Packet_GridPlaceGoal));
    idx += sizeof(Packet_GridPlaceGoal);
    
    // 3. Calculate CRC (ID + Len + Data)
    // ID is at buffer[2]
    // Count = 1(ID) + 1(Len) + sizeof(Data)
    uint8_t crc = 0;
    for(uint16_t i = 2; i < idx; i++) {
        crc = CRC8_TABLE[crc ^ buffer[i]];
    }
    buffer[idx++] = crc;
    
    // 4. Send Buffer
    serial_write(buffer, idx);
}
void send_GridAttackGoal(const Packet_GridAttackGoal* pkt) {
    // Header(4) + Data(sizeof) + CRC(1)
    uint8_t buffer[4 + sizeof(Packet_GridAttackGoal) + 1];
    uint16_t idx = 0;
    
    // 1. Prepare Header
    buffer[idx++] = FRAME_HEADER1;
    buffer[idx++] = FRAME_HEADER2;
    buffer[idx++] = PACKET_ID_GRIDATTACKGOAL;
    buffer[idx++] = sizeof(Packet_GridAttackGoal);
    
    // 2. Copy Data
    memcpy(&buffer[idx], pkt, sizeof(Packet_GridAttackGoal));
    idx += sizeof(Packet_GridAttackGoal);
    
    // 3. Calculate CRC (ID + Len + Data)
    // ID is at buffer[2]
    // Count = 1(ID) + 1(Len) + sizeof(Data)
    uint8_t crc = 0;
    for(uint16_t i = 2; i < idx; i++) {
        crc = CRC8_TABLE[crc ^ buffer[i]];
    }
    buffer[idx++] = crc;
    
    // 4. Send Buffer
    serial_write(buffer, idx);
}

/* USER CODE BEGIN Code_1 */
/* USER CODE END Code_1 */

/*
// --- 建议的消息发送模板 (以 Heartbeat 为例) ---
// 建议在定时器回调或主循环中以固定频率调用

void heartbeat_timer_callback(void) {
    static uint32_t hb_count = 0;
    Packet_Heartbeat pkt;
    pkt.count = hb_count++;
    send_Heartbeat(&pkt);
}
*/

