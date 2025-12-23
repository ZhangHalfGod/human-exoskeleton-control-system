#ifndef PROTOCOL_STACK_H
#define PROTOCOL_STACK_H

#include <stdint.h>
#include <stdbool.h>

// 通信协议栈版本定义
#define PROTOCOL_VERSION_MAJOR 1
#define PROTOCOL_VERSION_MINOR 0
#define PROTOCOL_VERSION_PATCH 0

// 最大数据长度定义
#define MAX_PACKET_SIZE 1024
#define MAX_PAYLOAD_SIZE 980

// 协议类型枚举
typedef enum {
    PROTOCOL_ETHERCAT = 0,
    PROTOCOL_CANOPEN = 1,
    PROTOCOL_WIFI = 2,
    PROTOCOL_BLUETOOTH = 3,
    PROTOCOL_USB = 4,
    PROTOCOL_MAX
} ProtocolType;

// 数据类型枚举
typedef enum {
    DATA_TYPE_REAL_TIME = 0,       // 实时数据：关节位置、速度、力/力矩等
    DATA_TYPE_NON_REAL_TIME = 1,   // 非实时数据：系统状态、参数配置等
    DATA_TYPE_EVENT = 2,           // 事件数据：故障信息、紧急停止等
    DATA_TYPE_MAX
} DataType;

// 优先级枚举
typedef enum {
    PRIORITY_HIGH = 0,
    PRIORITY_MEDIUM = 1,
    PRIORITY_LOW = 2,
    PRIORITY_MAX
} PriorityLevel;

// 通信状态枚举
typedef enum {
    COMM_STATE_DISCONNECTED = 0,
    COMM_STATE_CONNECTING = 1,
    COMM_STATE_CONNECTED = 2,
    COMM_STATE_ERROR = 3,
    COMM_STATE_MAX
} CommunicationState;

// 基础数据包结构
typedef struct {
    uint8_t protocol_type;         // 协议类型
    uint8_t data_type;             // 数据类型
    uint8_t priority;              // 优先级
    uint16_t packet_id;            // 数据包ID
    uint32_t timestamp;            // 时间戳
    uint16_t source_id;            // 源ID
    uint16_t destination_id;       // 目标ID
    uint16_t payload_length;       // 有效载荷长度
    uint8_t payload[MAX_PAYLOAD_SIZE]; // 有效载荷数据
    uint32_t crc32;                // CRC32校验
} Packet_t;

// 初始化协议栈
bool ProtocolStack_Init(ProtocolType protocol_type);

// 发送数据包
bool ProtocolStack_SendPacket(const Packet_t* packet);

// 接收数据包
bool ProtocolStack_ReceivePacket(Packet_t* packet);

// 关闭协议栈
void ProtocolStack_Close(void);

// 获取通信状态
CommunicationState ProtocolStack_GetState(void);

// 获取协议版本
void ProtocolStack_GetVersion(uint8_t* major, uint8_t* minor, uint8_t* patch);

#endif // PROTOCOL_STACK_H