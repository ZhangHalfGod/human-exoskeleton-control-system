#ifndef DATA_TRANSFER_H
#define DATA_TRANSFER_H

#include "protocol_stack.h"

// 数据传输状态枚举
typedef enum {
    DATA_TRANSFER_IDLE = 0,
    DATA_TRANSFER_SENDING = 1,
    DATA_TRANSFER_RECEIVING = 2,
    DATA_TRANSFER_COMPLETED = 3,
    DATA_TRANSFER_ERROR = 4,
    DATA_TRANSFER_MAX
} DataTransferState;

// 实时数据结构 - 关节数据
typedef struct {
    float position;       // 位置 (单位: 弧度或米)
    float velocity;       // 速度 (单位: 弧度/秒或米/秒)
    float force;          // 力/力矩 (单位: N或N·m)
    float acceleration;   // 加速度 (单位: 弧度/秒²或米/秒²)
} JointData_t;

// 系统状态数据结构
typedef struct {
    uint8_t system_mode;      // 系统模式
    uint8_t battery_level;    // 电池电量 (0-100%)
    float temperature;        // 温度 (单位: °C)
    uint16_t error_code;      // 错误码
    uint8_t warning_flags;    // 警告标志
    uint32_t uptime;          // 运行时间 (单位: 秒)
} SystemState_t;

// 事件数据结构
typedef struct {
    uint16_t event_id;        // 事件ID
    uint8_t event_type;       // 事件类型
    uint8_t event_severity;   // 事件严重程度
    char event_description[128]; // 事件描述
} EventData_t;

// 初始化数据传输模块
bool DataTransfer_Init(void);

// 发送关节数据
bool DataTransfer_SendJointData(uint16_t joint_id, const JointData_t* joint_data, PriorityLevel priority);

// 接收关节数据
bool DataTransfer_ReceiveJointData(uint16_t* joint_id, JointData_t* joint_data);

// 发送系统状态数据
bool DataTransfer_SendSystemState(const SystemState_t* system_state, PriorityLevel priority);

// 接收系统状态数据
bool DataTransfer_ReceiveSystemState(SystemState_t* system_state);

// 发送事件数据
bool DataTransfer_SendEventData(const EventData_t* event_data, PriorityLevel priority);

// 接收事件数据
bool DataTransfer_ReceiveEventData(EventData_t* event_data);

// 发送自定义数据
bool DataTransfer_SendCustomData(uint16_t data_id, const uint8_t* data, uint16_t data_length, PriorityLevel priority);

// 接收自定义数据
bool DataTransfer_ReceiveCustomData(uint16_t* data_id, uint8_t* data, uint16_t* data_length);

// 获取数据传输状态
DataTransferState DataTransfer_GetState(void);

// 清空数据缓冲区
void DataTransfer_FlushBuffers(void);

#endif // DATA_TRANSFER_H