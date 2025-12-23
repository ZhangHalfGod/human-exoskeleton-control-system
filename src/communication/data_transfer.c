#include "data_transfer.h"
#include <string.h>

// 全局变量定义
static DataTransferState g_transfer_state = DATA_TRANSFER_IDLE;
static uint16_t g_packet_counter = 0;

// 数据类型ID定义
#define DATA_TYPE_JOINT     0x01
#define DATA_TYPE_SYSTEM    0x02
#define DATA_TYPE_EVENT     0x03
#define DATA_TYPE_CUSTOM    0x04

// 生成唯一数据包ID
static uint16_t generate_packet_id(void)
{
    return g_packet_counter++;
}

// 初始化数据传输模块
bool DataTransfer_Init(void)
{
    g_transfer_state = DATA_TRANSFER_IDLE;
    g_packet_counter = 0;
    return true;
}

// 发送关节数据
bool DataTransfer_SendJointData(uint16_t joint_id, const JointData_t* joint_data, PriorityLevel priority)
{
    if (joint_data == NULL)
    {
        g_transfer_state = DATA_TRANSFER_ERROR;
        return false;
    }
    
    // 创建数据包
    Packet_t packet;
    memset(&packet, 0, sizeof(Packet_t));
    
    // 设置数据包头部
    packet.protocol_type = PROTOCOL_CANOPEN; // 默认使用CANopen协议
    packet.data_type = DATA_TYPE_REAL_TIME;
    packet.priority = priority;
    packet.packet_id = generate_packet_id();
    packet.timestamp = 0; // 待实现：获取系统时间戳
    packet.source_id = 0x0001; // 待实现：获取本地设备ID
    packet.destination_id = 0x0002; // 待实现：获取目标设备ID
    
    // 打包关节数据
    uint8_t* payload_ptr = packet.payload;
    
    // 写入关节ID
    memcpy(payload_ptr, &joint_id, sizeof(uint16_t));
    payload_ptr += sizeof(uint16_t);
    
    // 写入关节数据
    memcpy(payload_ptr, joint_data, sizeof(JointData_t));
    payload_ptr += sizeof(JointData_t);
    
    // 设置有效载荷长度
    packet.payload_length = payload_ptr - packet.payload;
    
    // 计算CRC32校验
    packet.crc32 = 0; // 先清零CRC字段
    packet.crc32 = crc32_calculate((const uint8_t*)&packet, sizeof(Packet_t) - sizeof(packet.crc32));
    
    // 发送数据包
    g_transfer_state = DATA_TRANSFER_SENDING;
    bool result = ProtocolStack_SendPacket(&packet);
    
    if (result)
    {
        g_transfer_state = DATA_TRANSFER_COMPLETED;
    }
    else
    {
        g_transfer_state = DATA_TRANSFER_ERROR;
    }
    
    return result;
}

// 接收关节数据
bool DataTransfer_ReceiveJointData(uint16_t* joint_id, JointData_t* joint_data)
{
    if (joint_id == NULL || joint_data == NULL)
    {
        g_transfer_state = DATA_TRANSFER_ERROR;
        return false;
    }
    
    // 接收数据包
    Packet_t packet;
    g_transfer_state = DATA_TRANSFER_RECEIVING;
    bool result = ProtocolStack_ReceivePacket(&packet);
    
    if (!result)
    {
        g_transfer_state = DATA_TRANSFER_ERROR;
        return false;
    }
    
    // 检查数据包类型
    if (packet.data_type != DATA_TYPE_REAL_TIME)
    {
        g_transfer_state = DATA_TRANSFER_ERROR;
        return false;
    }
    
    // 解包关节数据
    const uint8_t* payload_ptr = packet.payload;
    
    // 读取关节ID
    memcpy(joint_id, payload_ptr, sizeof(uint16_t));
    payload_ptr += sizeof(uint16_t);
    
    // 读取关节数据
    memcpy(joint_data, payload_ptr, sizeof(JointData_t));
    
    g_transfer_state = DATA_TRANSFER_COMPLETED;
    return true;
}

// 发送系统状态数据
bool DataTransfer_SendSystemState(const SystemState_t* system_state, PriorityLevel priority)
{
    if (system_state == NULL)
    {
        g_transfer_state = DATA_TRANSFER_ERROR;
        return false;
    }
    
    // 创建数据包
    Packet_t packet;
    memset(&packet, 0, sizeof(Packet_t));
    
    // 设置数据包头部
    packet.protocol_type = PROTOCOL_ETHERCAT; // 默认使用EtherCAT协议
    packet.data_type = DATA_TYPE_NON_REAL_TIME;
    packet.priority = priority;
    packet.packet_id = generate_packet_id();
    packet.timestamp = 0; // 待实现：获取系统时间戳
    packet.source_id = 0x0001; // 待实现：获取本地设备ID
    packet.destination_id = 0x0003; // 待实现：获取目标设备ID
    
    // 打包系统状态数据
    memcpy(packet.payload, system_state, sizeof(SystemState_t));
    packet.payload_length = sizeof(SystemState_t);
    
    // 计算CRC32校验
    packet.crc32 = 0; // 先清零CRC字段
    packet.crc32 = crc32_calculate((const uint8_t*)&packet, sizeof(Packet_t) - sizeof(packet.crc32));
    
    // 发送数据包
    g_transfer_state = DATA_TRANSFER_SENDING;
    bool result = ProtocolStack_SendPacket(&packet);
    
    if (result)
    {
        g_transfer_state = DATA_TRANSFER_COMPLETED;
    }
    else
    {
        g_transfer_state = DATA_TRANSFER_ERROR;
    }
    
    return result;
}

// 接收系统状态数据
bool DataTransfer_ReceiveSystemState(SystemState_t* system_state)
{
    if (system_state == NULL)
    {
        g_transfer_state = DATA_TRANSFER_ERROR;
        return false;
    }
    
    // 接收数据包
    Packet_t packet;
    g_transfer_state = DATA_TRANSFER_RECEIVING;
    bool result = ProtocolStack_ReceivePacket(&packet);
    
    if (!result)
    {
        g_transfer_state = DATA_TRANSFER_ERROR;
        return false;
    }
    
    // 检查数据包类型
    if (packet.data_type != DATA_TYPE_NON_REAL_TIME)
    {
        g_transfer_state = DATA_TRANSFER_ERROR;
        return false;
    }
    
    // 解包系统状态数据
    memcpy(system_state, packet.payload, sizeof(SystemState_t));
    
    g_transfer_state = DATA_TRANSFER_COMPLETED;
    return true;
}

// 发送事件数据
bool DataTransfer_SendEventData(const EventData_t* event_data, PriorityLevel priority)
{
    if (event_data == NULL)
    {
        g_transfer_state = DATA_TRANSFER_ERROR;
        return false;
    }
    
    // 创建数据包
    Packet_t packet;
    memset(&packet, 0, sizeof(Packet_t));
    
    // 设置数据包头部
    packet.protocol_type = PROTOCOL_WIFI; // 默认使用WiFi协议
    packet.data_type = DATA_TYPE_EVENT;
    packet.priority = priority;
    packet.packet_id = generate_packet_id();
    packet.timestamp = 0; // 待实现：获取系统时间戳
    packet.source_id = 0x0001; // 待实现：获取本地设备ID
    packet.destination_id = 0x0004; // 待实现：获取目标设备ID
    
    // 打包事件数据
    memcpy(packet.payload, event_data, sizeof(EventData_t));
    packet.payload_length = sizeof(EventData_t);
    
    // 计算CRC32校验
    packet.crc32 = 0; // 先清零CRC字段
    packet.crc32 = crc32_calculate((const uint8_t*)&packet, sizeof(Packet_t) - sizeof(packet.crc32));
    
    // 发送数据包
    g_transfer_state = DATA_TRANSFER_SENDING;
    bool result = ProtocolStack_SendPacket(&packet);
    
    if (result)
    {
        g_transfer_state = DATA_TRANSFER_COMPLETED;
    }
    else
    {
        g_transfer_state = DATA_TRANSFER_ERROR;
    }
    
    return result;
}

// 接收事件数据
bool DataTransfer_ReceiveEventData(EventData_t* event_data)
{
    if (event_data == NULL)
    {
        g_transfer_state = DATA_TRANSFER_ERROR;
        return false;
    }
    
    // 接收数据包
    Packet_t packet;
    g_transfer_state = DATA_TRANSFER_RECEIVING;
    bool result = ProtocolStack_ReceivePacket(&packet);
    
    if (!result)
    {
        g_transfer_state = DATA_TRANSFER_ERROR;
        return false;
    }
    
    // 检查数据包类型
    if (packet.data_type != DATA_TYPE_EVENT)
    {
        g_transfer_state = DATA_TRANSFER_ERROR;
        return false;
    }
    
    // 解包事件数据
    memcpy(event_data, packet.payload, sizeof(EventData_t));
    
    g_transfer_state = DATA_TRANSFER_COMPLETED;
    return true;
}

// 发送自定义数据
bool DataTransfer_SendCustomData(uint16_t data_id, const uint8_t* data, uint16_t data_length, PriorityLevel priority)
{
    if (data == NULL || data_length == 0 || data_length > MAX_PAYLOAD_SIZE - sizeof(uint16_t))
    {
        g_transfer_state = DATA_TRANSFER_ERROR;
        return false;
    }
    
    // 创建数据包
    Packet_t packet;
    memset(&packet, 0, sizeof(Packet_t));
    
    // 设置数据包头部
    packet.protocol_type = PROTOCOL_USB; // 默认使用USB协议
    packet.data_type = DATA_TYPE_NON_REAL_TIME;
    packet.priority = priority;
    packet.packet_id = generate_packet_id();
    packet.timestamp = 0; // 待实现：获取系统时间戳
    packet.source_id = 0x0001; // 待实现：获取本地设备ID
    packet.destination_id = 0x0005; // 待实现：获取目标设备ID
    
    // 打包自定义数据
    uint8_t* payload_ptr = packet.payload;
    
    // 写入数据ID
    memcpy(payload_ptr, &data_id, sizeof(uint16_t));
    payload_ptr += sizeof(uint16_t);
    
    // 写入自定义数据
    memcpy(payload_ptr, data, data_length);
    payload_ptr += data_length;
    
    // 设置有效载荷长度
    packet.payload_length = payload_ptr - packet.payload;
    
    // 计算CRC32校验
    packet.crc32 = 0; // 先清零CRC字段
    packet.crc32 = crc32_calculate((const uint8_t*)&packet, sizeof(Packet_t) - sizeof(packet.crc32));
    
    // 发送数据包
    g_transfer_state = DATA_TRANSFER_SENDING;
    bool result = ProtocolStack_SendPacket(&packet);
    
    if (result)
    {
        g_transfer_state = DATA_TRANSFER_COMPLETED;
    }
    else
    {
        g_transfer_state = DATA_TRANSFER_ERROR;
    }
    
    return result;
}

// 接收自定义数据
bool DataTransfer_ReceiveCustomData(uint16_t* data_id, uint8_t* data, uint16_t* data_length)
{
    if (data_id == NULL || data == NULL || data_length == NULL)
    {
        g_transfer_state = DATA_TRANSFER_ERROR;
        return false;
    }
    
    // 接收数据包
    Packet_t packet;
    g_transfer_state = DATA_TRANSFER_RECEIVING;
    bool result = ProtocolStack_ReceivePacket(&packet);
    
    if (!result)
    {
        g_transfer_state = DATA_TRANSFER_ERROR;
        return false;
    }
    
    // 检查数据包类型
    if (packet.data_type != DATA_TYPE_NON_REAL_TIME)
    {
        g_transfer_state = DATA_TRANSFER_ERROR;
        return false;
    }
    
    // 解包自定义数据
    const uint8_t* payload_ptr = packet.payload;
    
    // 读取数据ID
    memcpy(data_id, payload_ptr, sizeof(uint16_t));
    payload_ptr += sizeof(uint16_t);
    
    // 读取自定义数据
    uint16_t actual_data_length = packet.payload_length - sizeof(uint16_t);
    if (actual_data_length > *data_length)
    {
        g_transfer_state = DATA_TRANSFER_ERROR;
        return false;
    }
    
    memcpy(data, payload_ptr, actual_data_length);
    *data_length = actual_data_length;
    
    g_transfer_state = DATA_TRANSFER_COMPLETED;
    return true;
}

// 获取数据传输状态
DataTransferState DataTransfer_GetState(void)
{
    return g_transfer_state;
}

// 清空数据缓冲区
void DataTransfer_FlushBuffers(void)
{
    // 待实现：清空协议栈的发送和接收缓冲区
    g_transfer_state = DATA_TRANSFER_IDLE;
}
