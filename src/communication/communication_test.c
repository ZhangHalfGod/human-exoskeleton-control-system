#include <stdio.h>
#include "protocol_stack.h"
#include "data_transfer.h"
#include "synchronization.h"

int main(void)
{
    printf("=== 人体外骨骼控制系统通信模块测试 ===\n\n");
    
    // 1. 测试协议栈初始化
    printf("1. 测试协议栈初始化...\n");
    if (ProtocolStack_Init(PROTOCOL_CANOPEN))
    {
        printf("   ✅ 协议栈初始化成功\n");
    }
    else
    {
        printf("   ❌ 协议栈初始化失败\n");
        return -1;
    }
    
    // 2. 测试同步模块初始化
    printf("\n2. 测试同步模块初始化...\n");
    SyncConfig_t sync_config = {
        .sync_type = SYNC_TYPE_NETWORK,
        .sync_period = 1000,       // 1秒同步一次
        .sync_timeout = 5000,       // 5秒超时
        .max_offset = 100,          // 最大允许偏移100微秒
        .enable_auto_recovery = true // 启用自动恢复
    };
    
    if (Synchronization_Init(&sync_config))
    {
        printf("   ✅ 同步模块初始化成功\n");
        SyncStats_t stats;
        if (Synchronization_GetStats(&stats))
        {
            printf("   同步统计信息：\n");
            printf("   - 同步次数：%d\n", stats.sync_count);
            printf("   - 错误次数：%d\n", stats.error_count);
            printf("   - 当前偏移：%d 微秒\n", stats.current_offset);
        }
    }
    else
    {
        printf("   ❌ 同步模块初始化失败\n");
    }
    
    // 3. 测试数据传输模块初始化
    printf("\n3. 测试数据传输模块初始化...\n");
    if (DataTransfer_Init())
    {
        printf("   ✅ 数据传输模块初始化成功\n");
    }
    else
    {
        printf("   ❌ 数据传输模块初始化失败\n");
    }
    
    // 4. 测试关节数据发送
    printf("\n4. 测试关节数据发送...\n");
    JointData_t joint_data = {
        .position = 1.57,       // 90度
        .velocity = 0.5,
        .force = 10.5,
        .acceleration = 0.1
    };
    
    if (DataTransfer_SendJointData(1, &joint_data, PRIORITY_HIGH))
    {
        printf("   ✅ 关节数据发送成功\n");
    }
    else
    {
        printf("   ❌ 关节数据发送失败\n");
    }
    
    // 5. 测试系统状态数据发送
    printf("\n5. 测试系统状态数据发送...\n");
    SystemState_t system_state = {
        .system_mode = 0,
        .battery_level = 85,
        .temperature = 35.5,
        .error_code = 0,
        .warning_flags = 0,
        .uptime = 3600
    };
    
    if (DataTransfer_SendSystemState(&system_state, PRIORITY_MEDIUM))
    {
        printf("   ✅ 系统状态数据发送成功\n");
    }
    else
    {
        printf("   ❌ 系统状态数据发送失败\n");
    }
    
    // 6. 测试事件数据发送
    printf("\n6. 测试事件数据发送...\n");
    EventData_t event_data = {
        .event_id = 1001,
        .event_type = 0,
        .event_severity = 1,
        .event_description = "测试事件：系统启动成功"
    };
    
    if (DataTransfer_SendEventData(&event_data, PRIORITY_HIGH))
    {
        printf("   ✅ 事件数据发送成功\n");
    }
    else
    {
        printf("   ❌ 事件数据发送失败\n");
    }
    
    // 7. 测试自定义数据发送
    printf("\n7. 测试自定义数据发送...\n");
    uint8_t custom_data[] = {0x01, 0x02, 0x03, 0x04, 0x05};
    if (DataTransfer_SendCustomData(0x1001, custom_data, sizeof(custom_data), PRIORITY_LOW))
    {
        printf("   ✅ 自定义数据发送成功\n");
    }
    else
    {
        printf("   ❌ 自定义数据发送失败\n");
    }
    
    // 8. 测试协议栈关闭
    printf("\n8. 测试协议栈关闭...\n");
    ProtocolStack_Close();
    printf("   ✅ 协议栈关闭成功\n");
    
    // 9. 测试同步模块关闭
    printf("\n9. 测试同步模块关闭...\n");
    Synchronization_Close();
    printf("   ✅ 同步模块关闭成功\n");
    
    printf("\n=== 通信模块测试完成 ===\n");
    return 0;
}
