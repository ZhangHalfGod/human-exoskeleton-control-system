#include "synchronization.h"
#include <stdio.h>
#include <string.h>
#include <time.h>

// 全局变量定义
static SyncConfig_t g_sync_config;
static SyncState g_sync_state = SYNC_STATE_UNSYNCHRONIZED;
static SyncStats_t g_sync_stats;
static uint64_t g_base_time = 0;       // 基础时间 (单位: 微秒)
static int32_t g_time_offset = 0;      // 时间偏移 (单位: 微秒)
static uint64_t g_last_sync_time = 0;  // 最后一次同步时间 (单位: 微秒)

// 获取系统当前时间 (原始时间，未同步)
static uint64_t get_raw_system_time(void)
{
    // 待实现：获取系统当前时间，单位为微秒
    // 示例实现：使用系统时钟
    return (uint64_t)clock() * 1000000 / CLOCKS_PER_SEC;
}

// 初始化同步模块
bool Synchronization_Init(const SyncConfig_t* config)
{
    if (config == NULL)
    {
        g_sync_state = SYNC_STATE_ERROR;
        return false;
    }
    
    // 复制配置
    memcpy(&g_sync_config, config, sizeof(SyncConfig_t));
    
    // 初始化统计信息
    memset(&g_sync_stats, 0, sizeof(SyncStats_t));
    
    // 设置基础时间
    g_base_time = get_raw_system_time();
    g_time_offset = 0;
    g_last_sync_time = g_base_time;
    
    // 开始同步
    g_sync_state = SYNC_STATE_SYNCING;
    
    // 执行第一次同步
    if (Synchronization_PerformSync())
    {
        g_sync_state = SYNC_STATE_SYNCHRONIZED;
        return true;
    }
    else
    {
        g_sync_state = SYNC_STATE_ERROR;
        return false;
    }
}

// 更新同步配置
bool Synchronization_UpdateConfig(const SyncConfig_t* config)
{
    if (config == NULL)
    {
        return false;
    }
    
    // 复制新配置
    memcpy(&g_sync_config, config, sizeof(SyncConfig_t));
    
    return true;
}

// 获取当前同步状态
SyncState Synchronization_GetState(void)
{
    // 检查同步是否超时
    uint64_t current_time = get_raw_system_time();
    if (g_sync_state == SYNC_STATE_SYNCHRONIZED)
    {
        uint64_t time_since_last_sync = current_time - g_last_sync_time;
        if (time_since_last_sync > (uint64_t)g_sync_config.sync_timeout * 1000)
        {
            g_sync_state = SYNC_STATE_UNSYNCHRONIZED;
            if (g_sync_config.enable_auto_recovery)
            {
                g_sync_state = SYNC_STATE_SYNCING;
                Synchronization_PerformSync();
            }
        }
    }
    
    return g_sync_state;
}

// 获取当前系统时间 (同步后的时间)
uint64_t Synchronization_GetCurrentTime(void)
{
    uint64_t raw_time = get_raw_system_time();
    return raw_time + g_time_offset;
}

// 获取当前时间偏移
int32_t Synchronization_GetCurrentOffset(void)
{
    return g_time_offset;
}

// 获取同步统计信息
bool Synchronization_GetStats(SyncStats_t* stats)
{
    if (stats == NULL)
    {
        return false;
    }
    
    // 更新统计信息
    g_sync_stats.last_sync_time = (uint32_t)(g_last_sync_time / 1000); // 转换为毫秒
    g_sync_stats.current_offset = g_time_offset;
    
    // 复制统计信息
    memcpy(stats, &g_sync_stats, sizeof(SyncStats_t));
    
    return true;
}

// 执行一次同步
bool Synchronization_PerformSync(void)
{
    // 根据同步类型执行不同的同步逻辑
    bool sync_success = false;
    int32_t new_offset = 0;
    
    switch (g_sync_config.sync_type)
    {
        case SYNC_TYPE_HARDWARE:
            // 硬件同步实现
            // 待实现：使用硬件时钟同步
            new_offset = 0;
            sync_success = true;
            break;
            
        case SYNC_TYPE_SOFTWARE:
            // 软件同步实现
            // 待实现：使用软件算法同步
            new_offset = 0;
            sync_success = true;
            break;
            
        case SYNC_TYPE_NETWORK:
            // 网络同步实现
            // 待实现：使用网络协议同步（如NTP、PTP等）
            new_offset = 0;
            sync_success = true;
            break;
            
        default:
            sync_success = false;
            break;
    }
    
    if (sync_success)
    {
        // 更新同步状态
        g_sync_state = SYNC_STATE_SYNCHRONIZED;
        
        // 更新时间偏移
        g_time_offset = new_offset;
        
        // 更新最后同步时间
        g_last_sync_time = get_raw_system_time();
        
        // 更新统计信息
        g_sync_stats.sync_count++;
        
        // 更新平均偏移
        g_sync_stats.avg_offset = (g_sync_stats.avg_offset * (g_sync_stats.sync_count - 1) + abs(new_offset)) / g_sync_stats.sync_count;
        
        // 更新最大偏移
        if (abs(new_offset) > g_sync_stats.max_offset_recorded)
        {
            g_sync_stats.max_offset_recorded = abs(new_offset);
        }
        
        // 检查偏移是否超过最大允许值
        if (abs(new_offset) > (int32_t)g_sync_config.max_offset)
        {
            printf("Warning: Time offset exceeds maximum allowed value: %d microseconds\n", new_offset);
        }
    }
    else
    {
        // 更新错误统计
        g_sync_stats.error_count++;
        
        // 如果启用自动恢复，保持SYNCING状态；否则转为ERROR状态
        if (!g_sync_config.enable_auto_recovery)
        {
            g_sync_state = SYNC_STATE_ERROR;
        }
    }
    
    return sync_success;
}

// 关闭同步模块
void Synchronization_Close(void)
{
    g_sync_state = SYNC_STATE_UNSYNCHRONIZED;
    g_time_offset = 0;
    g_base_time = 0;
    g_last_sync_time = 0;
    
    // 清空统计信息
    memset(&g_sync_stats, 0, sizeof(SyncStats_t));
}
