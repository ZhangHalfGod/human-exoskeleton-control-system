#ifndef SYNCHRONIZATION_H
#define SYNCHRONIZATION_H

#include <stdint.h>
#include <stdbool.h>

// 同步状态枚举
typedef enum {
    SYNC_STATE_UNSYNCHRONIZED = 0,
    SYNC_STATE_SYNCING = 1,
    SYNC_STATE_SYNCHRONIZED = 2,
    SYNC_STATE_ERROR = 3,
    SYNC_STATE_MAX
} SyncState;

// 同步类型枚举
typedef enum {
    SYNC_TYPE_HARDWARE = 0,       // 硬件同步
    SYNC_TYPE_SOFTWARE = 1,       // 软件同步
    SYNC_TYPE_NETWORK = 2,        // 网络同步
    SYNC_TYPE_MAX
} SyncType;

// 同步配置结构
typedef struct {
    SyncType sync_type;           // 同步类型
    uint32_t sync_period;         // 同步周期 (单位: 毫秒)
    uint32_t sync_timeout;        // 同步超时时间 (单位: 毫秒)
    uint32_t max_offset;          // 最大允许时间偏移 (单位: 微秒)
    bool enable_auto_recovery;     // 是否启用自动恢复
} SyncConfig_t;

// 同步统计信息
typedef struct {
    uint32_t sync_count;          // 成功同步次数
    uint32_t error_count;         // 同步错误次数
    uint32_t last_sync_time;      // 最后一次同步时间 (单位: 毫秒)
    int32_t current_offset;       // 当前时间偏移 (单位: 微秒)
    int32_t max_offset_recorded;  // 记录的最大时间偏移 (单位: 微秒)
    double avg_offset;           // 平均时间偏移 (单位: 微秒)
} SyncStats_t;

// 初始化同步模块
bool Synchronization_Init(const SyncConfig_t* config);

// 更新同步配置
bool Synchronization_UpdateConfig(const SyncConfig_t* config);

// 获取当前同步状态
SyncState Synchronization_GetState(void);

// 获取当前系统时间 (同步后的时间)
uint64_t Synchronization_GetCurrentTime(void);

// 获取当前时间偏移
int32_t Synchronization_GetCurrentOffset(void);

// 获取同步统计信息
bool Synchronization_GetStats(SyncStats_t* stats);

// 执行一次同步
bool Synchronization_PerformSync(void);

// 关闭同步模块
void Synchronization_Close(void);

#endif // SYNCHRONIZATION_H