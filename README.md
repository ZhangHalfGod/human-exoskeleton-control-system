# 人体外骨骼控制系统

## 项目概述
本项目旨在开发一套完整的人体外骨骼机器控制系统，包括材料系统和控制系统。项目采用模块化设计，从手臂系统优先实现，逐步扩展到背部脊椎系统、腿部系统，最终实现全身系统集成。

## 核心功能
- 多层次、分布式的智能控制架构
- 多种先进控制算法（自适应、模糊PID、MPC、阻抗控制）
- 实时通信网络（EtherCAT、CANopen）
- 高安全等级设计（ISO 13849-1 PLd，IEC 61508 SIL 3）
- 复杂的人机交互系统
- 高性能硬件平台（Nvidia Jetson AGX Xavier、TI DSP等）

## 目录结构
```
├── docs/                  # 文档目录
│   ├── architecture/      # 架构文档
│   ├── 3d_modeling/       # 3D建模文档
│   ├── development_roadmap/  # 开发路线图
│   └── user_manual/       # 用户手册
├── src/                   # 源代码目录
│   ├── control_algorithms/  # 控制算法实现
│   ├── communication/     # 通信架构实现
│   ├── safety/            # 安全机制实现
│   ├── human_interface/   # 人机交互实现
│   ├── hardware_drivers/  # 硬件驱动实现
│   └── software_framework/    # 软件框架
├── models/                # 3D模型目录
│   ├── arm/               # 手臂模块
│   ├── back/              # 背部模块
│   ├── leg/               # 腿部模块
│   └── spine/             # 脊椎模块
├── tests/                 # 测试目录
├── extern/                # 外部依赖
├── README.md
└── project_execution_plan.md
```

## 开发路线
请参考 docs/development_roadmap/ 目录下的详细开发计划。