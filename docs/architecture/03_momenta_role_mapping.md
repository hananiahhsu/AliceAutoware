# 03. Mapping to Momenta Role Requirements

## 参考到的岗位画像

综合 Momenta 公开岗位信息，可以抽取出几类高频能力：

1. 自动驾驶车端模块研发、算法集成、功能开发与调试
2. Linux 下 C/C++ 编程，Python / Shell 脚本能力
3. 熟悉 CMake / GDB / 调试开发环境
4. 对感知、预测、规划、控制、地图、定位至少一个方向有实际经验
5. 支持量产项目交付，强调软件工程化、质量、效率与自动化工具链
6. 规划算法岗位强调图搜索、轨迹规划/优化、控制、不确定性下规划
7. 融合/感知岗位强调多传感器感知、工程落地与 C++/Python 能力

## 本仓库如何对应

| Momenta 能力要求 | 本仓库落点 |
|---|---|
| Linux / C++ / Python / Shell | C++17 核心模块 + Python 工具 + shell/bat 脚本 |
| CMake / GDB / 工程调试 | CMake 工程、CTest、CI、目录分层 |
| 自动驾驶模块研发 | perception / prediction / planning / control / runtime |
| 算法集成 | `AutonomyStack` 完成整条闭环装配 |
| 自动化工具链 | build/package/check/evaluate/render 脚本齐全 |
| 量产思维 | 模块清晰、日志输出、可回放、可评测、可扩展 |
| 规划控制能力 | 跟车 / 变道 / 轨迹生成 / PID + 横向控制 |
| 仿真研发 | `simulation::World` + HTML 回放 |

## 还没有覆盖但建议下一步扩展的部分

1. ROS2 通信层
2. 更真实的车辆动力学模型
3. 感知时序跟踪与多目标数据关联
4. 地图 / 定位 / Route / HD Map
5. OpenSCENARIO / OpenDRIVE 场景导入
6. CARLA 或 AWSIM 后端适配
7. TensorRT / CUDA / ONNX Runtime 推理部署
8. Docker 化部署与车端日志采集
