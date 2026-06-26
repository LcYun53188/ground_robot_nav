# 仿真目录 (Simulation Directory)

本目录用于存放与 **NVIDIA Isaac Sim** 相关的仿真资源，与 `src/` 源码解耦。

## 目录结构

*   **`assets/`**: 可选机器人模型 (USD/URDF) 和环境模型 (STL/OBJ 转换后的 USD)。
*   **`worlds/`**: 可选仿真场景文件 (.usd)，例如 `factory_with_slopes.usd`。
*   **`config/`**: 仿真特有的参数配置文件。
*   **`scripts/`**: 启动仿真的辅助脚本。

## 快速指南

1.  **最轻量验证**：默认不导入模型、不启动 Isaac Kit，只启动 ROS bridge 发布 pattern RGB-D、TF、里程计和 `/clock`。
2.  **运行仿真**：
    *   终端 1：`./simulation/scripts/run_isaac_sim_nav.sh`
    *   终端 2：`./scripts/with_venv.sh ros2 launch omni_bringup isaac_sim_nav.launch.py`
    *   可视化：`RVIZ_FIXED_FRAME=odom ./scripts/run_rviz_nav.sh -d src/omni_bringup/rviz/nvblox_map_check.rviz`
3.  **Isaac Sim 4.5 UI**：4.5 pip 环境位于 `.venv-isaac-sim-4.5`，运行 `./simulation/scripts/run_isaac_sim_45_ui.sh`。
4.  **真实 Isaac 渲染**：4.5 能稳定启动后，再按 `docs/ISAAC_SIM_SIMULATION.md` 把 `launch_mode` 和 `render_mode` 改成 `isaac`，并配置 USD 地图或机器人模型。
