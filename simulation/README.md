# 仿真目录 (Simulation Directory)

本目录用于存放与 **NVIDIA Isaac Sim** 相关的仿真资源，与 `src/` 源码解耦。

## 📁 目录结构

*   **`assets/`**: 存放机器人模型 (USD/URDF) 和环境模型 (STL/OBJ 转换后的 USD)。
*   **`worlds/`**: 存放搭建好的仿真场景文件 (.usd)，例如 `factory_with_slopes.usd`。
*   **`config/`**: 存放仿真特有的参数配置文件（如 Isaac Sim Bridge 的 OmniGraph 配置导出）。
*   **`scripts/`**: 存放启动仿真的辅助脚本，例如自动加载模型并启动 ROS 2 Bridge 的 Python 脚本。

## 🚀 快速指南

1.  **导入模型**：将 STP/STL 转换后的 USD 文件放入 `assets/`。
2.  **搭建场景**：在 Isaac Sim 中使用 `assets/` 中的模型搭建包含“钻洞”和“爬坡”特征的环境，并保存到 `worlds/`。
3.  **运行仿真**：
    *   按 `docs/ISAAC_SIM_SIMULATION.md` 配置 Isaac Sim 路径。
    *   终端 1：`./simulation/scripts/run_isaac_sim_nav.sh`
    *   终端 2：`./scripts/with_venv.sh ros2 launch omni_bringup isaac_sim_nav.launch.py`
