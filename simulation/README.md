# 仿真目录 (Simulation Directory)

本目录用于存放仿真资源，与 `src/` 源码解耦。当前推荐主线是
**Gazebo Harmonic + ros_gz**；Isaac Sim 相关文件保留为历史验证和备用路径。

## 目录结构

*   **`assets/`**: 可选机器人模型和环境模型。
*   **`config/`**: 仿真特有的参数配置文件。
*   **`scripts/`**: 启动仿真的辅助脚本。

## 快速指南

1.  **Gazebo Harmonic 主线**：启动 Gazebo、四轮全向机器人、RGB-D、IMU、里程计、`ros_gz` bridge，并复用当前 nvblox/Nav2 配置。默认会打开 RViz，并自动循环发送 Nav2 目标点。

    ```bash
    ./simulation/scripts/run_gazebo_harmonic_nav.sh
    ```

2.  **RViz 可视化**：

    ```bash
    RVIZ_FIXED_FRAME=odom ./scripts/run_rviz_nav.sh -d src/omni_bringup/rviz/nvblox_map_check.rviz
    ```

3.  **Isaac 轻量验证**：默认不导入模型、不启动 Isaac Kit，只启动 ROS bridge 发布 pattern RGB-D、TF、里程计和 `/clock`。
    *   终端 1：`./simulation/scripts/run_isaac_sim_nav.sh`
    *   终端 2：`./scripts/with_venv.sh ros2 launch omni_bringup isaac_sim_nav.launch.py`
4.  **Isaac Sim 4.5 UI**：4.5 pip 环境位于 `.venv-isaac-sim-4.5`，运行 `./simulation/scripts/run_isaac_sim_45_ui.sh`。
5.  **真实 Isaac 渲染**：4.5 能稳定启动后，再按 `docs/ISAAC_SIM_SIMULATION.md` 把 `launch_mode` 和 `render_mode` 改成 `isaac`，并配置 USD 地图或机器人模型。
