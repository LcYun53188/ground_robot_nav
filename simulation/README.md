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

    可通过 `arena:=rmuc_2024`、`rmuc_2025`、`rmul_2024` 或 `rmul_2025`
    选择项目内置 RoboMaster 场地。所有 Gazebo 场景同时发布
    `/mid360/points` 和 `/mid360/imu`，不依赖外部仿真仓库。

    RMUC 2025、全向底盘和仿真 OAK-D/MID360 的专用入口：

    ```bash
    ./simulation/scripts/run_rmuc_2025_sim.sh
    ```

    默认启动 Nav2 和 RViz，但不自动发送巡航目标；可在 RViz 中手动设置目标点。
    如需恢复预设点自动巡航：

    ```bash
    ./simulation/scripts/run_rmuc_2025_sim.sh launch_auto_goals:=true
    ```

    需要手动验证底盘运动或爬坡时，建议关闭 Nav2，避免导航速度与手动速度同时发布：

    ```bash
    ./simulation/scripts/run_rmuc_2025_sim.sh launch_navigation:=false
    ```

    然后在另一个终端运行键盘遥控窗口。`W/S` 控制前后，`A/D` 控制左右
    平移，`Q/E` 控制左右旋转；可同时按下多个运动键，数字 `1` 到 `9`
    选择 10% 到 90% 速度，`0` 选择 100% 速度：

    ```bash
    ./simulation/scripts/run_keyboard_teleop.sh
    ```

    该脚本会显式设置 `launch_gazebo:=true` 和 `launch_rviz:=true`，同时打开一个
    Gazebo 窗口和 RViz，并启动传感器桥接、nvblox 和 Nav2，同时关闭自动目标点，
    由操作者在 RViz 中手动下发导航目标。仅检查传感器、不启动导航栈时运行：

    ```bash
    ./simulation/scripts/run_rmuc_2025_sim.sh launch_navigation:=false
    ```

    OAK-D 默认开启，MID360 默认关闭；两者均可通过开关独立控制其 ROS bridge、
    图像 bridge 和静态 TF：

    ```bash
    ./simulation/scripts/run_rmuc_2025_sim.sh launch_oakd:=false
    ./simulation/scripts/run_rmuc_2025_sim.sh launch_mid360:=true
    ```

    专用入口默认加载 `gazebo_sensor_check.rviz`，显示 nvblox 三维网格、静态占据地图、
    Nav2 局部/全局代价地图、全局/局部路径、机器人轮廓，以及仿真里程计、MID360
    点云和 OAK-D 图像。彩虹色 ESDF 切片默认关闭，避免将距离着色误认为障碍物；
    需要时可在 `Nvblox 3D` 组中手动启用。点击 RViz 顶部工具栏的 `2D Goal Pose`，
    在地图中按住并拖动设置位置与朝向，即可直接发送手动导航目标。也可使用
    `Nav2 Goal` 选点，再在 `Navigation 2` 面板中启动或取消任务。

    OAK-D 深度显示固定使用 `0.2–8.0 m` 灰度范围，不使用 RViz 自动归一化。
    Gazebo 对量程外像素发布 `+Inf`；固定范围可避免这些像素干扰整帧最大值计算，
    导致深度窗口黑白交替或闪烁。nvblox 订阅
    `/rgbd_camera/depth_obstacles`：该话题从原始 `32FC1` 深度中移除低矮可通行
    平面和坡面，但保留墙体、较高障碍及无效深度边界。

    每次运行时，脚本会先查找本工作区旧的 Gazebo/ROS launch 进程，依次使用
    `SIGINT` 和超时后的 `SIGTERM` 完成清理；确认旧进程及仿真锁释放后才启动新
    实例，并在终端打印清理结果。

    底盘使用四轮 `MecanumDrive` 全向运动模型，支持 `/cmd_vel` 的前后、横移和旋转
    速度，并使用 DART 物理后端保留麦轮所需的各向异性接触摩擦。不要将启动参数改为
    Bullet Featherstone，否则横移和旋转会退化为沿车体 X 轴运动。启动入口默认只允许
    一个 Gazebo 仿真实例，避免多个实例同时发布 `/tf` 和里程计而导致机器人显示跳动。

2.  **RViz 可视化**：

    ```bash
    RVIZ_FIXED_FRAME=odom ./scripts/run_rviz_nav.sh -d src/omni_bringup/rviz/nvblox_map_check.rviz
    ```

3.  **Isaac 轻量验证**：默认不导入模型、不启动 Isaac Kit，只启动 ROS bridge 发布 pattern RGB-D、TF、里程计和 `/clock`。
    *   终端 1：`./simulation/scripts/run_isaac_sim_nav.sh`
    *   终端 2：`./scripts/with_venv.sh ros2 launch omni_bringup isaac_sim_nav.launch.py`
4.  **Isaac Sim 4.5 UI**：4.5 pip 环境位于 `.venv-isaac-sim-4.5`，运行 `./simulation/scripts/run_isaac_sim_45_ui.sh`。
5.  **真实 Isaac 渲染**：4.5 能稳定启动后，再按 `docs/ISAAC_SIM_SIMULATION.md` 把 `launch_mode` 和 `render_mode` 改成 `isaac`，并配置 USD 地图或机器人模型。
