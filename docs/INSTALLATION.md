# 安装与构建指南

本文档详细说明虚拟环境配置、依赖安装与包构建步骤。项目约定所有 Python 操作在虚拟环境 `.venv` 内进行。

---

## 1. 虚拟环境初始化

本项目使用 `uv` 作为包管理工具。`uv` 速度快且依赖解析精准。

### 1.1 安装 uv

如尚未安装 `uv`，请执行：

```bash
curl -LsSf https://astral.sh/uv/install.sh | sh
```

### 1.2 创建虚拟环境

在项目根目录创建 `.venv`：

```bash
uv venv .venv
```

### 1.3 安装项目依赖

#### 安装 oakd_perception 包

```bash
uv pip install --python .venv/bin/python -e src/oakd_perception
```

`-e` 标志表示可编辑模式，便于开发时修改源码生效。

#### 安装 OAK-D SDK

```bash
uv pip install --python .venv/bin/python depthai
```

### 1.4 验证安装

执行以下命令验证 depthai 已正确安装：

```bash
./scripts/with_venv.sh python -c "import depthai; print(depthai.__version__)"
```

如输出版本号，则安装成功。

---

## 2. 激活/停用虚拟环境

### 2.1 激活虚拟环境

```bash
source .venv/bin/activate
```

激活后，终端提示符前会出现 `(.venv)` 标志。所有后续 `pip` 与 `python` 命令会自动使用该环境。

### 2.2 停用虚拟环境

```bash
deactivate
```

或直接关闭终端。

### 2.3 在虚拟环境中安装额外包

激活后可直接使用 `pip`（无需指定 Python 路径）：

```bash
source .venv/bin/activate
pip install <package_name>
```

或不激活，通过 `uv` 指定路径：

```bash
uv pip install --python .venv/bin/python <package_name>
```

---

## 3. 查看与管理已安装包

### 3.1 列出虚拟环境中的包

```bash
uv pip list --python .venv/bin/python
```

### 3.2 删除虚拟环境（清理）

若需重建虚拟环境，先删除旧环境：

```bash
rm -rf .venv
```

然后按 1.2–1.3 重新创建。

---

## 4. VS Code 配置

### 4.1 选择解释器

工作区已预配置默认解释器为 `.venv/bin/python`。

确认方式：

1. 打开命令面板：`Ctrl+Shift+P`
2. 搜索 `Python: Select Interpreter`
3. 确认指向 `./.venv/bin/python`

### 4.2 自动激活

VS Code 新开终端时会自动使用 `.venv` 中的 Python。如不自动激活，可在终端添加 `.bashrc` 或 `.zshrc` 初始化脚本。

---

## 5. 使用 `with_venv.sh` 便捷执行

`scripts/with_venv.sh` 是一个包裹脚本，可在虚拟环境中直接执行任意命令，无需手动激活。

### 5.1 用法

```bash
./scripts/with_venv.sh <command>
```

### 5.2 示例

```bash
# 验证 depthai 版本
./scripts/with_venv.sh python -c "import depthai; print(depthai.__version__)"

# 列出已安装包
./scripts/with_venv.sh pip list

# 运行 ROS 2 命令
./scripts/with_venv.sh ros2 topic list
```

---

## 6. 构建项目

如需构建 NVIDIA Isaac ROS / nvblox 相关包，先按
[CUDA Toolkit 13.2 安装指南](./CUDA_TOOLKIT_13_2_INSTALLATION.md) 安装 `nvcc`。

### 6.1 使用 colcon 构建

在虚拟环境中使用 `colcon` 构建包：

```bash
./scripts/with_venv.sh colcon build --packages-select oakd_perception imu_fusion
```

### 6.2 构建选项

- `--packages-select oakd_perception imu_fusion` — 仅构建指定包
- `--parallel <N>` — 并行构建（加速）
- `--symlink-install` — 符号链接安装（开发模式，加速重建）

完整示例：

```bash
./scripts/with_venv.sh colcon build --packages-select oakd_perception imu_fusion --parallel 4 --symlink-install
```

### 6.3 构建输出

构建成功后，编译输出位于：

- `build/` — 中间文件（可删除以清空缓存）
- `install/` — 最终安装包
- `log/` — 构建日志

---

## 7. 故障排查

### 7.1 ModuleNotFoundError: No module named 'depthai'

**症状**：运行节点时提示找不到 depthai 模块。

**排查**：

```bash
./scripts/with_venv.sh python -c "import depthai; print('OK')"
```

若输出 `OK` 则正常；否则需重新安装（见 1.3）。

### 7.2 权限被拒绝

**症状**：运行脚本时提示 `Permission denied`。

**解决**：

```bash
chmod +x scripts/*.sh
```

### 7.3 UnicodeDecodeError 或其他编码问题

设置环境变量：

```bash
export LC_ALL=C.UTF-8
export LANG=C.UTF-8
```

然后重试。

---

## 8. 参考

- [uv 官方文档](https://docs.astral.sh/uv/)
- [ROS 2 安装指南](https://docs.ros.org/en/humble/Installation.html)
- [colcon 用户手册](https://colcon.readthedocs.io/)

---

如有问题或疑问，请查阅 [README.md](../README.md) 或 [CONTRIBUTING.md](./CONTRIBUTING.md)。
