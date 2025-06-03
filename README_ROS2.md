# PTZ 控制系统 (ROS2 版本)

## 📋 系统概述

这是一个基于 ROS2 的 PTZ (Pan-Tilt-Zoom) 控制系统，提供 Web 界面进行设备控制和图像管理。系统使用 ROS2 话题进行通信，支持实时状态监控和样本命令发布。

## 🚀 主要功能

- **🤖 ROS2 集成**: 使用 ROS2 话题进行 PTZ 设备通信
- **📡 实时监控**: 通过 WebSocket 实时显示 PTZ 状态
- **🎮 样本控制**: 选择预设样本并发布控制命令
- **📸 智能图片管理**: 浏览、搜索和管理 PTZ 拍摄的图片
- **🔍 高级搜索**: 支持文件名搜索和正则表达式匹配
- **📊 多维排序**: 按时间、名称、大小等多种方式排序
- **🔄 智能刷新**: 自动/手动刷新模式，实时更新图片列表
- **🔄 状态同步**: 自动同步设备状态和连接信息

## 🛠️ 系统要求

### 软件依赖
- Ubuntu 22.04 LTS (推荐)
- ROS2 Humble Hawksbill
- Python 3.8+
- Flask 和相关库

### ROS2 话题
- **订阅**: `/ptz_manager/ptz_stable` (Bool/String)
- **发布**: `/sample_face/set_index` (String)

## 📦 安装步骤

### 1. 安装 ROS2 Humble

```bash
# 添加 ROS2 APT 仓库
sudo apt update && sudo apt install curl gnupg2 lsb-release
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo sh -c 'echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'

# 安装 ROS2 Humble
sudo apt update
sudo apt install ros-humble-desktop
sudo apt install python3-rclpy python3-rosdep

# 初始化 rosdep
sudo rosdep init
rosdep update
```

### 2. 设置 ROS2 环境

```bash
# 添加到 ~/.bashrc
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc

# 验证安装
ros2 --version
```

### 3. 安装 Python 依赖

```bash
# 克隆项目
git clone <repository-url>
cd ptz-control-system

# 安装依赖
pip3 install -r requirements_ros2.txt
# 或者手动安装
pip3 install flask flask-socketio rclpy
```

### 4. 配置系统路径

```bash
# 创建必要目录
mkdir -p /home/sunrise/ptz_ws/images

# 设置权限
chmod +x start_ros2_server.sh
```

## 🚀 快速启动

### 1. 使用启动脚本

```bash
./start_ros2_server.sh
```

### 2. 手动启动

```bash
# 设置 ROS2 环境
source /opt/ros/humble/setup.bash

# 启动服务器
python3 app_ros2.py
```

### 3. 访问系统

打开浏览器访问：
- **主页**: http://localhost:5000/
- **图片浏览**: http://localhost:5000/images
- **控制面板**: http://localhost:5000/control

## 🎯 使用指南

### 主页功能
- 查看 PTZ 实时状态
- 监控 ROS2 系统状态
- 快速导航到各功能模块

### 智能图片管理
- **浏览模式**: 支持网格和列表两种视图
- **高级搜索**: 
  - 文件名模糊搜索
  - 正则表达式精确匹配
  - 实时搜索结果高亮显示
- **多维排序**:
  - 按修改时间排序 (默认)
  - 按文件名排序
  - 按文件大小排序
  - 支持升序/降序切换
- **智能刷新**:
  - 自动刷新模式 (30秒间隔)
  - 手动刷新控制
  - 实时倒计时显示
- **统计信息**: 显示总图片数、过滤结果、总大小等
- **图片预览**: 点击查看大图，支持下载

### PTZ 控制
- **动态样本加载**: 从 `~/face_detect/detection_output` 目录自动扫描可用样本
- **智能样本验证**: 自动检查 `results.json` 文件，确保包含 `"type": "face"` 的检测结果
- **样本预览**: 显示每个样本的 `result.jpg` 图片预览
- **实时刷新**: 支持手动刷新样本列表，无需重启服务器
- 通过 ROS2 话题发布控制命令
- 查看命令执行历史
- 实时监控设备状态

### 样本管理系统

#### 样本目录结构
```
~/face_detect/detection_output/
├── sample_1/
│   ├── results.json    # 检测结果文件
│   └── result.jpg      # 结果图片
├── sample_2/
│   ├── results.json
│   └── result.jpg
└── ...
```

#### 样本验证规则
- 目录必须包含 `results.json` 和 `result.jpg` 文件
- `results.json` 必须包含 `"type": "face"` 的检测结果
- 支持以下 JSON 格式:
  ```json
  // 格式1: 直接对象
  {"type": "face", "confidence": 0.95, ...}
  
  // 格式2: 对象数组
  [{"type": "face", "confidence": 0.95, ...}, ...]
  
  // 格式3: 包含results字段
  {"results": [{"type": "face", "confidence": 0.95, ...}]}
  ```

#### 样本管理功能
- **自动扫描**: 启动时自动扫描并加载有效样本
- **实时刷新**: 点击"刷新样本"按钮更新样本列表
- **图片预览**: 在控制界面显示样本的检测结果图片
- **状态反馈**: 显示样本加载状态和错误信息

## 🔧 配置说明

### 系统配置 (app_ros2.py)

```python
# 图片目录
IMAGES_DIR = "/home/sunrise/ptz_ws/images"

# 人脸检测输出目录
FACE_DETECT_DIR = "/home/sunrise/face_detect/detection_output"

# 动态样本选项（从 face_detect 目录加载）
SAMPLE_OPTIONS = get_sample_options()

# ROS2 话题
PTZ_STATUS_TOPIC = "/ptz_manager/ptz_stable"
SAMPLE_COMMAND_TOPIC = "/sample_face/set_index"
```

### ROS2 节点配置

```python
# QoS 配置
qos_profile = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    depth=10
)
```

## 🔍 高级功能

### 图片搜索功能

#### 基础搜索
- 输入文件名的任意部分进行模糊匹配
- 不区分大小写
- 实时搜索，无需按回车

#### 正则表达式搜索
- 启用"正则表达式模式"复选框
- 支持完整的正则表达式语法
- 示例:
  - `^IMG_.*\.jpg$` - 匹配以 IMG_ 开头的 jpg 文件
  - `\d{4}-\d{2}-\d{2}` - 匹配包含日期格式的文件名
  - `(close|medium|wide)_shot` - 匹配特定拍摄类型

### 排序功能

#### 排序选项
- **修改时间**: 按文件最后修改时间排序
- **文件名**: 按文件名字母顺序排序
- **文件大小**: 按文件大小排序

#### 排序方式
- **降序**: 从大到小 (默认)
- **升序**: 从小到大

#### 交互式排序
- 点击列表视图的表头可快速切换排序
- 排序指示器显示当前排序状态

### 自动刷新功能

#### 自动模式
- 每30秒自动刷新图片列表
- 实时倒计时显示
- 绿色指示灯表示自动刷新已启用

#### 手动模式
- 点击"手动刷新"按钮更新
- 灰色指示灯表示自动刷新已关闭

## 🔍 故障排除

### 常见问题

#### 1. ROS2 环境问题
```bash
# 检查 ROS2 环境
echo $ROS_DISTRO
ros2 topic list

# 重新设置环境
source /opt/ros/humble/setup.bash
```

#### 2. Python 模块导入错误
```bash
# 安装缺失的模块
pip3 install rclpy flask flask-socketio

# 检查模块
python3 -c "import rclpy; print('ROS2 Python OK')"
```

#### 3. 话题连接问题
```bash
# 检查话题状态
ros2 topic list
ros2 topic echo /ptz_manager/ptz_stable
ros2 topic info /sample_face/set_index
```

#### 4. ROS2 QoS 兼容性问题

**问题症状**: 出现类似以下错误信息
```
[WARN] New publisher discovered on topic '/sample_face/set_index', offering incompatible QoS. No messages will be received from it. Last incompatible policy: RELIABILITY
```

**解决方案**:

1. **使用 QoS 诊断工具**:
```bash
# 运行 QoS 诊断工具
python3 check_ros2_qos.py
```

2. **检查现有订阅者的 QoS 配置**:
```bash
# 查看话题详细信息
ros2 topic info /sample_face/set_index -v

# 检查发布者和订阅者的 QoS 配置
ros2 topic info /sample_face/set_index --verbose
```

3. **常见 QoS 不兼容情况**:
   - **RELIABILITY**: 发布者使用 `RELIABLE`，订阅者使用 `BEST_EFFORT`
   - **DURABILITY**: 发布者使用 `TRANSIENT_LOCAL`，订阅者使用 `VOLATILE`
   - **HISTORY**: 不同的历史策略设置

4. **系统自动处理**:
   - 系统会首先尝试 `RELIABLE` 策略
   - 如果失败，自动切换到 `BEST_EFFORT` 策略
   - 查看日志确认使用的策略

5. **手动调试步骤**:
```bash
# 1. 停止所有相关节点
ros2 node list
ros2 lifecycle set <node_name> shutdown

# 2. 清理话题
ros2 topic list | grep sample_face

# 3. 重新启动节点
# 先启动订阅者节点，再启动发布者节点

# 4. 验证连接
ros2 topic echo /sample_face/set_index
ros2 topic pub /sample_face/set_index std_msgs/String "data: 'test'"
```

6. **配置建议**:
   - 对于控制命令话题，推荐使用 `RELIABLE` + `VOLATILE`
   - 对于状态监控话题，可以使用 `BEST_EFFORT` + `VOLATILE`
   - 确保发布者和订阅者使用兼容的 QoS 配置

#### 5. 端口占用
```bash
# 检查端口使用
lsof -i:5000

# 终止占用进程
sudo kill -9 <PID>
```

#### 6. 图片搜索问题
- **正则表达式错误**: 检查正则表达式语法
- **搜索无结果**: 确认文件名拼写和大小写
- **搜索缓慢**: 大量图片时建议使用更精确的搜索条件

### 日志查看

系统日志会显示在终端中，包括：
- ROS2 节点连接状态
- 话题订阅和发布信息
- WebSocket 连接状态
- 搜索和排序操作
- 错误和警告信息

## 🔄 系统架构

```
┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│   Web Browser   │◄──►│  Flask Server   │◄──►│   ROS2 Node     │
│                 │    │  (app_ros2.py)  │    │ (PTZControlNode)│
└─────────────────┘    └─────────────────┘    └─────────────────┘
                                │                       │
                                ▼                       ▼
                        ┌─────────────────┐    ┌─────────────────┐
                        │   WebSocket     │    │   ROS2 Topics   │
                        │   (Real-time)   │    │   Communication │
                        └─────────────────┘    └─────────────────┘
```

## 📝 API 文档

### REST API 端点

- `GET /` - 主页
- `GET /images` - 图片浏览页面
- `GET /control` - 控制面板
- `GET /api/images` - 获取图片列表
- `GET /api/samples` - 获取样本列表
- `GET /api/refresh_samples` - 刷新样本列表
- `POST /api/execute_script` - 执行样本命令
- `GET /api/ptz_status` - 获取 PTZ 状态
- `GET /api/ros_status` - 获取 ROS 连接状态
- `GET /sample_images/<sample_name>` - 获取样本图片

### WebSocket 事件

- `ptz_status_update` - PTZ 状态更新
- `ros_status_update` - ROS 连接状态更新

## ⌨️ 键盘快捷键

### 图片浏览页面
- `Ctrl + F` - 聚焦搜索框
- `Ctrl + V` - 切换视图模式
- `F5` / `Ctrl + R` - 刷新图片列表
- `Esc` - 取消搜索框焦点

### 控制页面
- `Ctrl + Enter` - 执行选中的样本命令
- `F5` - 刷新状态信息

## 🤝 贡献指南

1. Fork 项目
2. 创建功能分支 (`git checkout -b feature/AmazingFeature`)
3. 提交更改 (`git commit -m 'Add some AmazingFeature'`)
4. 推送到分支 (`git push origin feature/AmazingFeature`)
5. 创建 Pull Request

## 📄 许可证

本项目采用 MIT 许可证 - 查看 [LICENSE](LICENSE) 文件了解详情。

## 📞 支持

如有问题或建议，请：
1. 查看故障排除部分
2. 提交 Issue
3. 联系开发团队

---

**注意**: 确保 ROS2 环境正确配置，并且相关的 PTZ 控制节点正在运行，以保证系统正常工作。 