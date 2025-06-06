#!/bin/bash

# PTZ控制系统启动脚本 (ROS2版本)
echo "================================================"
echo "     PTZ 控制系统 (ROS2版本) 启动脚本"
echo "================================================"

# 设置颜色输出
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# 检查ROS2环境
echo -e "${BLUE}检查 ROS2 环境...${NC}"
if [ -f "/opt/tros/humble/setup.bash" ]; then
    echo -e "${GREEN}✓ 找到 ROS2 环境设置文件${NC}"
    source /opt/tros/humble/setup.bash
    echo -e "${GREEN}✓ ROS2 环境已加载${NC}"
    echo "  - ROS_DISTRO: $ROS_DISTRO"
    echo "  - ROS_VERSION: $ROS_VERSION"
    echo "  - ROS_DOMAIN_ID: ${ROS_DOMAIN_ID:-0}"
else
    echo -e "${YELLOW}⚠ 未找到 ROS2 环境设置文件 /opt/tros/humble/setup.bash${NC}"
    echo -e "${YELLOW}  尝试查找其他 ROS2 安装路径...${NC}"
    
    # 尝试其他常见的ROS2路径
    if [ -f "/opt/ros/humble/setup.bash" ]; then
        echo -e "${GREEN}✓ 找到标准 ROS2 Humble 安装${NC}"
        source /opt/ros/humble/setup.bash
    elif [ -f "/opt/ros/galactic/setup.bash" ]; then
        echo -e "${GREEN}✓ 找到 ROS2 Galactic 安装${NC}"
        source /opt/ros/galactic/setup.bash
    else
        echo -e "${RED}✗ 未找到 ROS2 安装${NC}"
        echo -e "${RED}  请确保已正确安装 ROS2${NC}"
    fi
fi

# 检查并加载PTZ接口环境
echo -e "\n${BLUE}检查 PTZ 接口环境...${NC}"
PTZ_SETUP_PATH="/home/sunrise/ptz_ws/install/setup.bash"
if [ -f "$PTZ_SETUP_PATH" ]; then
    echo -e "${GREEN}✓ 找到 PTZ 接口环境设置文件${NC}"
    source "$PTZ_SETUP_PATH"
    echo -e "${GREEN}✓ PTZ 接口环境已加载${NC}"
    
    # 验证PTZ接口模块
    python3 -c "
try:
    from ptz_interfaces.srv import SwitchTrackingTarget
    print('  ✓ ptz_interfaces.srv.SwitchTrackingTarget 可用')
except ImportError as e:
    print(f'  ✗ PTZ接口导入失败: {e}')
    exit(1)
" 2>/dev/null
    
    if [ $? -eq 0 ]; then
        echo -e "${GREEN}✓ PTZ 接口模块验证成功${NC}"
    else
        echo -e "${RED}✗ PTZ 接口模块验证失败${NC}"
        echo -e "${YELLOW}  追踪目标切换功能将不可用${NC}"
    fi
else
    echo -e "${YELLOW}⚠ 未找到 PTZ 接口环境设置文件: $PTZ_SETUP_PATH${NC}"
    echo -e "${YELLOW}  请确保已编译 PTZ 接口包${NC}"
    echo -e "${YELLOW}  追踪目标切换功能将不可用${NC}"
fi

# 检查Python依赖
echo -e "\n${BLUE}检查 Python 依赖...${NC}"

# 检查基础依赖
MISSING_DEPS=()

python3 -c "import flask" 2>/dev/null || MISSING_DEPS+=("flask")
python3 -c "import flask_socketio" 2>/dev/null || MISSING_DEPS+=("flask-socketio")

if [ ${#MISSING_DEPS[@]} -eq 0 ]; then
    echo -e "${GREEN}✓ Flask 依赖已安装${NC}"
else
    echo -e "${YELLOW}⚠ 缺少依赖: ${MISSING_DEPS[*]}${NC}"
    echo -e "${YELLOW}  正在安装依赖...${NC}"
    
    if [ -f "requirements_ros2.txt" ]; then
        echo -e "${BLUE}  使用 requirements_ros2.txt 安装依赖...${NC}"
        pip3 install -r requirements_ros2.txt
    else
        echo -e "${BLUE}  手动安装基础依赖...${NC}"
        pip3 install flask flask-socketio
    fi
fi

# 检查ROS2 Python模块

# 检查可选依赖
echo -e "\n${BLUE}检查可选依赖...${NC}"
python3 -c "import PIL" 2>/dev/null
if [ $? -eq 0 ]; then
    echo -e "${GREEN}✓ Pillow (图片处理) 已安装${NC}"
else
    echo -e "${YELLOW}⚠ Pillow 未安装 (可选，用于图片处理)${NC}"
fi

# 检查必要目录
echo -e "\n${BLUE}检查系统配置...${NC}"
IMAGES_DIR="/home/sunrise/ptz_ws/images"
SCRIPT_PATH="/home/sunrise/ptz_ws/pub_sample.sh"
FACE_DETECT_DIR="/home/sunrise/face_detect/detection_output"

if [ -d "$IMAGES_DIR" ]; then
    echo -e "${GREEN}✓ 图片目录存在: $IMAGES_DIR${NC}"
    IMAGE_COUNT=$(find "$IMAGES_DIR" -name "*.jpg" -o -name "*.jpeg" -o -name "*.png" -o -name "*.gif" 2>/dev/null | wc -l)
    echo "  - 当前图片数量: $IMAGE_COUNT"
else
    echo -e "${YELLOW}⚠ 图片目录不存在: $IMAGES_DIR${NC}"
    echo "  - 将在服务器启动时创建"
    mkdir -p "$IMAGES_DIR" 2>/dev/null && echo -e "${GREEN}  ✓ 已创建图片目录${NC}"
fi

if [ -d "$FACE_DETECT_DIR" ]; then
    echo -e "${GREEN}✓ 人脸检测输出目录存在: $FACE_DETECT_DIR${NC}"
    SAMPLE_COUNT=$(find "$FACE_DETECT_DIR" -maxdepth 1 -type d | wc -l)
    SAMPLE_COUNT=$((SAMPLE_COUNT - 1))  # 减去父目录本身
    echo "  - 发现样本目录数量: $SAMPLE_COUNT"
    
    # 检查有效样本数量
    VALID_SAMPLES=0
    for sample_dir in "$FACE_DETECT_DIR"/*; do
        if [ -d "$sample_dir" ] && [ -f "$sample_dir/results.json" ] && [ -f "$sample_dir/result.jpg" ]; then
            VALID_SAMPLES=$((VALID_SAMPLES + 1))
        fi
    done
    echo "  - 有效样本数量: $VALID_SAMPLES (包含 results.json 和 result.jpg)"
else
    echo -e "${YELLOW}⚠ 人脸检测输出目录不存在: $FACE_DETECT_DIR${NC}"
    echo "  - 样本控制功能可能不可用"
    echo "  - 请确保人脸检测系统已运行并生成输出"
fi

if [ -f "$SCRIPT_PATH" ]; then
    echo -e "${GREEN}✓ 脚本文件存在: $SCRIPT_PATH${NC}"
else
    echo -e "${YELLOW}⚠ 脚本文件不存在: $SCRIPT_PATH${NC}"
    echo "  - 将使用 ROS2 话题进行通信"
fi

# 检查网络端口
echo -e "\n${BLUE}检查网络端口...${NC}"
if command -v lsof >/dev/null 2>&1; then
    if lsof -i:5000 >/dev/null 2>&1; then
        echo -e "${YELLOW}⚠ 端口 5000 已被占用${NC}"
        echo "  正在使用端口 5000 的进程:"
        lsof -i:5000
        echo -e "${YELLOW}  请手动停止占用进程或更改端口${NC}"
    else
        echo -e "${GREEN}✓ 端口 5000 可用${NC}"
    fi
else
    echo -e "${YELLOW}⚠ lsof 命令不可用，无法检查端口状态${NC}"
fi

# 显示启动信息
echo -e "\n${BLUE}================================================${NC}"
echo -e "${GREEN}启动 PTZ 控制系统 (ROS2版本)...${NC}"
echo -e "${BLUE}================================================${NC}"
echo "访问地址:"
echo "  🏠 主页:       http://localhost:5000/"
echo "  📷 图片浏览:   http://localhost:5000/images"
echo "  🎮 控制面板:   http://localhost:5000/control"
echo ""
echo "新增功能:"
echo "  🔍 图片搜索 (支持正则表达式)"
echo "  📊 多种排序方式 (时间/名称/大小)"
echo "  🔄 自动/手动刷新模式"
echo "  📡 实时状态监控"
echo ""
echo "系统特性:"
echo "  🤖 ROS2 话题通信"
echo "  📡 实时状态监控"
echo "  🔄 WebSocket 实时更新"
echo "  📸 智能图片管理"
echo ""
echo -e "${YELLOW}提示: 使用 Ctrl+C 停止服务器${NC}"
echo -e "${BLUE}================================================${NC}"

# 启动服务器
python3 app_ros2.py 