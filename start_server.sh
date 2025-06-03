#!/bin/bash

# PTZ控制系统启动脚本

echo "正在启动PTZ控制系统..."

# 检查ROS环境
if [ -f "/opt/tros/humble/setup.bash" ]; then
    echo "找到ROS环境配置文件"
    source /opt/tros/humble/setup.bash
else
    echo "警告: 未找到ROS环境配置文件 /opt/tros/humble/setup.bash"
fi

# 检查Python依赖
echo "检查Python依赖..."
python3 -c "import flask, flask_socketio" 2>/dev/null
if [ $? -ne 0 ]; then
    echo "安装Python依赖..."
    pip install -r requirements.txt
fi

# 检查图片目录
if [ ! -d "/home/sunrise/ptz_ws/images" ]; then
    echo "警告: 图片目录 /home/sunrise/ptz_ws/images 不存在"
fi

# 检查脚本文件
if [ ! -f "/home/sunrise/ptz_ws/pub_sample.sh" ]; then
    echo "警告: 脚本文件 /home/sunrise/ptz_ws/pub_sample.sh 不存在"
fi

# 启动服务器
echo "启动Flask服务器..."
echo "访问地址:"
echo "  - 主页: http://$(hostname -I | awk '{print $1}'):5000/"
echo "  - 图片浏览: http://$(hostname -I | awk '{print $1}'):5000/images"
echo "  - 控制面板: http://$(hostname -I | awk '{print $1}'):5000/control"
echo ""
echo "按 Ctrl+C 停止服务器"
echo ""

python3 app.py 