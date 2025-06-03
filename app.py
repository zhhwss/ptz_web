#!/usr/bin/env python3
import os
import subprocess
import threading
import time
from datetime import datetime
from flask import Flask, render_template, jsonify, request, send_from_directory
from flask_socketio import SocketIO, emit
import json

app = Flask(__name__)
app.config['SECRET_KEY'] = 'your-secret-key'
socketio = SocketIO(app, cors_allowed_origins="*")

# 配置路径
IMAGES_DIR = "/home/sunrise/ptz_ws/images"
SCRIPT_PATH = "/home/sunrise/ptz_ws/pub_sample.sh"

# 存储当前选择的样本
current_sample = "Long_Shot_Standing_Back_Turned_Looking_Back_Hands_Behind_Back"

# 可用的样本选项（从脚本中提取）
SAMPLE_OPTIONS = [
    "Close-Up_Standing_Back_Turned_Looking_Back_Both_Hands_Raised",
    "Long_Shot_Standing_Back_Turned_Looking_Back_Hands_Behind_Back",
    "Medium_Close-Up_Standing_Front_Facing_Head_Tilted_Up_Hands_in_Pockets",
    "Medium_Close-Up_Standing_Front_Facing_Turning_Head_Up_One_Hand_Raised",
    "Medium_Shot_Sitting_Side_View_Head_Tilted_Up_One_Leg_Bent_One_Hand_Resting",
    "Medium_Shot_Standing_Front_Facing_Head_Turned_Legs_Apart_One_Hand_Raised_High_One_Low",
    "Wide_Shot_Leaning_Front_Facing_Turning_Head_Legs_Crossed_One_Hand_Leaning",
    "Wide_Shot_Standing_Head_Tilted_Sideways_Right_Leg_Supporting_Arms_Extended"
]

# 存储ROS话题状态
ptz_status = {"status": "未连接", "timestamp": ""}

def get_images():
    """获取图片目录中的所有图片文件"""
    try:
        if not os.path.exists(IMAGES_DIR):
            return []
        
        files = []
        for filename in os.listdir(IMAGES_DIR):
            if filename.lower().endswith(('.jpg', '.jpeg', '.png', '.gif')):
                filepath = os.path.join(IMAGES_DIR, filename)
                stat = os.stat(filepath)
                files.append({
                    'name': filename,
                    'size': stat.st_size,
                    'modified': datetime.fromtimestamp(stat.st_mtime).strftime('%Y-%m-%d %H:%M:%S')
                })
        
        # 按修改时间排序，最新的在前
        files.sort(key=lambda x: x['modified'], reverse=True)
        return files
    except Exception as e:
        print(f"Error getting images: {e}")
        return []

def monitor_ros_topic():
    """监听ROS话题的线程函数"""
    while True:
        try:
            # 设置ROS2环境变量
            env = os.environ.copy()
            
            # 添加ROS2环境变量
            setup_script = "/opt/tros/humble/setup.bash"
            if os.path.exists(setup_script):
                # 通过source命令获取环境变量
                source_cmd = f"source {setup_script} && env"
                result = subprocess.run(source_cmd, shell=True, capture_output=True, text=True, timeout=10)
                
                if result.returncode == 0:
                    # 解析环境变量
                    for line in result.stdout.split('\n'):
                        if '=' in line and not line.startswith('_'):
                            key, value = line.split('=', 1)
                            env[key] = value
            
            # 使用ros2 topic echo监听话题，设置较短的超时时间
            cmd = ["ros2", "topic", "echo", "/ptz_manager/ptz_stable", "--once"]
            result = subprocess.run(cmd, 
                                  capture_output=True, 
                                  text=True, 
                                  timeout=8,  # 减少超时时间
                                  env=env)
            
            if result.returncode == 0 and result.stdout.strip():
                # 解析ROS消息
                status_data = result.stdout.strip()
                timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
                
                global ptz_status
                ptz_status = {
                    "status": status_data,
                    "timestamp": timestamp
                }
                
                # 通过WebSocket发送更新
                socketio.emit('ptz_status_update', ptz_status)
                print(f"PTZ状态更新: {status_data}")
            else:
                # 如果没有收到消息，更新状态
                ptz_status = {
                    "status": "等待数据...",
                    "timestamp": datetime.now().strftime('%Y-%m-%d %H:%M:%S')
                }
                socketio.emit('ptz_status_update', ptz_status)
                
        except subprocess.TimeoutExpired:
            print("ROS话题监听超时")
            ptz_status = {
                "status": "监听超时",
                "timestamp": datetime.now().strftime('%Y-%m-%d %H:%M:%S')
            }
            socketio.emit('ptz_status_update', ptz_status)
        except Exception as e:
            print(f"Error monitoring ROS topic: {e}")
            ptz_status = {
                "status": f"错误: {str(e)}",
                "timestamp": datetime.now().strftime('%Y-%m-%d %H:%M:%S')
            }
            socketio.emit('ptz_status_update', ptz_status)
        
        time.sleep(3)  # 增加检查间隔，减少系统负载

# 启动ROS话题监听线程
ros_thread = threading.Thread(target=monitor_ros_topic, daemon=True)
ros_thread.start()

@app.route('/')
def index():
    """主页 - 重定向到控制页面"""
    return render_template('index.html')

@app.route('/images')
def images_page():
    """图片浏览页面"""
    return render_template('images.html')

@app.route('/control')
def control_page():
    """控制页面"""
    return render_template('control.html', 
                         sample_options=SAMPLE_OPTIONS, 
                         current_sample=current_sample)

@app.route('/api/images')
def api_images():
    """获取图片列表的API"""
    images = get_images()
    return jsonify(images)

@app.route('/api/execute_script', methods=['POST'])
def execute_script():
    """执行脚本的API"""
    try:
        global current_sample
        
        data = request.get_json()
        selected_sample = data.get('sample', current_sample)
        
        current_sample = selected_sample
        
        # 设置ROS2环境变量
        env = os.environ.copy()
        
        # 添加ROS2环境变量
        setup_script = "/opt/tros/humble/setup.bash"
        if os.path.exists(setup_script):
            # 通过source命令获取环境变量
            source_cmd = f"source {setup_script} && env"
            env_result = subprocess.run(source_cmd, shell=True, capture_output=True, text=True, timeout=10)
            
            if env_result.returncode == 0:
                # 解析环境变量
                for line in env_result.stdout.split('\n'):
                    if '=' in line and not line.startswith('_'):
                        key, value = line.split('=', 1)
                        env[key] = value
        
        # 创建临时脚本文件
        temp_script = f"""#!/bin/bash
source /opt/tros/humble/setup.bash
ros2 topic pub --once /sample_face/set_index std_msgs/String '{{data: "{selected_sample}"}}'
"""
        
        temp_script_path = "/tmp/temp_pub_sample.sh"
        with open(temp_script_path, 'w') as f:
            f.write(temp_script)
        
        os.chmod(temp_script_path, 0o755)
        
        # 执行脚本，使用正确的环境变量和较短的超时时间
        result = subprocess.run([temp_script_path], 
                              capture_output=True, 
                              text=True, 
                              timeout=15,  # 减少超时时间
                              env=env)
        
        # 清理临时文件
        if os.path.exists(temp_script_path):
            os.remove(temp_script_path)
        
        if result.returncode == 0:
            return jsonify({
                'success': True, 
                'message': f'成功执行样本: {selected_sample}',
                'output': result.stdout
            })
        else:
            return jsonify({
                'success': False, 
                'message': f'执行失败: {result.stderr}',
                'output': result.stderr
            })
            
    except subprocess.TimeoutExpired:
        # 清理临时文件
        if 'temp_script_path' in locals() and os.path.exists(temp_script_path):
            os.remove(temp_script_path)
        return jsonify({
            'success': False, 
            'message': '脚本执行超时'
        })
    except Exception as e:
        # 清理临时文件
        if 'temp_script_path' in locals() and os.path.exists(temp_script_path):
            os.remove(temp_script_path)
        return jsonify({
            'success': False, 
            'message': f'执行错误: {str(e)}'
        })

@app.route('/api/ptz_status')
def api_ptz_status():
    """获取PTZ状态的API"""
    return jsonify(ptz_status)

@app.route('/images/<filename>')
def serve_image(filename):
    """提供图片文件"""
    return send_from_directory(IMAGES_DIR, filename)

@socketio.on('connect')
def handle_connect():
    """WebSocket连接处理"""
    print('Client connected')
    emit('ptz_status_update', ptz_status)

@socketio.on('disconnect')
def handle_disconnect():
    """WebSocket断开处理"""
    print('Client disconnected')

if __name__ == '__main__':
    # 确保模板目录存在
    os.makedirs('templates', exist_ok=True)
    os.makedirs('static', exist_ok=True)
    
    print("启动网页服务器...")
    print(f"图片目录: {IMAGES_DIR}")
    print(f"脚本路径: {SCRIPT_PATH}")
    print("访问地址:")
    print("  - 主页: http://0.0.0.0:5000/")
    print("  - 图片浏览: http://0.0.0.0:5000/images")
    print("  - 控制页面: http://0.0.0.0:5000/control")
    
    socketio.run(app, host='0.0.0.0', port=5000, debug=True, allow_unsafe_werkzeug=True) 