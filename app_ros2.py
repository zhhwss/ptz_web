#!/usr/bin/env python3
import os
import threading
import time
import subprocess
import re
import logging
from datetime import datetime
from flask import Flask, render_template, jsonify, request, send_from_directory
from flask_socketio import SocketIO, emit
import json
import signal
import sys

# ROS2 imports
try:
    import rclpy
    from rclpy.node import Node
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
    from std_msgs.msg import String
    from std_msgs.msg import Bool  # 假设ptz_stable是Bool类型，根据实际情况调整
    from ai_msgs.msg import PerceptionTargets  # 导入人脸检测消息类型
    ROS2_AVAILABLE = True
except ImportError as e:
    print(f"ROS2 Python模块导入失败: {e}")
    print("请确保已正确安装ROS2并source了setup.bash")
    ROS2_AVAILABLE = False

# 配置日志
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

app = Flask(__name__)
app.config['SECRET_KEY'] = 'your-secret-key'
socketio = SocketIO(app, cors_allowed_origins="*", async_mode='threading')

# 配置路径
IMAGES_DIR = "/home/sunrise/ptz_ws/images"
SCRIPT_PATH = "/home/sunrise/ptz_ws/pub_sample.sh"
FACE_DETECT_DIR = "/home/sunrise/face_detect/detection_output"

# 存储当前选择的样本
current_sample = ""

# 存储当前PTZ模式，默认为模式2
current_ptz_mode = 2

# 动态获取样本选项
def get_sample_options():
    """从 face_detect/detection_output 目录动态获取有效的样本选项"""
    sample_options = []
    
    try:
        if not os.path.exists(FACE_DETECT_DIR):
            print(f"警告: 样本目录不存在: {FACE_DETECT_DIR}")
            return []
        
        # 遍历检测输出目录
        for item in os.listdir(FACE_DETECT_DIR):
            item_path = os.path.join(FACE_DETECT_DIR, item)
            
            # 检查是否为目录
            if not os.path.isdir(item_path):
                continue
            
            # 检查是否存在 results.json
            results_json_path = os.path.join(item_path, "results.json")
            if not os.path.exists(results_json_path):
                print(f"跳过 {item}: 缺少 results.json")
                continue
            
            # 检查是否存在 result.jpg
            result_jpg_path = os.path.join(item_path, "result.jpg")
            if not os.path.exists(result_jpg_path):
                print(f"跳过 {item}: 缺少 result.jpg")
                continue
            
            try:
                # 读取并解析 results.json
                with open(results_json_path, 'r', encoding='utf-8') as f:
                    results_data = json.load(f)
                
                # 检查是否包含 face 类型的检测结果
                has_face = False
                
                # 检查 raw_detections 数组
                if 'raw_detections' in results_data:
                    for detection in results_data['raw_detections']:
                        if isinstance(detection, dict) and 'rois' in detection:
                            for roi in detection['rois']:
                                if isinstance(roi, dict) and roi.get('type') == 'face':
                                    has_face = True
                                    break
                        if has_face:
                            break
                
                # 如果 raw_detections 中没有找到，检查 transformed_detections
                if not has_face and 'transformed_detections' in results_data:
                    for detection in results_data['transformed_detections']:
                        if isinstance(detection, dict) and 'rois' in detection:
                            for roi in detection['rois']:
                                if isinstance(roi, dict) and roi.get('type') == 'face':
                                    has_face = True
                                    break
                        if has_face:
                            break
                
                # 兼容旧格式：直接检查顶层或results字段
                if not has_face:
                    if isinstance(results_data, list):
                        # 如果是列表，检查每个元素
                        for result in results_data:
                            if isinstance(result, dict) and result.get('type') == 'face':
                                has_face = True
                                break
                    elif isinstance(results_data, dict):
                        # 如果是字典，检查是否有 face 类型
                        if results_data.get('type') == 'face':
                            has_face = True
                        # 或者检查是否有包含 face 的结果列表
                        elif 'results' in results_data:
                            for result in results_data['results']:
                                if isinstance(result, dict) and result.get('type') == 'face':
                                    has_face = True
                                    break
                
                if has_face:
                    sample_options.append({
                        'name': item,
                        'path': item_path,
                        'image_path': result_jpg_path
                    })
                    print(f"✓ 添加样本: {item}")
                else:
                    print(f"跳过 {item}: results.json 中没有 face 类型的检测结果")
                    
            except json.JSONDecodeError as e:
                print(f"跳过 {item}: results.json 格式错误 - {e}")
    except Exception as e:
                print(f"跳过 {item}: 处理 results.json 时出错 - {e}")
    
    except Exception as e:
        print(f"获取样本选项时出错: {e}")
    
    print(f"总共找到 {len(sample_options)} 个有效样本")
    return sample_options

# 可用的样本选项（动态获取）
SAMPLE_OPTIONS = get_sample_options()

# 设置默认样本
if SAMPLE_OPTIONS:
    current_sample = SAMPLE_OPTIONS[0]['name']

# 存储ROS话题状态
ptz_status = {"status": "初始化中...", "timestamp": ""}
ros_status = {"connected": False, "error": None}
last_target_point_data = None  # 缓存最后的目标点数据

if ROS2_AVAILABLE:
    class PTZControlNode(Node):
        """ROS2节点，用于监听PTZ状态和发布样本命令"""
        
        def __init__(self):
            super().__init__('ptz_web_control')
            
            # 创建订阅者的QoS配置 (用于接收PTZ状态)
            subscriber_qos = QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                durability=DurabilityPolicy.VOLATILE,
                history=HistoryPolicy.KEEP_LAST,
                depth=10
            )
            
            # 创建发布者的QoS配置 (用于发送样本命令)
            # 使用RELIABLE确保消息能够可靠传递
            publisher_qos = QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.VOLATILE,
                history=HistoryPolicy.KEEP_LAST,
                depth=10
            )
            
            # 创建订阅者监听PTZ状态
            self.ptz_subscription = self.create_subscription(
                Bool,  # 根据实际消息类型调整，可能是Bool、String或其他类型
                '/ptz_manager/ptz_stable',
                self.ptz_status_callback,
                10,
            )
            
            # 创建订阅者监听外部目标点（用于确认样本设置）
            self.external_target_point_sub = self.create_subscription(
                String,
                '/ptz_manager/external_target_point',
                self.external_target_point_callback,
                subscriber_qos
            )
            
            # 创建订阅者监听人脸检测数据（用于计算帧率）
            try:
                # QoS配置 - 订阅用
                qos_profile_sub = QoSProfile(
                    reliability=ReliabilityPolicy.BEST_EFFORT,
                    history=HistoryPolicy.KEEP_LAST,
                    depth=1
                )
                
                # 订阅人脸检测话题
                self.face_detection_sub = self.create_subscription(
                    PerceptionTargets,
                    '/hobot_face_landmarks_detection',
                    self.face_detection_callback,
                    qos_profile_sub
                )
                self.get_logger().info('已订阅人脸检测话题: /hobot_face_landmarks_detection')
            except Exception as e:
                self.get_logger().warn(f'订阅人脸检测话题失败: {e}')
                self.face_detection_sub = None
            
            # 创建发布者发送样本命令
            self.sample_publisher = self.create_publisher(
                String,
                '/sample_face/set_index',
                publisher_qos
            )
            
            # 创建发布者发送PTZ模式命令
            try:
                from std_msgs.msg import Int32
                self.mode_publisher = self.create_publisher(
                    Int32,
                    '/ptz_manager/mode',
                    publisher_qos
                )
                self.get_logger().info(f'发布话题: /ptz_manager/mode (QoS: RELIABLE)')
            except ImportError as e:
                self.get_logger().error(f'导入Int32消息类型失败: {e}')
                self.mode_publisher = None
            
            # 创建定时器定期检查连接状态
            self.status_timer = self.create_timer(5.0, self.check_connection_status)
            
            # 创建定时器定期计算人脸检测帧率
            self.face_fps_timer = self.create_timer(1.0, self.calculate_face_fps)
            
            # 创建定时器定期获取系统状态
            self.system_status_timer = self.create_timer(10.0, self.update_system_status)
            
            # 创建定时器定期检查硬件状态（每5秒检查一次）
            self.hardware_status_timer = self.create_timer(10.0, self.update_hardware_status)
            
            self.get_logger().info('PTZ控制节点已启动')
            self.get_logger().info(f'订阅话题: /ptz_manager/ptz_stable (QoS: BEST_EFFORT)')
            self.get_logger().info(f'订阅话题: /ptz_manager/external_target_point (QoS: BEST_EFFORT)')
            self.get_logger().info(f'发布话题: /sample_face/set_index (QoS: RELIABLE)')
            self.last_message_time = time.time()
            
            # 用于跟踪样本设置状态
            self.pending_sample = None
            self.sample_confirmation_timeout = 10.0  # 10秒超时
            self.sample_send_time = None
            
            # 用于计算人脸检测帧率
            self.face_message_times = []
            self.face_fps = 0.0
            self.last_face_message_time = 0
            
            # 初始化状态
            global ptz_status, ros_status
            ptz_status = {
                "status": "ROS节点已启动，等待数据...",
                "timestamp": datetime.now().strftime('%Y-%m-%d %H:%M:%S')
            }
            ros_status = {"connected": True, "error": None}
        
        def ptz_status_callback(self, msg):
            """PTZ状态回调函数"""
            try:
                self.last_message_time = time.time()
                
                # 根据消息类型解析状态
                if hasattr(msg, 'data'):
                    if isinstance(msg.data, bool):
                        status_data = "稳定" if msg.data else "不稳定"
                    else:
                        status_data = str(msg.data)
                else:
                    status_data = str(msg)
                
                timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
                
                global ptz_status
                ptz_status = {
                    "status": status_data,
                    "timestamp": timestamp
                }
                
                # 通过WebSocket发送更新
                socketio.emit('ptz_status_update', ptz_status)
                self.get_logger().info(f'PTZ状态更新: {status_data}')
                
            except Exception as e:
                self.get_logger().error(f'处理PTZ状态消息时出错: {e}')
                ptz_status = {
                    "status": f"消息处理错误: {str(e)}",
                    "timestamp": datetime.now().strftime('%Y-%m-%d %H:%M:%S')
                }
                socketio.emit('ptz_status_update', ptz_status)
                
        def check_connection_status(self):
            """检查连接状态"""
            current_time = time.time()
            if current_time - self.last_message_time > 10:  # 10秒没有收到消息
                global ptz_status
            ptz_status = {
                        "status": "等待数据... (可能话题未发布)",
                "timestamp": datetime.now().strftime('%Y-%m-%d %H:%M:%S')
            }
            socketio.emit('ptz_status_update', ptz_status)
        
        def publish_sample_command(self, sample_name):
            """发布样本命令"""
            try:
                # 设置待确认状态
                self.pending_sample = sample_name
                self.sample_send_time = time.time()
                
                msg = String()
                msg.data = sample_name
                self.sample_publisher.publish(msg)
                self.get_logger().info(f'发布样本命令: {sample_name} (等待确认...)')
                
                return True, f'成功发布样本: {sample_name}'
            except Exception as e:
                self.get_logger().error(f'发布样本命令时出错: {e}')
                
                # 尝试使用备用QoS配置重新创建发布者
                try:
                    self.get_logger().warn('尝试使用备用QoS配置重新创建发布者...')
                    
                    # 销毁现有发布者
                    self.destroy_publisher(self.sample_publisher)
                    
                    # 使用BEST_EFFORT策略重新创建发布者
                    fallback_qos = QoSProfile(
                        reliability=ReliabilityPolicy.BEST_EFFORT,
                        durability=DurabilityPolicy.VOLATILE,
                        history=HistoryPolicy.KEEP_LAST,
                        depth=10
                    )
                    
                    self.sample_publisher = self.create_publisher(
                        String,
                        '/sample_face/set_index',
                        fallback_qos
                    )
                    
                    self.get_logger().info('发布者已使用BEST_EFFORT策略重新创建')
                    
                    # 重新尝试发布
                    self.pending_sample = sample_name
                    self.sample_send_time = time.time()
                    
                    msg = String()
                    msg.data = sample_name
                    self.sample_publisher.publish(msg)
                    self.get_logger().info(f'使用备用配置发布样本命令: {sample_name} (等待确认...)')
                    return True, f'成功发布样本 (备用配置): {sample_name}'
                    
                except Exception as fallback_error:
                    self.get_logger().error(f'备用发布配置也失败: {fallback_error}')
                    # 清除待确认状态
                    self.pending_sample = None
                    self.sample_send_time = None
                    return False, f'发布失败: {str(e)} (备用配置也失败: {str(fallback_error)})'

        def external_target_point_callback(self, msg):
            """外部目标点回调函数，用于确认样本设置状态"""
            global last_target_point_data
            
            try:
                # 解析JSON消息
                target_data = json.loads(msg.data)
                
                # 获取sample_file字段
                received_sample = target_data.get('sample_file', '')
                
                self.get_logger().info(f'收到外部目标点数据: sample_file={received_sample}')
                
                # 检查是否与待确认的样本匹配
                confirmation_sent = False
                if self.pending_sample and received_sample:
                    if received_sample == self.pending_sample:
                        # 样本设置成功
                        elapsed_time = time.time() - self.sample_send_time if self.sample_send_time else 0
                        self.get_logger().info(f'样本设置成功确认: {received_sample} (耗时: {elapsed_time:.2f}秒)')
                        
                        # 通过WebSocket发送成功确认
                        confirmation_data = {
                            'success': True,
                            'sample_name': received_sample,
                            'message': f'样本 {received_sample} 设置成功',
                            'target_data': target_data,
                            'elapsed_time': elapsed_time
                        }
                        socketio.emit('sample_confirmation', confirmation_data)
                        confirmation_sent = True
                        
                        # 清除待确认状态
                        self.pending_sample = None
                        self.sample_send_time = None
                        
                    else:
                        # 收到的样本名称不匹配
                        self.get_logger().warn(f'样本名称不匹配: 期望={self.pending_sample}, 收到={received_sample}')
                
                # 构建目标点数据更新
                target_point_update = {
                    'x': target_data.get('x', 0),
                    'y': target_data.get('y', 0),
                    'camera_width': target_data.get('camera_width', 0),
                    'camera_height': target_data.get('camera_height', 0),
                    'crop_width': target_data.get('crop_width', 0),
                    'crop_height': target_data.get('crop_height', 0),
                    'crop_x': target_data.get('crop_x', 0),
                    'crop_y': target_data.get('crop_y', 0),
                    'sample_file': received_sample,
                    'original_width': target_data.get('original_width', 0),
                    'original_height': target_data.get('original_height', 0),
                    'timestamp': datetime.now().strftime('%Y-%m-%d %H:%M:%S'),
                    'is_confirmation': confirmation_sent
                }
                
                # 缓存目标点数据
                last_target_point_data = target_point_update.copy()
                
                # 发送目标点数据更新（用于实时显示当前样本状态）
                socketio.emit('target_point_update', target_point_update)
                
            except json.JSONDecodeError as e:
                self.get_logger().error(f'解析外部目标点JSON失败: {e}')
            except Exception as e:
                self.get_logger().error(f'处理外部目标点消息时出错: {e}')

        def face_detection_callback(self, msg):
            """人脸检测回调函数"""
            try:
                current_time = time.time()
                self.last_face_message_time = current_time  # 更新最后收到消息的时间
                self.face_message_times.append(current_time)
                
                # 只保留最近5秒的消息时间戳
                cutoff_time = current_time - 5.0
                self.face_message_times = [t for t in self.face_message_times if t > cutoff_time]
                
            except Exception as e:
                self.get_logger().error(f'处理人脸检测数据时出错: {e}')

        def calculate_face_fps(self):
            """计算人脸检测帧率"""
            try:
                current_time = time.time()
                
                # 过滤出最近5秒的消息
                self.face_message_times = [t for t in self.face_message_times if current_time - t <= 5.0]
                
                # 计算帧率
                self.face_fps = len(self.face_message_times) / 5.0
                
                # 检查是否长时间没有数据
                if hasattr(self, 'last_face_message_time') and current_time - self.last_face_message_time > 10.0:
                    self.face_fps = 0.0
                    self.face_message_times = []
                
                # 通过WebSocket发送更新
                try:
                    socketio.emit('face_fps_update', {
                        'fps': round(self.face_fps, 2),
                        'message_count': len(self.face_message_times),
                        'last_message_time': self.last_face_message_time if hasattr(self, 'last_face_message_time') else 0,
                        'active': self.face_fps > 0,
                        'timestamp': datetime.now().strftime('%Y-%m-%d %H:%M:%S')
                    })
                except Exception as e:
                    self.get_logger().warn(f"发送人脸帧率WebSocket更新失败: {e}")
                
                self.get_logger().info(f"人脸检测帧率: {self.face_fps:.2f} FPS, 消息数: {len(self.face_message_times)}")
                
            except Exception as e:
                self.get_logger().error(f"计算人脸检测帧率时出错: {e}")
                self.face_fps = 0.0

        def update_system_status(self):
            """获取系统温度信息"""
            status = {
                'temperature': {
                    'cpu': 0.0,
                    'bpu': 0.0,
                    'ddr': 0.0
                },
                'timestamp': datetime.now().strftime('%Y-%m-%d %H:%M:%S')
            }
            
            try:
                # 执行hrut_somstatus命令获取系统状态
                result = subprocess.run(['hrut_somstatus'], 
                                      capture_output=True, 
                                      text=True, 
                                      timeout=5)
                
                if result.returncode == 0:
                    output = result.stdout
                    
                    # 解析温度信息
                    temp_patterns = {
                        'ddr': r'DDR\s*:\s*([\d.]+)\s*\(C\)',
                        'bpu': r'BPU\s*:\s*([\d.]+)\s*\(C\)',
                        'cpu': r'CPU\s*:\s*([\d.]+)\s*\(C\)'
                    }
                    
                    for temp_type, pattern in temp_patterns.items():
                        match = re.search(pattern, output)
                        if match:
                            status['temperature'][temp_type] = float(match.group(1))
                
                else:
                    self.get_logger().warn(f"hrut_somstatus命令执行失败: {result.stderr}")
                    
            except subprocess.TimeoutExpired:
                self.get_logger().warn("hrut_somstatus命令执行超时")
            except FileNotFoundError:
                self.get_logger().warn("hrut_somstatus命令未找到")
            except Exception as e:
                self.get_logger().error(f"获取系统状态时出错: {e}")
            
            # 通过WebSocket发送系统状态更新
            try:
                socketio.emit('system_status_update', status)
                self.get_logger().info(f"系统温度更新: CPU={status['temperature']['cpu']}°C, BPU={status['temperature']['bpu']}°C, DDR={status['temperature']['ddr']}°C")
            except Exception as e:
                self.get_logger().warn(f"发送系统状态WebSocket更新失败: {e}")

        def publish_ptz_mode_command(self, mode):
            """发布PTZ模式命令"""
            try:
                if self.mode_publisher is None:
                    return False, "PTZ模式发布者未初始化"
                
                # 验证模式参数
                if mode not in [1, 2, 3]:
                    return False, f"无效的PTZ模式: {mode}，仅支持模式1、2、3"
                
                from std_msgs.msg import Int32
                msg = Int32()
                msg.data = mode
                self.mode_publisher.publish(msg)
                
                mode_names = {
                    1: "仅云台防抖",
                    2: "人脸跟踪+Roll稳定", 
                    3: "人脸跟踪+Roll归零"
                }
                
                self.get_logger().info(f'发布PTZ模式命令: 模式{mode} ({mode_names[mode]})')
                
                return True, f'成功切换到模式{mode}: {mode_names[mode]}'
                
            except Exception as e:
                self.get_logger().error(f'发布PTZ模式命令时出错: {e}')
                return False, f'发布失败: {str(e)}'

        def update_hardware_status(self):
            """检查硬件状态"""
            try:
                # 检查各个硬件设备文件是否存在
                hardware_devices = {
                    'imu': '/dev/ptz_imu',
                    'motor': '/dev/ptz_485', 
                    'camera': '/dev/video1'
                }
                
                status = {
                    'success': True,
                    'hardware': {},
                    'timestamp': datetime.now().strftime('%Y-%m-%d %H:%M:%S')
                }
                
                for device_name, device_path in hardware_devices.items():
                    try:
                        # 检查设备文件是否存在
                        exists = os.path.exists(device_path)
                        status['hardware'][device_name] = {
                            'exists': exists,
                            'path': device_path,
                            'status': 'online' if exists else 'offline'
                        }
                    except Exception as e:
                        status['hardware'][device_name] = {
                            'exists': False,
                            'path': device_path,
                            'status': 'error',
                            'error': str(e)
                        }
                
                # 通过WebSocket发送硬件状态更新
                try:
                    socketio.emit('hardware_status_update', status)
                    self.get_logger().info(f"硬件状态更新: IMU={status['hardware']['imu']['status']}, 电机={status['hardware']['motor']['status']}, 相机={status['hardware']['camera']['status']}")
                except Exception as e:
                    self.get_logger().warn(f"发送硬件状态WebSocket更新失败: {e}")
                    
            except Exception as e:
                self.get_logger().error(f"检查硬件状态时出错: {e}")
                # 发送错误状态
                try:
                    error_status = {
                        'hardware': {
                            'imu': {'status': 'error', 'error': str(e)},
                            'motor': {'status': 'error', 'error': str(e)},
                            'camera': {'status': 'error', 'error': str(e)}
                        },
                        'timestamp': datetime.now().strftime('%Y-%m-%d %H:%M:%S')
                    }
                    socketio.emit('hardware_status_update', error_status)
                except Exception as emit_error:
                    self.get_logger().error(f"发送硬件错误状态失败: {emit_error}")

else:
    # ROS2不可用时的虚拟类
    class PTZControlNode:
        def __init__(self):
            pass
        
        def publish_sample_command(self, sample_name):
            return False, "ROS2不可用"

        def publish_ptz_mode_command(self, mode):
            return False, "ROS2不可用"

# 全局ROS节点实例
ros_node = None
ros_executor = None
ros_thread = None

def init_ros():
    """初始化ROS2"""
    global ros_node, ros_executor, ros_thread, ros_status
    
    if not ROS2_AVAILABLE:
        ros_status = {"connected": False, "error": "ROS2 Python模块不可用"}
        return False
    
    try:
        # 检查ROS2环境
        if 'ROS_DISTRO' not in os.environ:
            # 尝试source ROS2环境
            setup_script = "/opt/tros/humble/setup.bash"
            if os.path.exists(setup_script):
                print(f"尝试加载ROS2环境: {setup_script}")
                # 注意：这里只是提示，实际的环境变量需要在启动脚本中设置
            else:
                print("警告: 未找到ROS2环境设置文件")
        
        rclpy.init()
        ros_node = PTZControlNode()
        ros_executor = MultiThreadedExecutor(num_threads=2)
        ros_executor.add_node(ros_node)
        
        # 在单独线程中运行ROS执行器
        ros_thread = threading.Thread(target=ros_executor.spin, daemon=True)
        ros_thread.start()
                
        print("ROS2节点初始化成功")
        ros_status = {"connected": True, "error": None}
        return True
        
    except Exception as e:
        print(f"ROS2初始化失败: {e}")
        ros_status = {"connected": False, "error": str(e)}
        return False

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

@app.route('/')
def index():
    """主页"""
    return render_template('index_ros2.html')

@app.route('/images')
def images_page():
    """图片浏览页面"""
    return render_template('images_ros2.html')

@app.route('/control')
def control_page():
    """控制页面"""
    return render_template('control_ros2.html', 
                         sample_options=SAMPLE_OPTIONS, 
                         current_sample=current_sample)

@app.route('/offline_test')
def offline_test_page():
    """离线资源测试页面"""
    return render_template('offline_test.html')

@app.route('/api/images')
def api_images():
    """获取图片列表的API"""
    images = get_images()
    return jsonify(images)

@app.route('/api/samples')
def api_samples():
    """获取样本列表的API"""
    return jsonify(SAMPLE_OPTIONS)

@app.route('/api/refresh_samples')
def api_refresh_samples():
    """刷新样本列表的API"""
    global SAMPLE_OPTIONS, current_sample
    SAMPLE_OPTIONS = get_sample_options()
    
    # 如果当前样本不在新列表中，重置为第一个
    current_sample_names = [sample['name'] for sample in SAMPLE_OPTIONS]
    if current_sample not in current_sample_names:
        current_sample = SAMPLE_OPTIONS[0]['name'] if SAMPLE_OPTIONS else ""
    
    return jsonify({
        'success': True,
        'samples': SAMPLE_OPTIONS,
        'current_sample': current_sample,
        'message': f'刷新完成，找到 {len(SAMPLE_OPTIONS)} 个有效样本'
    })

@app.route('/sample_images/<sample_name>')
def serve_sample_image(sample_name):
    """提供样本图片文件"""
    try:
        # 查找对应的样本
        for sample in SAMPLE_OPTIONS:
            if sample['name'] == sample_name:
                return send_from_directory(
                    os.path.dirname(sample['image_path']), 
                    os.path.basename(sample['image_path'])
                )
        
        # 如果没找到，返回404
        return "Sample not found", 404
    except Exception as e:
        print(f"提供样本图片时出错: {e}")
        return "Error serving sample image", 500

@app.route('/api/execute_script', methods=['POST'])
def execute_script():
    """执行脚本的API - 使用ROS2发布消息"""
    try:
        global current_sample, ros_node
        
        data = request.get_json()
        selected_sample = data.get('sample', current_sample)
        
        # 验证样本是否存在
        sample_names = [sample['name'] for sample in SAMPLE_OPTIONS]
        if selected_sample not in sample_names:
            return jsonify({
                'success': False, 
                'message': f'样本 "{selected_sample}" 不存在，请刷新样本列表'
            })
        
        current_sample = selected_sample
        
        if not ros_status["connected"]:
            return jsonify({
                'success': False, 
                'message': f'ROS节点未连接: {ros_status.get("error", "未知错误")}'
            })
        
        if ros_node is None:
            return jsonify({
                'success': False, 
                'message': 'ROS节点未初始化'
            })
        
        # 使用ROS节点发布消息
        success, message = ros_node.publish_sample_command(selected_sample)
        
        return jsonify({
            'success': success,
            'message': message,
            'output': f'通过ROS2发布样本: {selected_sample}' if success else '',
            'current_sample': current_sample
        })
            
    except Exception as e:
        return jsonify({
            'success': False, 
            'message': f'执行错误: {str(e)}'
        })

@app.route('/api/ptz_status')
def api_ptz_status():
    """获取PTZ状态的API"""
    return jsonify({
        **ptz_status,
        "ros_connected": ros_status["connected"],
        "ros_error": ros_status.get("error")
    })

@app.route('/api/ros_status')
def api_ros_status():
    """获取ROS连接状态的API"""
    return jsonify(ros_status)

@app.route('/api/system_temperature')
def api_system_temperature():
    """获取系统温度的API"""
    try:
        # 执行hrut_somstatus命令获取系统状态
        result = subprocess.run(['hrut_somstatus'], 
                              capture_output=True, 
                              text=True, 
                              timeout=5)
        
        temp_data = {
            'cpu_temp': 0.0,
            'bpu_temp': 0.0,
            'ddr_temp': 0.0,
            'timestamp': datetime.now().strftime('%Y-%m-%d %H:%M:%S'),
            'success': False
        }
        
        if result.returncode == 0:
            output = result.stdout
            
            # 解析温度信息
            temp_patterns = {
                'ddr_temp': r'DDR\s*:\s*([\d.]+)\s*\(C\)',
                'bpu_temp': r'BPU\s*:\s*([\d.]+)\s*\(C\)',
                'cpu_temp': r'CPU\s*:\s*([\d.]+)\s*\(C\)'
            }
            
            for temp_key, pattern in temp_patterns.items():
                match = re.search(pattern, output)
                if match:
                    temp_data[temp_key] = float(match.group(1))
            
            temp_data['success'] = True
        
        return jsonify(temp_data)
        
    except subprocess.TimeoutExpired:
            return jsonify({
            'cpu_temp': 0.0,
            'bpu_temp': 0.0,
            'ddr_temp': 0.0,
            'error': 'hrut_somstatus命令执行超时',
                'success': False, 
            'timestamp': datetime.now().strftime('%Y-%m-%d %H:%M:%S')
        })
    except FileNotFoundError:
        return jsonify({
            'cpu_temp': 0.0,
            'bpu_temp': 0.0,
            'ddr_temp': 0.0,
            'error': 'hrut_somstatus命令未找到',
            'success': False, 
            'timestamp': datetime.now().strftime('%Y-%m-%d %H:%M:%S')
        })
    except Exception as e:
        return jsonify({
            'cpu_temp': 0.0,
            'bpu_temp': 0.0,
            'ddr_temp': 0.0,
            'error': f'获取系统温度时出错: {e}',
            'success': False, 
            'timestamp': datetime.now().strftime('%Y-%m-%d %H:%M:%S')
        })

@app.route('/images/<filename>')
def serve_image(filename):
    """提供图片文件"""
    return send_from_directory(IMAGES_DIR, filename)

@socketio.on('connect')
def handle_connect():
    """WebSocket连接处理"""
    print('Client connected')
    emit('ptz_status_update', {
        **ptz_status,
        "ros_connected": ros_status["connected"],
        "ros_error": ros_status.get("error")
    })

@socketio.on('disconnect')
def handle_disconnect():
    """WebSocket断开处理"""
    print('Client disconnected')

def cleanup_ros():
    """清理ROS资源"""
    global ros_node, ros_executor, ros_thread
    try:
        if ros_executor:
            ros_executor.shutdown()
        if ros_node:
            ros_node.destroy_node()
        if ROS2_AVAILABLE:
            rclpy.shutdown()
        print("ROS2资源已清理")
    except Exception as e:
        print(f"清理ROS2资源时出错: {e}")

def signal_handler(sig, frame):
    """信号处理器"""
    print('\n正在关闭服务器...')
    cleanup_ros()
    sys.exit(0)

@app.route('/api/target_point_status')
def api_target_point_status():
    """获取目标点状态的API"""
    global last_target_point_data
    
    try:
        if last_target_point_data:
            return jsonify({
                'success': True,
                'data': last_target_point_data,
                'timestamp': datetime.now().strftime('%Y-%m-%d %H:%M:%S')
            })
        else:
            return jsonify({
                'success': False,
                'message': '暂无目标点数据',
                'timestamp': datetime.now().strftime('%Y-%m-%d %H:%M:%S')
            })
    except Exception as e:
        return jsonify({
            'success': False,
            'error': f'获取目标点状态时出错: {e}',
            'timestamp': datetime.now().strftime('%Y-%m-%d %H:%M:%S')
        })

@app.route('/api/face_fps_status')
def api_face_fps_status():
    """获取人脸检测帧率状态的API"""
    if ros_node and hasattr(ros_node, 'face_fps'):
        return jsonify({
            'fps': round(ros_node.face_fps, 2),
            'message_count': len(ros_node.face_message_times) if hasattr(ros_node, 'face_message_times') else 0,
            'last_message_time': ros_node.last_face_message_time if hasattr(ros_node, 'last_face_message_time') else 0,
            'active': ros_node.face_fps > 0 if hasattr(ros_node, 'face_fps') else False,
            'timestamp': datetime.now().strftime('%Y-%m-%d %H:%M:%S')
        })
    else:
        return jsonify({
            'fps': 0.0,
            'message_count': 0,
            'last_message_time': 0,
            'active': False,
            'timestamp': datetime.now().strftime('%Y-%m-%d %H:%M:%S')
        })

@app.route('/api/system_status')
def api_system_status():
    """获取系统温度状态的API"""
    try:
        # 执行hrut_somstatus命令获取系统状态
        result = subprocess.run(['hrut_somstatus'], 
                              capture_output=True, 
                              text=True, 
                              timeout=5)
        
        status = {
            'temperature': {
                'cpu': 0.0,
                'bpu': 0.0,
                'ddr': 0.0
            },
            'timestamp': datetime.now().strftime('%Y-%m-%d %H:%M:%S')
        }
        
        if result.returncode == 0:
            output = result.stdout
            
            # 解析温度信息
            temp_patterns = {
                'ddr': r'DDR\s*:\s*([\d.]+)\s*\(C\)',
                'bpu': r'BPU\s*:\s*([\d.]+)\s*\(C\)',
                'cpu': r'CPU\s*:\s*([\d.]+)\s*\(C\)'
            }
            
            for temp_type, pattern in temp_patterns.items():
                match = re.search(pattern, output)
                if match:
                    status['temperature'][temp_type] = float(match.group(1))
        
        return jsonify(status)
        
    except subprocess.TimeoutExpired:
        return jsonify({'error': 'hrut_somstatus命令执行超时'})
    except FileNotFoundError:
        return jsonify({'error': 'hrut_somstatus命令未找到'})
    except Exception as e:
        return jsonify({'error': f'获取系统状态时出错: {e}'})

@app.route('/api/hardware_status')
def api_hardware_status():
    """获取硬件设备状态的API"""
    try:
        # 检查各个硬件设备文件是否存在
        hardware_devices = {
            'imu': '/dev/ptz_imu',
            'motor': '/dev/ptz_485', 
            'camera': '/dev/video1'
        }
        
        status = {
            'success': True,
            'hardware': {},
            'timestamp': datetime.now().strftime('%Y-%m-%d %H:%M:%S')
        }
        
        for device_name, device_path in hardware_devices.items():
            try:
                # 检查设备文件是否存在
                exists = os.path.exists(device_path)
                status['hardware'][device_name] = {
                    'exists': exists,
                    'path': device_path,
                    'status': 'online' if exists else 'offline'
                }
            except Exception as e:
                status['hardware'][device_name] = {
                    'exists': False,
                    'path': device_path,
                    'status': 'error',
                    'error': str(e)
                }
        
        return jsonify(status)
        
    except Exception as e:
        return jsonify({
            'success': False,
            'error': f'获取硬件状态时出错: {e}',
            'timestamp': datetime.now().strftime('%Y-%m-%d %H:%M:%S')
        })

@app.route('/api/ptz_mode', methods=['POST'])
def api_set_ptz_mode():
    """设置PTZ模式的API"""
    try:
        global current_ptz_mode, ros_node
        
        data = request.get_json()
        mode = data.get('mode')
        
        if mode is None:
            return jsonify({
                'success': False, 
                'message': '缺少mode参数'
            })
        
        # 验证模式参数
        try:
            mode = int(mode)
            if mode not in [1, 2, 3]:
                return jsonify({
                    'success': False, 
                    'message': f'无效的PTZ模式: {mode}，仅支持模式1、2、3'
                })
        except (ValueError, TypeError):
            return jsonify({
                'success': False, 
                'message': f'模式参数必须是整数: {mode}'
            })
        
        if not ros_status["connected"]:
            return jsonify({
                'success': False, 
                'message': f'ROS节点未连接: {ros_status.get("error", "未知错误")}'
            })
        
        if ros_node is None:
            return jsonify({
                'success': False, 
                'message': 'ROS节点未初始化'
            })
        
        # 使用ROS节点发布模式命令
        success, message = ros_node.publish_ptz_mode_command(mode)
        
        if success:
            # 更新当前模式缓存
            current_ptz_mode = mode
        
        return jsonify({
            'success': success,
            'message': message,
            'current_mode': current_ptz_mode,
            'mode_description': {
                1: "仅云台防抖",
                2: "人脸跟踪+Roll稳定", 
                3: "人脸跟踪+Roll归零"
            }.get(mode, "未知模式")
        })
            
    except Exception as e:
        return jsonify({
            'success': False, 
            'message': f'执行错误: {str(e)}'
        })

@app.route('/api/ptz_mode')
def api_get_ptz_mode():
    """获取当前PTZ模式的API"""
    global current_ptz_mode
    
    mode_descriptions = {
        1: "仅云台防抖",
        2: "人脸跟踪+Roll稳定", 
        3: "人脸跟踪+Roll归零"
    }
    
    return jsonify({
        'success': True,
        'current_mode': current_ptz_mode,
        'mode_description': mode_descriptions.get(current_ptz_mode, "未知模式"),
        'available_modes': [
            {'mode': 1, 'description': "仅云台防抖"},
            {'mode': 2, 'description': "人脸跟踪+Roll稳定"},
            {'mode': 3, 'description': "人脸跟踪+Roll归零"}
        ],
        'timestamp': datetime.now().strftime('%Y-%m-%d %H:%M:%S')
    })

# 进程管理器类
class ProcessManager:
    def __init__(self):
        self.processes = {}
        self.log_files = {}
        self.scripts = {
            'cam': '/home/sunrise/ptz_ws/start_cam.sh',
            'ptz': '/home/sunrise/ptz_ws/start_ptz.sh',
            'sample': '/home/sunrise/ptz_ws/start_sample.sh'
        }
        self.log_dir = '/tmp/ptz_logs'
        self.max_log_size = 1024 * 1024  # 1MB
        
        # 确保日志目录存在
        os.makedirs(self.log_dir, exist_ok=True)
        
    def start_process(self, process_type):
        """启动指定进程"""
        try:
            if process_type in self.processes and self.processes[process_type].poll() is None:
                return {'success': False, 'message': '进程已在运行'}
            
            script_path = self.scripts.get(process_type)
            if not script_path:
                return {'success': False, 'message': '未知的进程类型'}
            
            if not os.path.exists(script_path):
                return {'success': False, 'message': f'脚本文件不存在: {script_path}'}
            
            # 检查脚本是否有执行权限
            if not os.access(script_path, os.X_OK):
                try:
                    # 尝试添加执行权限
                    os.chmod(script_path, 0o755)
                    logger.info(f"已为脚本 {script_path} 添加执行权限")
                except Exception as e:
                    return {'success': False, 'message': f'脚本无执行权限且无法修改: {str(e)}'}
            
            # 创建日志文件路径
            log_file_path = os.path.join(self.log_dir, f'{process_type}.log')
            self.log_files[process_type] = log_file_path
            
            # 启动进程，输出到日志文件
            with open(log_file_path, 'w') as log_file:
                log_file.write(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S')}] 启动进程: {process_type}\n")
                log_file.write(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S')}] 脚本路径: {script_path}\n")
                log_file.flush()
                
                # 使用 subprocess.Popen 启动进程，输出重定向到文件
                process = subprocess.Popen(
                    ['bash', script_path],
                    stdout=log_file,
                    stderr=subprocess.STDOUT,
                    universal_newlines=True,
                    cwd=os.path.dirname(script_path),
                    # 设置新的进程组，便于后续管理
                    preexec_fn=os.setsid if hasattr(os, 'setsid') else None
                )
            
            self.processes[process_type] = process
            
            logger.info(f"进程 {process_type} 启动成功，PID: {process.pid}")
            
            # 异步通知状态更新
            self._emit_status_update(process_type, 'running')
            
            return {'success': True, 'message': f'进程启动成功，PID: {process.pid}'}
            
        except Exception as e:
            logger.error(f"启动进程 {process_type} 失败: {str(e)}")
            return {'success': False, 'message': f'启动失败: {str(e)}'}
    
    def stop_process(self, process_type):
        """停止指定进程"""
        try:
            if process_type not in self.processes:
                return {'success': False, 'message': '进程未运行'}
            
            process = self.processes[process_type]
            if process.poll() is not None:
                # 进程已经结束，清理记录
                del self.processes[process_type]
                self._emit_status_update(process_type, 'stopped')
                return {'success': True, 'message': '进程已停止'}
            
            try:
                # 尝试优雅停止整个进程组
                if hasattr(os, 'killpg'):
                    os.killpg(os.getpgid(process.pid), signal.SIGTERM)
                else:
                    process.terminate()
                
                # 等待5秒
                try:
                    process.wait(timeout=5)
                    logger.info(f"进程 {process_type} 优雅停止成功")
                except subprocess.TimeoutExpired:
                    # 强制杀死进程组
                    if hasattr(os, 'killpg'):
                        os.killpg(os.getpgid(process.pid), signal.SIGKILL)
                    else:
                        process.kill()
                    process.wait()
                    logger.warning(f"进程 {process_type} 强制停止")
                    
            except ProcessLookupError:
                # 进程已经不存在
                logger.info(f"进程 {process_type} 已经结束")
            except Exception as e:
                logger.error(f"停止进程 {process_type} 时出错: {str(e)}")
                # 尝试强制杀死
                try:
                    process.kill()
                    process.wait()
                except:
                    pass
            
            # 清理
            if process_type in self.processes:
                del self.processes[process_type]
            
            # 写入日志文件
            if process_type in self.log_files:
                try:
                    with open(self.log_files[process_type], 'a') as log_file:
                        log_file.write(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S')}] 进程停止\n")
                except:
                    pass
            
            # 通过WebSocket发送状态更新
            self._emit_status_update(process_type, 'stopped')
            
            logger.info(f"进程 {process_type} 停止成功")
            return {'success': True, 'message': '进程停止成功'}
            
        except Exception as e:
            logger.error(f"停止进程 {process_type} 失败: {str(e)}")
            return {'success': False, 'message': f'停止失败: {str(e)}'}
    
    def stop_all_processes(self):
        """停止所有进程，包括相关的系统进程"""
        results = []
        
        # 停止我们启动的进程
        for process_type in list(self.processes.keys()):
            result = self.stop_process(process_type)
            results.append(f"{process_type}: {result['message']}")
        
        # 额外杀死所有包含关键词的进程
        try:
            # 查找并杀死包含tros、humble、ptz_ws的进程
            keywords = ['tros', 'humble', 'ptz_ws']
            killed_processes = []
            
            for keyword in keywords:
                try:
                    # 使用 pgrep 查找进程
                    result = subprocess.run(['pgrep', '-f', keyword], 
                                          capture_output=True, text=True)
                    if result.returncode == 0:
                        pids = result.stdout.strip().split('\n')
                        for pid in pids:
                            if pid.strip():
                                try:
                                    # 获取进程信息
                                    proc_info = subprocess.run(['ps', '-p', pid, '-o', 'comm='], 
                                                             capture_output=True, text=True)
                                    if proc_info.returncode == 0:
                                        process_name = proc_info.stdout.strip()
                                        
                                        # 先尝试优雅停止
                                        os.kill(int(pid), signal.SIGTERM)
                                        time.sleep(0.5)
                                        
                                        # 检查是否还在运行
                                        try:
                                            os.kill(int(pid), 0)  # 检查进程是否存在
                                            # 如果还在运行，强制杀死
                                            os.kill(int(pid), signal.SIGKILL)
                                            killed_processes.append(f"{process_name}({pid})")
                                        except ProcessLookupError:
                                            # 进程已经停止
                                            killed_processes.append(f"{process_name}({pid})")
                                            
                                except (ProcessLookupError, ValueError):
                                    # 进程不存在或PID无效
                                    continue
                                except Exception as e:
                                    logger.error(f"杀死进程 {pid} 时出错: {str(e)}")
                                    
                except Exception as e:
                    logger.error(f"查找关键词 {keyword} 的进程时出错: {str(e)}")
            
            if killed_processes:
                results.append(f"额外清理的进程: {', '.join(killed_processes)}")
            else:
                results.append("未找到需要额外清理的进程")
                
        except Exception as e:
            logger.error(f"清理系统进程时出错: {str(e)}")
            results.append(f"清理系统进程失败: {str(e)}")
        
        return {
            'success': True,
            'message': '批量停止完成',
            'details': results
        }
    
    def get_process_status(self, process_type):
        """获取进程状态"""
        try:
            # 检查进程是否在运行
            status = 'stopped'
            if process_type in self.processes:
                process = self.processes[process_type]
                if process.poll() is None:
                    status = 'running'
                else:
                    # 进程已结束，清理
                    del self.processes[process_type]
                    # 写入日志
                    if process_type in self.log_files:
                        try:
                            with open(self.log_files[process_type], 'a') as log_file:
                                log_file.write(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S')}] 进程已结束\n")
                        except:
                            pass
            
            # 从日志文件读取日志
            log_content = self._read_log_file(process_type)
            
            return {
                'status': status,
                'log': log_content
            }
            
        except Exception as e:
            logger.error(f"获取进程状态失败: {str(e)}")
            return {'status': 'stopped', 'log': f'获取状态失败: {str(e)}'}
    
    def _read_log_file(self, process_type, tail_lines=500):
        """从日志文件读取最新的日志内容"""
        if process_type not in self.log_files:
            return '暂无日志文件'
        
        log_file_path = self.log_files[process_type]
        
        try:
            if not os.path.exists(log_file_path):
                return '日志文件不存在'
            
            # 检查文件大小，如果太大则只读取尾部
            file_size = os.path.getsize(log_file_path)
            if file_size > self.max_log_size:
                # 使用 tail 命令读取最后几行
                try:
                    result = subprocess.run(['tail', '-n', str(tail_lines), log_file_path], 
                                          capture_output=True, text=True, timeout=5)
                    if result.returncode == 0:
                        content = result.stdout
                        if file_size > self.max_log_size:
                            content = f"[日志文件过大，只显示最后{tail_lines}行]\n" + content
                        return content
                except:
                    pass
            
            # 直接读取整个文件
            with open(log_file_path, 'r', encoding='utf-8') as f:
                content = f.read()
                lines = content.split('\n')
                if len(lines) > tail_lines:
                    content = f"[日志文件过长，只显示最后{tail_lines}行]\n" + '\n'.join(lines[-tail_lines:])
                return content
                
        except Exception as e:
            logger.error(f"读取日志文件失败: {str(e)}")
            return f'读取日志文件失败: {str(e)}'
    
    def _emit_status_update(self, process_type, status):
        """异步发送状态更新"""
        try:
            if socketio:
                socketio.emit('process_status_update', {
                    'process': process_type,
                    'status': status
                })
        except Exception as e:
            logger.error(f"发送WebSocket状态更新失败: {str(e)}")
    
    def get_log_tail(self, process_type, lines=100):
        """获取日志文件的尾部内容，用于实时更新"""
        return self._read_log_file(process_type, lines)
    
    def clear_logs(self, process_type=None):
        """清理日志文件"""
        try:
            if process_type:
                # 清理指定进程的日志
                if process_type in self.log_files:
                    log_file = self.log_files[process_type]
                    if os.path.exists(log_file):
                        os.remove(log_file)
                    return {'success': True, 'message': f'已清理 {process_type} 的日志'}
                else:
                    return {'success': False, 'message': '进程不存在'}
            else:
                # 清理所有日志
                for log_file in self.log_files.values():
                    if os.path.exists(log_file):
                        os.remove(log_file)
                return {'success': True, 'message': '已清理所有日志'}
                
        except Exception as e:
            logger.error(f"清理日志失败: {str(e)}")
            return {'success': False, 'message': f'清理失败: {str(e)}'}

# 全局进程管理器实例
process_manager = ProcessManager()

# 进程管理API路由
@app.route('/api/process/start', methods=['POST'])
def api_start_process():
    """启动进程的API"""
    try:
        data = request.get_json()
        process_type = data.get('process')
        
        if not process_type:
            return jsonify({'success': False, 'message': '缺少进程类型参数'})
        
        result = process_manager.start_process(process_type)
        return jsonify(result)
        
    except Exception as e:
        logger.error(f"启动进程API失败: {str(e)}")
        return jsonify({'success': False, 'message': f'API错误: {str(e)}'})

@app.route('/api/process/stop', methods=['POST'])
def api_stop_process():
    """停止进程的API"""
    try:
        data = request.get_json()
        process_type = data.get('process')
        
        if not process_type:
            return jsonify({'success': False, 'message': '缺少进程类型参数'})
        
        result = process_manager.stop_process(process_type)
        return jsonify(result)
        
    except Exception as e:
        logger.error(f"停止进程API失败: {str(e)}")
        return jsonify({'success': False, 'message': f'API错误: {str(e)}'})

@app.route('/api/process/status/<process_type>')
def api_process_status(process_type):
    """获取进程状态的API"""
    try:
        result = process_manager.get_process_status(process_type)
        return jsonify(result)
        
    except Exception as e:
        logger.error(f"获取进程状态API失败: {str(e)}")
        return jsonify({'status': 'stopped', 'log': f'API错误: {str(e)}'})

# 进程管理页面路由
@app.route('/process_manager')
def process_manager_page():
    """进程管理页面"""
    return render_template('process_manager.html')

@app.route('/video_display')
def video_display_page():
    """视频显示页面"""
    return render_template('video_display.html')

@app.route('/api/process/stop_all', methods=['POST'])
def api_stop_all_processes():
    """停止所有进程的API，包括系统相关进程"""
    try:
        result = process_manager.stop_all_processes()
        return jsonify(result)
        
    except Exception as e:
        logger.error(f"停止所有进程API失败: {str(e)}")
        return jsonify({'success': False, 'message': f'API错误: {str(e)}'})

@app.route('/api/process/clear_logs', methods=['POST'])
def api_clear_logs():
    """清理日志的API"""
    try:
        data = request.get_json() or {}
        process_type = data.get('process')
        
        result = process_manager.clear_logs(process_type)
        return jsonify(result)
        
    except Exception as e:
        logger.error(f"清理日志API失败: {str(e)}")
        return jsonify({'success': False, 'message': f'API错误: {str(e)}'})

@app.route('/api/process/log_tail/<process_type>')
def api_process_log_tail(process_type):
    """获取进程日志尾部内容的API"""
    try:
        lines = request.args.get('lines', 100, type=int)
        log_content = process_manager.get_log_tail(process_type, lines)
        return jsonify({
            'success': True,
            'log': log_content,
            'process': process_type
        })
        
    except Exception as e:
        logger.error(f"获取进程日志尾部API失败: {str(e)}")
        return jsonify({'success': False, 'log': f'API错误: {str(e)}'})

if __name__ == '__main__':
    # 注册信号处理器
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    # 确保模板目录存在
    os.makedirs('templates', exist_ok=True)
    os.makedirs('static', exist_ok=True)
    
    print("启动PTZ控制系统 (ROS2版本)...")
    print(f"图片目录: {IMAGES_DIR}")
    print(f"脚本路径: {SCRIPT_PATH}")
    
    # 初始化ROS2
    if not init_ros():
        print("警告: ROS2初始化失败，某些功能可能不可用")
        print("请确保:")
        print("1. 已正确安装ROS2")
        print("2. 已source ROS2环境: source /opt/tros/humble/setup.bash")
        print("3. ROS2节点正在运行")
        ptz_status = {
            "status": "ROS2初始化失败",
            "timestamp": datetime.now().strftime('%Y-%m-%d %H:%M:%S')
        }
    
    print("访问地址:")
    print("  - 主页: http://0.0.0.0:5000/")
    print("  - 图片浏览: http://0.0.0.0:5000/images")
    print("  - 控制页面: http://0.0.0.0:5000/control")
    print("\n提示: 使用 Ctrl+C 停止服务器")
    
    try:
        socketio.run(app, host='0.0.0.0', port=5000, debug=False, allow_unsafe_werkzeug=True)
    except KeyboardInterrupt:
        print("\n正在关闭服务器...")
    finally:
        cleanup_ros() 