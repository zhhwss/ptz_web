#!/usr/bin/env python3
"""
ROS2 QoS 诊断工具
用于检查话题的 QoS 配置和兼容性问题
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from std_msgs.msg import String
import sys
import time

class QoSDiagnosticNode(Node):
    def __init__(self):
        super().__init__('qos_diagnostic_node')
        
        # 测试不同的QoS配置
        self.test_qos_configs = [
            {
                'name': 'RELIABLE + VOLATILE',
                'qos': QoSProfile(
                    reliability=ReliabilityPolicy.RELIABLE,
                    durability=DurabilityPolicy.VOLATILE,
                    history=HistoryPolicy.KEEP_LAST,
                    depth=10
                )
            },
            {
                'name': 'BEST_EFFORT + VOLATILE',
                'qos': QoSProfile(
                    reliability=ReliabilityPolicy.BEST_EFFORT,
                    durability=DurabilityPolicy.VOLATILE,
                    history=HistoryPolicy.KEEP_LAST,
                    depth=10
                )
            },
            {
                'name': 'RELIABLE + TRANSIENT_LOCAL',
                'qos': QoSProfile(
                    reliability=ReliabilityPolicy.RELIABLE,
                    durability=DurabilityPolicy.TRANSIENT_LOCAL,
                    history=HistoryPolicy.KEEP_LAST,
                    depth=10
                )
            }
        ]
        
        self.publishers = []
        self.test_topic = '/sample_face/set_index'
        
    def test_publisher_qos(self):
        """测试不同QoS配置的发布者"""
        print(f"\n🔍 测试话题: {self.test_topic}")
        print("=" * 50)
        
        for i, config in enumerate(self.test_qos_configs):
            try:
                print(f"\n📡 测试配置 {i+1}: {config['name']}")
                
                # 创建发布者
                publisher = self.create_publisher(
                    String,
                    self.test_topic,
                    config['qos']
                )
                
                # 等待一段时间让发布者建立连接
                time.sleep(2)
                
                # 尝试发布消息
                msg = String()
                msg.data = f"test_message_{i}"
                publisher.publish(msg)
                
                print(f"  ✅ 成功创建发布者")
                print(f"  📤 成功发布测试消息: {msg.data}")
                
                self.publishers.append(publisher)
                
            except Exception as e:
                print(f"  ❌ 失败: {str(e)}")
        
        print(f"\n✅ 测试完成，成功创建 {len(self.publishers)} 个发布者")
        
    def cleanup(self):
        """清理资源"""
        for publisher in self.publishers:
            try:
                self.destroy_publisher(publisher)
            except:
                pass

def main():
    print("🚀 ROS2 QoS 诊断工具启动")
    print("=" * 50)
    
    try:
        rclpy.init()
        
        # 检查话题是否存在
        print("📋 检查现有话题...")
        import subprocess
        try:
            result = subprocess.run(['ros2', 'topic', 'list'], 
                                  capture_output=True, text=True, timeout=5)
            if result.returncode == 0:
                topics = result.stdout.strip().split('\n')
                print(f"  发现 {len(topics)} 个话题:")
                for topic in topics[:10]:  # 只显示前10个
                    print(f"    - {topic}")
                if len(topics) > 10:
                    print(f"    ... 还有 {len(topics) - 10} 个话题")
            else:
                print("  ⚠️ 无法获取话题列表")
        except Exception as e:
            print(f"  ⚠️ 话题列表检查失败: {e}")
        
        # 检查目标话题信息
        target_topics = ['/ptz_manager/ptz_stable', '/sample_face/set_index']
        for topic in target_topics:
            print(f"\n🎯 检查目标话题: {topic}")
            try:
                result = subprocess.run(['ros2', 'topic', 'info', topic], 
                                      capture_output=True, text=True, timeout=5)
                if result.returncode == 0:
                    print("  ✅ 话题存在")
                    # 解析QoS信息
                    lines = result.stdout.split('\n')
                    for line in lines:
                        if 'QoS profile' in line or 'Reliability' in line or 'Durability' in line:
                            print(f"    {line.strip()}")
                else:
                    print("  ❌ 话题不存在或无法访问")
            except Exception as e:
                print(f"  ⚠️ 检查失败: {e}")
        
        # 创建诊断节点并测试
        node = QoSDiagnosticNode()
        node.test_publisher_qos()
        
        print("\n💡 建议:")
        print("  1. 如果所有配置都失败，检查ROS2环境是否正确设置")
        print("  2. 确保目标节点正在运行并监听相应话题")
        print("  3. 使用 'ros2 topic echo /sample_face/set_index' 验证消息传递")
        print("  4. 检查网络配置和ROS_DOMAIN_ID设置")
        
        # 清理
        node.cleanup()
        node.destroy_node()
        
    except KeyboardInterrupt:
        print("\n⏹️ 用户中断")
    except Exception as e:
        print(f"\n❌ 诊断过程中出错: {e}")
    finally:
        try:
            rclpy.shutdown()
        except:
            pass

if __name__ == '__main__':
    main() 