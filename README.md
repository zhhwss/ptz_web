# PTZ 控制系统

一个基于Flask的网页服务器，用于控制PTZ设备和浏览实时图片。

## 功能特性

### 1. 图片浏览 (`/images`)
- 实时浏览 `~/ptz_ws/images` 目录中的图片
- 自动刷新功能（每5秒）
- 图片预览和详细信息显示
- 响应式网格布局，支持手机和电脑访问

### 2. 控制面板 (`/control`)
- **脚本执行**: 通过网页界面执行 `~/ptz_ws/pub_sample.sh` 脚本
- **参数选择**: 可以从网页上选择和修改脚本中的样本参数
- **ROS话题监听**: 实时监听 `/ptz_manager/ptz_stable` 话题
- **执行日志**: 显示所有操作的详细日志

### 3. 实时通信
- 使用WebSocket实现实时数据更新
- ROS话题状态实时推送到网页
- 无需手动刷新即可看到最新状态

## 安装和运行

### 1. 安装依赖
```bash
pip install -r requirements.txt
```

### 2. 启动服务器
```bash
python app.py
```

### 3. 访问网页
服务器启动后，可以通过以下地址访问：

- **主页**: http://your-server-ip:5000/
- **图片浏览**: http://your-server-ip:5000/images
- **控制面板**: http://your-server-ip:5000/control

## 系统要求

- Python 3.7+
- ROS2 环境 (需要 `/opt/tros/humble/setup.bash`)
- 网络访问权限

## 配置说明

### 路径配置
在 `app.py` 中可以修改以下路径：

```python
IMAGES_DIR = "/home/sunrise/ptz_ws/images"  # 图片目录
SCRIPT_PATH = "/home/sunrise/ptz_ws/pub_sample.sh"  # 脚本路径
```

### 样本选项
系统会自动从脚本中提取可用的样本选项，包括：

- Close-Up_Standing_Back_Turned_Looking_Back_Both_Hands_Raised
- Long_Shot_Standing_Back_Turned_Looking_Back_Hands_Behind_Back
- Medium_Close-Up_Standing_Front_Facing_Head_Tilted_Up_Hands_in_Pockets
- 等等...

### ROS话题监听
系统监听 `/ptz_manager/ptz_stable` 话题，每2秒检查一次新数据。

## 移动端支持

网页采用响应式设计，完全支持手机和平板电脑访问：

- 自适应布局
- 触摸友好的界面
- 移动端优化的导航

## 安全注意事项

- 服务器绑定到 `0.0.0.0:5000`，可以从网络中的任何设备访问
- 建议在生产环境中配置防火墙和访问控制
- 脚本执行具有系统权限，请确保网络安全

## 故障排除

### 1. ROS话题无法监听
- 确保ROS2环境已正确配置
- 检查话题 `/ptz_manager/ptz_stable` 是否存在
- 验证 `/opt/tros/humble/setup.bash` 路径是否正确

### 2. 图片无法显示
- 检查图片目录权限
- 确保图片格式为 jpg, jpeg, png, gif
- 验证图片目录路径是否正确

### 3. 脚本执行失败
- 检查脚本文件权限
- 确保ROS2环境变量已设置
- 查看执行日志获取详细错误信息

## 开发说明

### 项目结构
```
.
├── app.py              # 主应用文件
├── requirements.txt    # Python依赖
├── README.md          # 说明文档
└── templates/         # HTML模板
    ├── base.html      # 基础模板
    ├── index.html     # 主页
    ├── images.html    # 图片浏览页面
    └── control.html   # 控制面板页面
```

### API接口
- `GET /api/images` - 获取图片列表
- `POST /api/execute_script` - 执行脚本
- `GET /api/ptz_status` - 获取PTZ状态
- `GET /images/<filename>` - 获取图片文件

### WebSocket事件
- `ptz_status_update` - PTZ状态更新事件 # ptz_web
