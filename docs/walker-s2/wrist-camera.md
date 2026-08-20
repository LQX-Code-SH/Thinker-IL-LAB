# Walker S2 EDU 探索者：腕部相机（RealSense D405）

> `ubt_IL/walker/realsense_wrist_camera/`：Intel RealSense D405 腕部相机 ROS2 publisher，为 Walker S2 EDU 探索者 真机部署提供腕部视觉输入。

## 快速开始

```bash
# 1. 安装（自动发现相机、生成配置、持久化 PATH）
bash /ubt_IL/walker/realsense_wrist_camera/scripts/install.sh

# 2. 启动（默认后台运行）
bash /ubt_IL/walker/realsense_wrist_camera/scripts/start.sh

# 3. 查看状态 / 停止
bash /ubt_IL/walker/realsense_wrist_camera/scripts/start.sh --status
bash /ubt_IL/walker/realsense_wrist_camera/scripts/start.sh --stop
```

调试时前台运行：`bash start.sh --fg`；其余参数原样透传给 CLI（如 `bash start.sh --fg --discover`）。

## 验证发布

```bash
ros2 topic echo /sensor/camera/wrist_right/color/raw --once --qos-reliability best_effort
```

## 配置

由 `install.sh` 自动生成（`configs/wrist_cameras.json`）；重新生成用：

```bash
find-realsense-cameras
```

手动指定配置启动：

```bash
realsense-wrist-camera --config /path/to/cameras.json &
```

配置格式（左右腕各一台 D405，`shm_msgs/Image1m`，640×480@15fps）：

```json
{
  "cameras": [
    {
      "serial": "261122273724",
      "topic": "/sensor/camera/wrist_right/color/raw",
      "msg_type": "shm_msgs/Image1m",
      "frame_id": "wrist_right",
      "width": 640,
      "height": 480,
      "fps": 15
    },
    {
      "serial": "261022273252",
      "topic": "/sensor/camera/wrist_left/color/raw",
      "msg_type": "shm_msgs/Image1m",
      "frame_id": "wrist_left",
      "width": 640,
      "height": 480,
      "fps": 15
    }
  ]
}
```

## 与真机部署的集成

腕部相机在标准话题上发布 `shm_msgs/Image1m`，`walker_camera_relay.py` 会自动接管，**rollout 无需任何额外配置**：

```bash
# 终端 1
bash /ubt_IL/walker/realsense_wrist_camera/scripts/start.sh

# 终端 2 - 正常 rollout
POLICY_PATH=/ubt_IL/model/<policy>/checkpoints/080000/pretrained_model \
bash /ubt_IL/scripts/deploy/walker_s2/rollout.sh
```

## 安装准备

**udev 规则**（宿主机，一次性）：

```bash
sudo sh -c 'echo "SUBSYSTEM==\"usb\", ATTR{idVendor}==\"8086\", ATTR{idProduct}==\"0b5b\", MODE=\"0666\"" > /etc/udev/rules.d/99-realsense-d405.rules'
sudo udevadm control --reload-rules
sudo udevadm trigger
```

**Docker**：容器必须以 `--privileged` 运行（项目 `run.sh` 已包含）。

## 常见问题

| 现象 | 处理 |
|------|------|
| `No RealSense devices detected` | 检查 USB 连接与 udev 规则 |
| `ModuleNotFoundError: pyrealsense2` | 重新执行 `install.sh` |
| `ModuleNotFoundError: rclpy` | `source /opt/ros/humble/setup.bash` |
| USB `Permission denied` | Docker 缺 `--privileged` |
