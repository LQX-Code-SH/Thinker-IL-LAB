# Walker S2 EDU: Wrist Camera (RealSense D405)

> `ubt_IL/walker/realsense_wrist_camera/`: an Intel RealSense D405 wrist camera ROS 2 publisher that provides wrist vision input for Walker S2 EDU real-robot deployment.

## Quick start

```bash
# 1. Install (auto-discover cameras, generate config, persist PATH)
bash /ubt_IL/walker/realsense_wrist_camera/scripts/install.sh

# 2. Start (runs in the background by default)
bash /ubt_IL/walker/realsense_wrist_camera/scripts/start.sh

# 3. Check status / stop
bash /ubt_IL/walker/realsense_wrist_camera/scripts/start.sh --status
bash /ubt_IL/walker/realsense_wrist_camera/scripts/start.sh --stop
```

For debugging, run in the foreground with `bash start.sh --fg`; other arguments are passed through to the CLI (e.g. `bash start.sh --fg --discover`).

## Verify publishing

```bash
ros2 topic echo /sensor/camera/wrist_right/color/raw --once --qos-reliability best_effort
```

## Configuration

`install.sh` generates `configs/wrist_cameras.json` automatically; regenerate it with:

```bash
find-realsense-cameras
```

Start with an explicit config:

```bash
realsense-wrist-camera --config /path/to/cameras.json &
```

Config format (one D405 per wrist, `shm_msgs/Image1m`, 640×480@15fps):

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

## Integration with real-robot deployment

The wrist cameras publish `shm_msgs/Image1m` on standard topics; `walker_camera_relay.py` picks them up automatically, so **rollout needs no extra configuration**:

```bash
# Terminal 1
bash /ubt_IL/walker/realsense_wrist_camera/scripts/start.sh

# Terminal 2 - normal rollout
POLICY_PATH=/ubt_IL/model/<policy>/checkpoints/080000/pretrained_model \
bash /ubt_IL/scripts/deploy/walker_s2/rollout.sh
```

## Installation prerequisites

**udev rules** (host, one time):

```bash
sudo sh -c 'echo "SUBSYSTEM==\"usb\", ATTR{idVendor}==\"8086\", ATTR{idProduct}==\"0b5b\", MODE=\"0666\"" > /etc/udev/rules.d/99-realsense-d405.rules'
sudo udevadm control --reload-rules
sudo udevadm trigger
```

**Docker**: the container must run with `--privileged` (already included in the project `run.sh`).

## FAQ

| Symptom | Fix |
|------|------|
| `No RealSense devices detected` | Check the USB connection and udev rules |
| `ModuleNotFoundError: pyrealsense2` | Rerun `install.sh` |
| `ModuleNotFoundError: rclpy` | `source /opt/ros/humble/setup.bash` |
| USB `Permission denied` | Docker is missing `--privileged` |
