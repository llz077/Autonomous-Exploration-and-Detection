# Alert Player

This package subscribes to a std_msgs/String topic and plays an alert when any of the following trigger phrases are received.

* intruder
* closed room
* started
* stopped



---
## 📦 Install Dependencies

Ensure all necessary system packages are installed:

```bash
pip install playsound
```

---
## 🚀 Clone and Build the Packages

1. Copy the `alert_player` package to your ROS workspace (e.g., `~/ros2_ws/src`):
```bash
cp -r alert_player ~/ros2_ws/src/
```

2. Build the packages & Source the workspace:
```bash
cd ~/ros2_ws
colcon build --symlink-install --packages-up-to alert_player
source ~/.bashrc
```

---
## 📍Launch the Package

The package can be launched using ros2 run or the package launch file.
```bash
ros2 launch alert_player alert_player.launch.py
```

### 📎 Send an alert trigger

1. Open a new terminal and enter:
```bash
ros2 topic pub -1 /alert std_msgs/msg/String data:\ 'intruder'
```

You should hear "Intruder" alert being played! Make sure your audio is not muted!