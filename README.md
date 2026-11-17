# AeroSight-ROS2

在 RViz2 中可视化宇树 **Unitree G1（29 自由度）** 人形机器人，并运行走路、招手、舞蹈等关节空间 / 笛卡尔空间演示 demo，无需真机。
Visualise the Unitree G1 29-DOF humanoid in RViz2 (ROS 2 Humble) and drive it with Python demos — no real robot needed.

## 快速开始 / Quick start (Docker, macOS / Linux / Windows)

```bash
docker compose up -d --build
```
浏览器打开 / open <http://localhost:6081/vnc.html?autoconnect=1&resize=scale> 即可看到 RViz2（容器内软件渲染，通过 noVNC 显示）。

在另一个终端运行 demo / run a demo:

```bash
scripts/run_demo.sh joint_control      # 交互: stand | wave | walk | set <joint> <deg> | list | quit
scripts/run_demo.sh dance_basic        # 4 段自动舞蹈
scripts/run_demo.sh dance_beat         # 124 BPM 节拍舞蹈
scripts/run_demo.sh cartesian --left 0.35 0.25 0.05 -90 0 0   # 手部笛卡尔位姿 (FK + 数值 IK)
```

- demo 会先停掉默认的零位 `joint_state_publisher`（否则两者争抢 `/joint_states`）。恢复：`docker compose restart`。
- 想用滑块调关节：`docker compose run --rm -p 6081:6080 g1 --gui`
- 停止：`docker compose down`。宿主机端口 6081 被占用时，修改 `docker-compose.yml`。

## 原生 Ubuntu 22.04 / Native Ubuntu 22.04

```bash
sudo apt install ros-humble-desktop ros-humble-joint-state-publisher-gui python3-pykdl
scripts/start.sh            # robot_state_publisher + TF + RViz2  (加 --gui 使用滑块)
source /opt/ros/humble/setup.bash
/usr/bin/python3 -m g1_demos.joint_control     # 必须用系统 Python 3.10 / system Python 3.10 required
```

## 目录结构 / Layout

```
config/g1_robot.rviz        RViz2 配置 (Fixed Frame=world, RobotModel=/robot_description)
description/                G1 29-DOF URDF 模板 (g1_29dof.urdf.in) + meshes/
g1_demos/
  common.py                 关节列表、/joint_states 发布节点、插值/播放工具、运行封装
  joint_control.py          交互式关节控制 (wave / walk / set)
  dance_basic.py            基础舞蹈 demo
  dance_beat.py             节拍舞蹈 demo
  cartesian.py              笛卡尔位姿控制 (PyKDL FK + 数值 IK)
scripts/start.sh            启动 robot_state_publisher / TF / RViz2（渲染 URDF 中的 mesh 路径）
scripts/run_demo.sh         在容器内运行 demo
docker/                     Dockerfile + 入口脚本 (Xvfb + x11vnc + noVNC)
docs/                       原始搭建指南 (PDF)
```

## 说明 / Notes
- `start.sh` 启动时把 `description/g1_29dof.urdf.in` 中的 `@MESH_DIR@` 替换为本仓库 mesh 绝对路径，因此无需 colcon 构建，也没有硬编码用户目录。
- 控制的是 `/joint_states`（正运动学）；TF 为 `map → world → pelvis (z=0.75)`。
- G1 URDF 与 meshes 来自 Unitree / [g1pilot](https://github.com/hucebot/g1pilot)，遵循其 BSD-3-Clause 许可（见 `LICENSE`）。
