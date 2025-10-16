# 🦾 基于 Intel RealSense D435i 与 RTAB-Map 的 Unitree Go2 建图与导航系统（ROS 2 Humble）

本项目提供了一个基于 **ROS 2 Humble + Ubuntu 22.04** 的完整工作空间，用于实现 **Unitree Go2 四足机器人** 搭载 **Intel RealSense D435i 深度相机** 的实时 **RGB-D SLAM 建图与定位系统**。  

系统通过对 **深度相机与机器人内置里程计的时间同步** 与 **TF 坐标配置优化**，成功实现了在复杂环境下的 **稳定建图与精确定位**。

---

## 🚀 一、项目简介

本项目的主要目标是让 **Unitree Go2** 机器人能够利用 **Intel RealSense D435i** 相机进行 **实时环境感知与地图构建**。  
系统成功解决了 RTAB-Map 常见的“相机数据与里程计时间戳不匹配”问题，实现了流畅的 SLAM 映射过程。  

经过优化的 TF 树、可靠的 QoS 配置以及多源数据同步机制，使机器人能在实际环境中进行连续、稳定的建图和导航。

---

## 🧩 二、功能特点

- ✅ 支持 D435i 的 **RGB-D + IMU** 数据流  
- ✅ 实时 **2D/3D 环境建图**（RTAB-Map）  
- ✅ **相机与 Unitree Go2 里程计数据同步**  
- ✅ 完整的 **TF 坐标树**（`map → odom → base_link → camera_link`）  
- ✅ 可靠的 **QoS 与时间同步机制**  
- ✅ 可视化与地图保存（RTAB-Map Viz + RViz2）  
- ✅ 支持 **Unitree Lite3 / Go2 实机验证**

---

## ⚙️ 三、系统环境

| 组件 | 版本 |
|------|------|
| 操作系统 | Ubuntu 22.04 |
| ROS 2 发行版 | Humble |
| RTAB-Map | ≥ 0.21 |
| RealSense SDK | ≥ 2.55 |
| 相机型号 | Intel RealSense D435i |
| 机器人平台 | Unitree Lite3 / Go2 |
| GPU（可选） | NVIDIA RTX 系列 |

---

## 🧠 四、工作空间构建

```bash
# 1. 克隆仓库
mkdir -p ~/rtabmap_ws/src
cd ~/rtabmap_ws/src
git clone https://github.com/L-winder2002/Unitree-Go2-Mapping-and-Navigation-Using-Intel-RealSense-D435i-and-RTAB-Map.git

# 2. 安装依赖
cd ~/rtabmap_ws
rosdep update
rosdep install --from-paths src --ignore-src -y

# 3. 编译
colcon build --symlink-install
source install/setup.bash


---

## 🔧 五、系统启动与使用

```bash
# 1. 克隆仓库
mkdir -p ~/rtabmap_ws/src
cd ~/rtabmap_ws/src
git clone https://github.com/L-winder2002/Unitree-Go2-Mapping-and-Navigation-Using-Intel-RealSense-D435i-and-RTAB-Map.git

# 2. 安装依赖
cd ~/rtabmap_ws
rosdep update
rosdep install --from-paths src --ignore-src -y

# 3. 编译
colcon build --symlink-install
source install/setup.bash