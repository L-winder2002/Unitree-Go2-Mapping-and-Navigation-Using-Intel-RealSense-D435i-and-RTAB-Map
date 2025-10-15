# 🐕 基于 Intel RealSense D435i 与 RTAB-Map 的四足机器人建图系统（ROS 2 Humble）

本仓库提供了一个基于 **ROS 2 Humble + Ubuntu 22.04** 的完整工作空间，用于实现四足机器人（Unitree Go / Lite3）使用 **Intel RealSense D435i 深度相机** 进行实时 **RGB-D SLAM 建图与定位**。  

系统集成了：
- 🎥 **RealSense D435i 相机驱动**（RGB + Depth + IMU）
- 🧠 **RTAB-Map 实时建图算法**
- 🧭 **TF 坐标树与里程计同步配置**
- 🗺️ **可视化与地图保存**

---

## 🧩 一、功能特点

- ✅ 支持 D435i 的 RGB-D + IMU 数据流  
- ✅ 实时 2D/3D 环境建图（RTAB-Map）  
- ✅ 与四足机器人（Unitree）兼容  
- ✅ 支持可靠的 QoS、TF 树与时间同步配置  
- ✅ 可扩展深度图 → 占据栅格自定义节点  

---

## ⚙️ 二、系统环境

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

## 🧠 三、工作空间构建

```bash
# 1. 克隆仓库
mkdir -p ~/rtabmap_ws/src
cd ~/rtabmap_ws/src
git clone https://github.com/L-winder2002/rtabmap_ws.git

# 2. 安装依赖
cd ~/rtabmap_ws
rosdep update
rosdep install --from-paths src --ignore-src -y

# 3. 编译
colcon build --symlink-install
source install/setup.bash