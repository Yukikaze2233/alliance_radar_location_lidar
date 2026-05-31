# RADAR-LOCATION-LIDAR

基于 **深圳市机智人科技** 纯固态面阵激光雷达 **WS_30PCD_ET3** 开发的激光雷达定位方案。

> 后续将加入基于 **Livox Mid-70** 的定位支持。

---

## Docker 环境

### 创建容器
```bash
docker run -it -d --name RADAR --privileged --restart=always \
  -v /dev:/dev \
  -v /home/pinkpanda/linux-RADAR/RADAR-2026/RADAR-LOCATION-LIDAR:/workspace \
  --network host \
  harryh_radar
```

### 命令解释

| 参数 | 说明 |
|------|------|
| `-it` | 保持 stdin 打开并分配 TTY，支持 `docker attach` 交互 |
| `-d` | 后台运行（detach mode） |
| `--name RADAR` | 容器命名为 `RADAR` |
| `--privileged` | 授予容器特权模式，允许直接访问宿主机硬件设备（LiDAR 等） |
| `--restart=always` | 容器退出后自动重启（含 Docker 守护进程重启时） |
| `-v /dev:/dev` | 挂载宿主机设备目录，使容器可直接访问 `/dev/ttyUSB*` 等串口与传感器 |
| `-v <host>:<container>` | 挂载项目工作目录到容器的 `/workspace` |
| `--network host` | 容器直接使用宿主机网络栈，无需端口映射，低延迟通信 |
| `harryh_radar` | 使用的 Docker 镜像名称 |

---

## Foxglove Bridge 配置

### 安装
```bash
sudo apt update && sudo apt install ros-$ROS_DISTRO-foxglove-bridge -y
```

### 启动
```bash
ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=8765
```

| 参数 | 说明 |
|------|------|
| `ros2 launch` | ROS2 启动工具，用于启动 launch 文件 |
| `foxglove_bridge` | Foxglove 桥接功能包名 |
| `foxglove_bridge_launch.xml` | 启动配置文件，加载桥接节点 |
| `port:=8765` | 指定 WebSocket 服务端口为 `8765`，Foxglove Studio 通过此端口连接 |

---

## WS_30PCD_ET3 开发资料

### 产品手册
- [产品手册](https://gitee.com/shenzhen-smart-people/product_manual.git)

### 驱动包
| 平台 | 地址 |
|------|------|
| ROS | https://gitee.com/shenzhen-smart-people/ws_30pcd_et3_ros.git |
| ROS2 | https://gitee.com/shenzhen-smart-people/ws_30pcd_et3_ros2.git |
| Windows (VS2022) | https://gitee.com/shenzhen-smart-people/ws_30pcd_et3_win10.git |
| Ubuntu (PCL) | https://gitee.com/shenzhen-smart-people/ws_30pcd_et3_pcl.git |
| Ubuntu (Qt) | https://gitee.com/shenzhen-smart-people/ubuntu_qt_sdk.git |

### 结构图纸
- [结构图纸](https://gitee.com/shenzhen-smart-people/structural_drawings.git)

---

## 目录结构

```
RADAR-LOCATION-LIDAR/
├── ros_ws/               # ROS 工作空间
├── ws_30pcd_ros_ws/      # WS_30PCD ROS 驱动工作空间
├── SLAM-learn/           # SLAM 学习资料
└── README.md
```
