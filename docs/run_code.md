# Lightning-LM 运行指南

## 基本信息

- 代码地址: `/home/jerett/OpenProject/LidarSlam/lightning-lm`
- 数据地址: `/home/jerett/Data/yunshenchu_build3`
- 数据内容: 云深处 building3 数据集 (db3 格式)

## 1. 安装依赖

```bash
sudo apt install libopencv-dev libpcl-dev pcl-tools libyaml-cpp-dev libgoogle-glog-dev libgflags-dev ros-humble-pcl-conversions
```

## 2. 编译

```bash
cd /home/jerett/OpenProject/LidarSlam/lightning-lm
colcon build --packages-select lightning
source install/setup.bash
```

## 3. 建图

云深处数据使用 Livox 雷达（CustomMsg 格式），对应配置文件为 `config/default_yunshenchu.yaml`（lidar_type=1）。

### 离线建图（推荐）

```bash
cd /home/jerett/OpenProject/LidarSlam/lightning-lm
source install/setup.bash
ros2 run lightning run_slam_offline \
  --config ./config/default_yunshenchu.yaml \
  --input_bag /home/jerett/Data/yunshenchu_build3/building3_0.db3
```

建图结束后，地图自动保存至 `data/new_map/` 目录。

### 在线建图

```bash
cd /home/jerett/OpenProject/LidarSlam/lightning-lm
source install/setup.bash
ros2 run lightning run_slam_online --config ./config/default_yunshenchu.yaml
```

然后在另一个终端播包：

```bash
ros2 bag play /home/jerett/Data/yunshenchu_build3/building3_0.db3
```

建图完成后保存地图：

```bash
ros2 service call /lightning/save_map lightning/srv/SaveMap "{map_id: new_map}"
```

## 4. 查看地图

```bash
pcl_viewer ./data/new_map/global.pcd
```

- `global.pcd` 为完整点云地图（仅用于显示）
- 实际地图分块存储
- `map.pgm` 为 2D 栅格地图

## 5. 定位

### 离线定位

确认 `config/default_yunshenchu.yaml` 中 `system.map_path` 指向建图输出的地图目录（默认 `./data/new_map/`），然后：

```bash
cd /home/jerett/OpenProject/LidarSlam/lightning-lm
source install/setup.bash
ros2 run lightning run_loc_offline \
  --config ./config/default_yunshenchu.yaml \
  --input_bag /home/jerett/Data/yunshenchu_build3/building3_0.db3
```

### 在线定位

```bash
cd /home/jerett/OpenProject/LidarSlam/lightning-lm
source install/setup.bash
ros2 run lightning run_loc_online --config ./config/default_yunshenchu.yaml
```

然后在另一个终端播包：

```bash
ros2 bag play /home/jerett/Data/yunshenchu_build3/building3_0.db3
```

定位结果通过 TF 话题输出，频率与 IMU 同频（50-100Hz）。

## 配置说明

`config/default_yunshenchu.yaml` 中与云深处数据相关的关键配置：

| 配置项 | 值 | 说明 |
|--------|-----|------|
| `fasterlio.lidar_type` | 1 | Livox 雷达（CustomMsg 格式） |
| `common.lidar_topic` | `/livox/lidar` | 点云话题 |
| `common.imu_topic` | `/livox/imu` | IMU 话题 |
| `loop_closing.with_height` | true | 高度约束（building3 为多层，已适配） |
| `fasterlio.imu_filter` | true | IMU 滤波（建议开启） |
