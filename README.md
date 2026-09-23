# Wheeltec 4WD 豬舍模擬與 LiDAR 控制

本專案提供 ROS 2 Humble／Gazebo Ignition Fortress 豬舍模擬場景、四輪差速機器人模型，以及 LiDAR 傳統走道控制程式。

目前可使用 `scripts/launch_clean.sh` 開啟 Gazebo，再獨立執行 `rl_pig_pen/fsm_patrol.py` 進行走道直行。控制器僅讀取 `/scan` 並發布 `/cmd_vel`，不依賴底盤 odom／IMU，也不需要載入模型。

新增獨立入口 `rl_pig_pen/fsm_lidar_patrol.py`，以傳統控制完成左支線前進到底、倒退穿過路口至右端、前進回中心、轉回主幹道並繼續下一個路口的循環，直到主幹道終點。原本直行腳本維持不變。

完成設定的 3 個路口後，進入 `FINAL_MAIN`，只沿主幹道行駛並確認終點牆，不再接受第四個支線入口；確認終點後才停止並回報 `DONE`。

## 目前功能範圍

| 功能 | 狀態 |
| --- | --- |
| Gazebo 世界、車子生成、LiDAR 與 ROS 橋接 | 保留啟動流程 |
| 相機、深度點雲、RViz 與豬隻行為 | 保留相關腳本，依啟動選項載入 |
| LiDAR 走道平行修正、左右置中、前方過近停車 | 現有直行控制器 |
| 完整支線巡邏、倒車、路口轉 90° | 新增獨立傳統控制入口，已通過靜態雷達／理想運動路線測試，待 Gazebo 物理驗證 |
| 舊 PPO／BC 訓練與模型測試 | 程式已移除，不再提供可用入口 |
| 傳統控制＋RL 混合架構 | 尚未實作 |

`fsm_patrol.py` 的現有類別是 `PigPenStraightLineController`；檔名不代表它已能完成整條巡邏路線。更詳細的控制參數與限制請見 [LiDAR 控制說明](rl_pig_pen/README.md)。

## 快速開始

以下以專案位置 `/home/an/Desktop/4wd` 為例。這些步驟啟動的是模擬環境，不會啟動實體底盤串口程式。

### 1. 啟動 Gazebo

終端 1：

```bash
cd /home/an/Desktop/4wd
bash scripts/launch_clean.sh
```

預設開啟 Gazebo GUI，並啟動相機相關橋接與可用的 RViz／點雲節點；**不會自動啟動行車控制器**。

只需要雷達時，可改用以下指令，仍會開啟 Gazebo 視窗：

```bash
bash scripts/launch_clean.sh --lidar-only
```

上述啟動方式擇一使用。其他選項：

| 選項 | 說明 |
| --- | --- |
| `--lidar-only` | 建立停用相機的暫存場景／車型，略過相機、點雲及 RViz 啟動流程 |
| `--headless` | 不開 Gazebo GUI，亦略過相機橋接及 RViz；不等同於移除車型中的相機 |
| `--rtf 1` | 設定目標模擬速度倍率；實際速度受負載限制 |
| `--env-id N` | 設定 ROS domain（0～100）及 Gazebo 通訊隔離 |
| 世界檔路徑 | 替換預設 world，例如 `bash scripts/launch_clean.sh worlds/pig_pen_16units.world` |

若使用 `--env-id N`，其他控制／診斷終端也必須設定 `export ROS_DOMAIN_ID=N`。

### 2. 啟動 LiDAR 直行控制

等模擬與雷達就緒後，在終端 2 執行：

```bash
cd /home/an/Desktop/4wd
source /opt/ros/humble/setup.bash
/usr/bin/python3 rl_pig_pen/fsm_patrol.py
```

收到第一筆掃描後，車子便會開始控制。預設巡航速度為 0.2 m/s，直行修正角速度上限為 ±0.3 rad/s；前方 ±30° 內的最短距離小於 0.55 m 時停車，障礙離開後會恢復前進。

同一台車只應有一個行車控制器發布 `/cmd_vel`。不要同時執行 FSM、鍵盤遙控或速度測試。

停止時先在控制器終端按 `Ctrl-C`，再停止 `launch_clean.sh`。啟動腳本會清理自己建立的模擬程序。

### 3. 手動控制（取代 FSM）

先停止直行控制器，再於已載入 ROS 環境的終端執行：

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p speed:=0.2 -p turn:=0.3
```

鍵盤焦點需放在該終端；`i` 前進、`,` 後退、`j/l` 左右轉、`k` 停車。

### 4. 完整傳統巡邏（取代直行／手動控制）

從預設主幹道起點測試，建議 Gazebo 使用 `--lidar-only --rtf 1` 啟動。先停止其他行車控制器，再在另一個終端執行：

```bash
cd /home/an/Desktop/4wd
source /opt/ros/humble/setup.bash
/usr/bin/python3 rl_pig_pen/fsm_lidar_patrol.py
```

控制器會顯示目前狀態、已完成路口及支線端點數。轉彎和路口置中使用 LiDAR 幾何完成判斷；感測中斷或追蹤異常會鎖定停車。詳細流程與測試範圍見 [完整巡邏說明](rl_pig_pen/README.md)。

## 執行環境與建置

現有啟動流程以 Ubuntu 22.04、ROS 2 Humble、Gazebo Ignition Fortress 6 為基礎。執行前應備妥：

- `ign gazebo`、`ros_gz_sim`、`ros_gz_bridge`。
- `robot_state_publisher`、`tf2_ros`，以及 Python 的 `rclpy`、NumPy、ROS 訊息套件。
- 完整相機模式另需 RViz、`depth_image_proc`，以及視覺脚本使用的 OpenCV／ArUco 與 `cv_bridge`。
- 手動控制另需 `teleop_twist_keyboard`。

目前直行控制不需要 PyTorch、Gymnasium 或 Stable-Baselines3。若 Python 找不到 ROS 套件，先載入 `/opt/ros/humble/setup.bash`，並使用 `/usr/bin/python3`。

需要建置本地 Wheeltec ROS 2 套件時，可在相依套件已備妥後執行：

```bash
cd /home/an/Desktop/4wd
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to turn_on_wheeltec_robot
source install/setup.bash
```

`src/transport_drivers/` 已存在，不要再次 clone 覆蓋。建置底盤套件不代表需要在模擬時啟動實體串口節點。

啟動腳本目前包含 NVIDIA EGL／GLX 環境設定；非 NVIDIA 主機或不同顯示環境需要另行確認相容性。此文件不代表已完成 Xavier 或實車驗證。

## 場景、車型與通訊

| 項目 | 目前設定 |
| --- | --- |
| 預設世界 | [worlds/pig_pen_16units.world](worlds/pig_pen_16units.world) |
| 車型 | [four_wheel_diff_bs_robot.urdf](turn_on_wheeltec_robot/urdf/four_wheel_diff_bs_robot.urdf) |
| Gazebo 模型名稱 | `wheeltec_mini` |
| 出生位置 | `x=0, y=-11.3, z=0.05, yaw=1.5708`，由 `launch_clean.sh` 設定 |
| 雷達 | `/scan`，`sensor_msgs/msg/LaserScan` |
| 行車指令 | `/cmd_vel`，`geometry_msgs/msg/Twist` |
| 模擬里程計 | 橋接至 `/odom`，但目前直行控制器不訂閱 |
| 相機 | `/camera/depth/image_raw`、`/camera/color/image_raw`、`/camera_right/image_raw`，依啟動模式橋接 |

世界與豬隻資源分別位於 `worlds/` 與 `pig_model/`。若專案搬到其他位置，啟動前可用 `PIG_MODEL_PKG_DIR` 指向實際的 `pig_model` 目錄；腳本的預設值是 `$HOME/Desktop/4wd/pig_model`。

## 目錄與常用工具

| 路徑 | 用途 |
| --- | --- |
| [scripts/launch_clean.sh](scripts/launch_clean.sh) | 模擬主入口 |
| [scripts/spawn_robot.sh](scripts/spawn_robot.sh) | 生成機器人 |
| [scripts/prepare_training_scene.py](scripts/prepare_training_scene.py) | 為 `--rtf`／`--lidar-only` 產生暫存世界與車型；仍是啟動流程的相依程式 |
| [scripts/pig_behavior.py](scripts/pig_behavior.py) | 豬隻行為控制 |
| [scripts/aruco_face_detector.py](scripts/aruco_face_detector.py) | ArUco 視覺辨識 |
| [scripts/pointcloud_qos_relay.py](scripts/pointcloud_qos_relay.py) | 深度點雲 QoS 轉接 |
| [scripts/control_cmdvel.py](scripts/control_cmdvel.py) | 手動指定速度與持續時間的測試工具，會讓車子移動 |
| [scripts/cmdvel_to_stm32.py](scripts/cmdvel_to_stm32.py) | 實體底盤串口速度轉接，不由模擬入口啟動 |
| [rl_pig_pen/](rl_pig_pen/README.md) | 獨立直行與巡邏控制、幾何估計、設定及測試 |
| `turn_on_wheeltec_robot/` | ROS 2 底盤套件、URDF 與模型資源 |
| `turn_on_wheeltec_robot_ros1_backup/` | ROS 1 備份，不是目前 ROS 2 執行入口 |
| `src/transport_drivers/` | 傳輸／串口相依套件 |
| [zed_description/](zed_description/README.md) | ZED 相機描述資源 |
| `rviz/` | RViz 設定 |
| `logs/` | 模擬啟動日誌 |
| `build/`、`install/`、`log/` | colcon 建置產物與建置日誌 |
| `車子資料/`、根目錄 PDF | 硬體與底盤參考資料 |

`scripts/launch_rl_training.sh` 雖然仍保留，但引用的舊 PPO 程式已刪除，**目前不能用來訓練**。舊 checkpoint、示範資料或日誌即使仍在磁碟，也不會被現有直行控制器載入。

## 診斷與測試

在模擬運行時，另開終端：

```bash
cd /home/an/Desktop/4wd
source /opt/ros/humble/setup.bash
ros2 topic list
ros2 topic hz /scan
```

停止頻率檢查後，可執行只讀取感測資料、不發布速度的診斷：

```bash
/usr/bin/python3 rl_pig_pen/check_lidar.py
```

啟動失敗或缺少資料時，優先檢查：

- `logs/gz_sim_server.log`：世界、資源與感測器載入。
- `logs/gz_sim_gui.log`：Gazebo 視窗與顯示問題。
- `logs/spawn.log`：機器人生成。
- `logs/ros_gz_bridge.log`：ROS／Gazebo 通訊。

使用 `--env-id N` 時，上述日誌位於 `logs/env_N/`。LiDAR 全為無效值可能涉及渲染、資源或感測器設定，不應只憑症狀認定單一原因。結束模擬請優先使用啟動終端的 `Ctrl-C`，避免廣泛終止其他 ROS／Gazebo 工作。

不啟動 Gazebo 的靜態檢查：

```bash
bash -n scripts/launch_clean.sh scripts/spawn_robot.sh
source /opt/ros/humble/setup.bash
OPENBLAS_NUM_THREADS=1 /usr/bin/python3 -B -m unittest discover -s rl_pig_pen/tests -v
```

測試涵蓋場景準備、靜態雷達下的完整路線、初始偏移及感測異常停止；理想運動測試不代表已通過 Gazebo 物理或實體車驗收。

## 安全限制與後續方向

原本直行控制器尚未具備掃描過期停車、低可信度停車、完整車體碰撞檢查及路線完成判斷。新巡邏入口另加入感測失聯、追蹤異常與障礙淨空保護，但仍需實測；目前也沒有避障繞行功能。模擬啟動中的靜態 `odom → base_link` TF 不等同於移動中的定位結果，尚未提供完整驗證的 SLAM／Nav2 啟動流程。

實體部署時，LiDAR 驅動、速度介面、指令逾時停車與急停需要分別確認。現有串口轉接腳本會重送最後收到的速度，不能假設上層失聯便會自動停車；本次整理並未修改底盤程式。

後續優先以 Gazebo 驗證完整傳統巡邏在滑動、延遲與動態障礙下的表現，再評估是否需要 RL 處理特定轉彎任務。
