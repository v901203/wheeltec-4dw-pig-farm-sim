# Wheeltec 4WD 豬舍 ROS 2 / Gazebo 模擬專案

本專案基於 **ROS 2 Humble** 與 **Gazebo Ignition Fortress 6**，專為四輪驅動 (4WD) 巡檢機器人在豬舍環境中的感測器融合、深度學習（強化學習 RL / 行為複製 BC）、SLAM 建圖與自主導航所設計。支援 Gazebo 模擬環境與實體 Wheeltec STM32 底盤雙軌運行。

---

## 目錄

1. [網格與場景規格](#1-網格與場景規格-pig-pen-world)
2. [機器人硬體與感測器配置](#2-機器人硬體與感測器配置)
3. [環境安裝與建置](#3-環境安裝與建置-prerequisites--build)
4. [快速開始](#4-快速開始-quick-start)
5. [強化學習訓練 (RL Pipeline)](#5-強化學習訓練-rl-pipeline)
6. [行為複製 (Behavioral Cloning)](#6-行為複製-behavioral-cloning)
7. [SLAM 建圖與自主導航](#7-slam-建圖與自主導航)
8. [常用工具與轉接腳本](#8-常用工具與轉接腳本-scripts)
9. [串口控制協議 (STM32 實體底盤)](#9-串口控制協議-stm32-實體底盤)
10. [外部依賴與 Git 子模組](#10-外部依賴與-git-子模組-zed-description)
11. [移植紀錄 (ROS1 → ROS2 / Fortress)](#11-移植紀錄-ros1--ros2--fortress)
12. [常見問題與除錯](#12-常見問題與除錯-troubleshooting)

---

## 1. 網格與場景規格 (Pig Pen World)

目前預設使用的豬舍世界檔為 **`worlds/pig_pen_16units.world`**（可依需求替換為 `worlds/pig_pen_8units(lv4).world`），包含 **16 間單獨豬舍** 與 **16 隻帶有 ArUco 視覺標記的假豬模型**：

### 豬舍結構規格
* **單間尺寸**：長 3.30m × 寬 2.55m 長方形。
* **四面欄杆牆**：
  * **長邊牆 (`left_wall` / `right_wall`)**：3.30m，各含 32 根圓柱欄杆。
  * **短邊牆 (`front_wall` / `back_wall`)**：2.55m，各含 24 根圓柱欄杆。
  * **欄杆規格**：總高 83cm、直徑 1.5cm、中心間距 10cm（淨空隙 8.5cm）。
  * **高度校正**：下橫桿 0.035m、欄杆中心 0.415m、上橫桿 0.795m，完全貼合地面無懸空。

### 長邊相鄰佈局 (Long-Side Adjacent)
* **左排 (X = -2.4m)**：`pen1` ～ `pen8`，沿 Y 軸長邊相鄰排列（間距 4cm）。
* **右排 (X = +2.4m)**：`pen9` ～ `pen16`，沿 Y 軸長邊相鄰排列（間距 4cm）。
* **中央巡檢走道**：兩排之間預留 **1.5m 寬走道**（沿 Y 軸延伸，X 範圍約在 -0.75m ～ +0.75m）。
* **走道支線（十字路口）**：豬欄排列形成 3 個十字形交叉路口，RL 巡邏路線需進入每條支線。

### 豬隻與 ArUco 視覺標記
* 每間豬舍配備 1 隻假豬模型 (`farm_pig`)，放置於飼料盆旁（偏移 0.7m 避開碰撞體）。
* 豬隻前端帶有 **ArUco Marker 貼圖**，統一朝向中央走道：
  * **左排 (`pig1` ～ `pig8`)**：ArUco 面朝向 +X 方向。
  * **右排 (`pig9` ～ `pig16`)**：ArUco 面旋轉 180 度朝向 -X 方向。

---

## 2. 機器人硬體與感測器配置

底盤基於 **Wheeltec 4WD 差速車 (`wheeltec_mini`)**，URDF 檔位於 `turn_on_wheeltec_robot/urdf/four_wheel_diff_bs_robot.urdf`：

* **底盤尺寸**：0.40m × 0.30m × 0.065m，離地高度 4.5cm，輪胎直徑 15cm（半徑 7.5cm）。
* **預設出生點**：`X=0.0, Y=-14.0, Z=0.05`，朝向 Y 軸正方向（`yaw = 1.5708`），位於中央走道右端起點（面向巡邏方向）。
* **感測器裝備**：
  1. **2D LiDAR (M10P-PHY)**：Topic `/scan`，位於 Z 軸 0.245m 處，360° 掃描，最大量測距離 25m。
  2. **前方深度相機 (ZED X)**：
     * 深度圖 Topic：`/camera/depth/image_raw`
     * 彩色圖 Topic：`/camera/color/image_raw`
  3. **右側監視相機 (Right Camera)**：
     * 彩色圖 Topic：`/camera_right/image_raw`（朝右側拍攝豬欄）

> **注意（RTX 5090）**：Ubuntu 22.04 內建的 Mesa 不支援 Blackwell 架構 GPU（PCI ID `0x7d67`），導致 Gazebo GPU LiDAR 全部輸出 `inf`。`launch_clean.sh` 已自動設定 `__EGL_VENDOR_LIBRARY_FILENAMES` 強制使用 NVIDIA EGL 驅動，無需手動處理。

---

## 3. 環境安裝與建置 (Prerequisites & Build)

### 系統需求
* Ubuntu 22.04 LTS (Jammy)
* ROS 2 Humble
* Gazebo Ignition Fortress 6（`libignition-gazebo6`）
* Python 3.10+
* NVIDIA Driver 580+（RTX 5090 需 580.173.02 以上）

### 依賴套件安裝

~~~bash
sudo apt update
sudo apt install -y \
  ros-humble-ros-gz-sim \
  ros-humble-ros-gz \
  ros-humble-xacro \
  ros-humble-robot-state-publisher \
  ros-humble-joint-state-publisher \
  ros-humble-depth-image-proc \
  ros-humble-teleop-twist-keyboard \
  libasio-dev
~~~

### RL 訓練 Python 套件

~~~bash
pip install gymnasium stable-baselines3[extra] tensorboard --break-system-packages
~~~

### 工作區編譯

`turn_on_wheeltec_robot` 已從 ROS1（catkin）移植為 ROS2（ament_cmake），並新增 `transport_drivers`（serial_driver）作為序列埠函式庫：

~~~bash
cd ~/Desktop/4wd

# 首次使用需 clone transport_drivers（ros-drivers/serial_driver 相依）
mkdir -p src
git clone https://github.com/ros-drivers/transport_drivers.git src/

source /opt/ros/humble/setup.bash
rosdep install --from-paths src turn_on_wheeltec_robot --ignore-src -r -y
colcon build --symlink-install --packages-up-to turn_on_wheeltec_robot
~~~

### 環境自動載入（建議一次性設定）

~~~bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
echo "source ~/Desktop/4wd/install/setup.bash" >> ~/.bashrc
~~~

---

## 4. 快速開始 (Quick Start)

### 一般模擬（含 GUI）

啟動 Gazebo 豬舍場景、機器人 Spawn、ROS 2 Bridge 通訊、點雲處理節點與 RViz2：

~~~bash
cd ~/Desktop/4wd/scripts
./launch_clean.sh
~~~

支援的選項：

| 選項 | 說明 |
| :--- | :--- |
| `--headless` | 無頭模式：不開 Gazebo GUI 與 RViz，適合 RL 訓練 |
| `--env-id N` | 平行訓練環境隔離，設定 `IGN_PARTITION=pig_pen_N` |
| `./launch_clean.sh <world_file>` | 指定不同的 world 檔 |

此腳本會自動啟動：
* Gazebo world 豬舍場景（Ignition Fortress，`ign gazebo`）
* 機器人 URDF 自動 Spawn（出生點 Y=-14，走道右端起點）
* ROS 2 Bridge 通訊（`/scan`、`/odom`、`/cmd_vel`；有頭模式額外橋接相機 topics）
* 深度點雲轉換節點與 QoS Relay（僅有頭模式）
* 豬隻隨機行為控制節點（僅有頭模式）
* RViz 預設配置（僅有頭模式）

### 一鍵 RL 訓練（推薦）

~~~bash
cd ~/Desktop/4wd/scripts
./launch_rl_training.sh
~~~

自動執行：啟動無頭模擬 → 等待環境就緒 → 首次使用自動校準路徑點 → 啟動 TensorBoard → 開始 PPO 訓練。

### 手動場景與車型控制

~~~bash
# 僅啟動 Gazebo + 機器人（不含感測器與 RViz）
./scripts/launch_pig_pen.sh

# 手動 Spawn 特定車型（旗艦版）
./scripts/spawn_robot.sh --file turn_on_wheeltec_robot/urdf/flagship_four_wheel_diff_bs_robot.urdf \
    --model flagship_four_wheel_diff_bs --pos 0 0 0.05 --yaw 1.5708
~~~

### 鍵盤遙控小車

~~~bash
source ~/.bashrc
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/cmd_vel
~~~

**鍵盤按鍵對應：**

| 按鍵 | 動作 | 按鍵 | 動作 |
| :---: | :--- | :---: | :--- |
| `i` | 前進 | `k` | 強制停止 |
| `,` | 後退 | `q` / `z` | 最大速度 ±10% |
| `j` / `l` | 左轉 / 右轉 | `w` / `x` | 僅線速度 ±10% |
| `u` / `o` | 左前 / 右前轉向 | `e` / `c` | 僅角速度 ±10% |
| `m` / `.` | 左後 / 右後轉向 | | |

---

## 5. 強化學習訓練 (RL Pipeline)

使用 **PPO（Proximal Policy Optimization）** 訓練豬舍巡邏策略，透過 Stable-Baselines3 與 Gazebo 模擬對接。

### 架構概覽

```
launch_rl_training.sh
├── launch_clean.sh --headless     # Gazebo 無頭模擬
├── calibrate_waypoints.py         # 首次校準路徑點（互動式）
├── tensorboard                    # 訓練監控（背景）
└── train_ppo.py                   # PPO 訓練主程式
    └── pig_pen_env.py             # Gymnasium 環境（Gazebo 對接）
```

### 觀察空間 / 動作空間 / 獎勵函數

| 項目 | 內容 |
| :--- | :--- |
| **觀察空間**（39 維） | 36 條 LiDAR 射線（均勻取樣，正規化至 [0,1]）+ 到下個路徑點距離 + sin/cos 方位角 |
| **動作空間**（連續 2 維） | 線速度 ∈ [0, 0.5] m/s；角速度 ∈ [-1.5, 1.5] rad/s |
| **進度獎勵** | +5 × 靠近路徑點距離（公尺） |
| **到達路徑點** | +50 |
| **完成全程巡邏** | +200 |
| **碰撞懲罰** | -100，episode 終止 |
| **時間懲罰** | -0.01 / step |

### 巡邏路線設計

機器人從走道右端出發，依序拜訪主幹道與 3 條支線的上下端，最終抵達走道左端，共 16 個路徑點：

```
起點(右端) → 路口1 → 支線1上端 → 支線1下端 → 路口2 → 支線2上端 → 支線2下端
→ 路口3 → 支線3上端 → 支線3下端 → 終點(左端)
```

### 步驟 1：路徑點校準（首次使用）

`launch_rl_training.sh` 首次執行會自動觸發校準。若需手動重新校準：

~~~bash
# 刪除舊的路徑點檔案
rm ~/Desktop/4wd/rl_pig_pen/waypoints.json

# 重新執行（腳本會自動進入校準模式）
cd ~/Desktop/4wd/scripts
./launch_rl_training.sh
~~~

校準工具（`calibrate_waypoints.py`）特性：
* 每記錄一個點立刻寫入 `waypoints_partial.json`（防崩潰遺失）
* 重新執行會詢問是否從斷點繼續，無需從頭來

### 步驟 2：開始訓練

~~~bash
cd ~/Desktop/4wd/scripts
./launch_rl_training.sh
~~~

訓練過程監控（另開 Terminal）：

~~~bash
tensorboard --logdir ~/Desktop/4wd/rl_pig_pen/logs/
# 開瀏覽器：http://localhost:6006
~~~

### 訓練輸出

| 路徑 | 內容 |
| :--- | :--- |
| `rl_pig_pen/checkpoints/` | 每 20,000 步儲存一次的模型權重 |
| `rl_pig_pen/checkpoints/best/` | EvalCallback 評估最佳模型（停用中，見下注） |
| `rl_pig_pen/logs/` | TensorBoard 訓練曲線 |

> **注意**：`EvalCallback` 已停用（會建立第二個 Gazebo 環境造成衝突）。如需評估，訓練完後手動載入模型測試。

### RL 訓練超參數（`train_ppo.py`）

| 參數 | 值 | 說明 |
| :--- | :--- | :--- |
| `TOTAL_TIMESTEPS` | 3,000,000 | 約需 3–5 天（單環境，10 FPS） |
| `n_steps` | 2048 | 每次更新蒐集的步數 |
| `batch_size` | 512 | mini-batch 大小 |
| `learning_rate` | 3e-4 | Adam 學習率 |
| `net_arch` | [256, 256] | 兩層全連接網路 |
| `device` | cpu | MlpPolicy 用 CPU 比 GPU 快（GPU 適合 CNN 影像輸入） |

---

## 6. 行為複製 (Behavioral Cloning)

> **狀態**：規劃中，尚未實作。以下為預計流程，待錄製足夠的人工駕駛資料後啟用。

### 1. 錄製手動駕駛資料

啟動模擬後，開啟錄製腳本，手動駕駛小車在走道巡邏以收集 `/scan` 雷達資料與對應速度指令：

~~~bash
python3 scripts/teleop_recorder.py
~~~

*(資料將自動存為 `.npz` 格式於 `recorded_data/` 目錄中)*

### 2. GPU 加速行為複製訓練

讀取錄製的專家資料，於 GPU 上預訓練神經網路 Policy：

~~~bash
python3 scripts/train_bc_model.py
~~~

### 3. 走道巡邏控制節點

測試純雷達相對距離的走道自動巡邏與到底轉向邏輯：

~~~bash
python3 scripts/hallway_patrol.py
~~~

---

## 7. SLAM 建圖與自主導航

若要啟動 SLAM 建圖（需先安裝 `ros-humble-slam-toolbox`）：

~~~bash
ros2 launch slam_toolbox online_async_launch.py
~~~

儲存建好的地圖：

~~~bash
ros2 run nav2_map_server map_saver_cli -f my_pigpen_map
~~~

---

## 8. 常用工具與轉接腳本 (`scripts/`)

### 啟動腳本

| 腳本 | 說明 |
| :--- | :--- |
| `launch_clean.sh` | 模擬環境主啟動腳本。支援 `--headless`（無頭模式）、`--env-id N`（平行訓練隔離）。 |
| `launch_rl_training.sh` | 一鍵 RL 訓練腳本：自動啟動無頭模擬 → 校準路徑點 → TensorBoard → PPO 訓練。 |

### RL 訓練相關（`rl_pig_pen/`）

| 腳本 | 說明 |
| :--- | :--- |
| `calibrate_waypoints.py` | 互動式路徑點校準工具。支援中途中斷續點（`waypoints_partial.json` 自動存檔）。 |
| `pig_pen_env.py` | Gymnasium 環境：LiDAR 觀察、連續 cmd_vel 動作、Gazebo teleport reset、odom baseline 追蹤。支援 `env_id` 平行訓練。 |
| `train_ppo.py` | PPO 訓練主程式：Stable-Baselines3 + TensorBoard + checkpoint。 |
| `waypoints.json` | 校準後的巡邏路徑點座標（由 `calibrate_waypoints.py` 產生）。 |

### 轉接與工具腳本

| 腳本 | 說明 |
| :--- | :--- |
| `pointcloud_qos_relay.py` | 將 `BEST_EFFORT` 點雲轉換為 `RELIABLE` 的 `/camera/depth/points_reliable` 供 RViz 穩定顯示。 |
| `republish_odom.py` | 將 `/model/wheeltec_mini/odometry` 轉發至標準 `/odom` Topic。 |
| `cmdvel_to_wheels.py` | 依差速車運動學將 `/cmd_vel` 換算為左右輪角速度。 |
| `cmdvel_to_stm32.py` | 將 `/cmd_vel` 轉為實體 STM32 串口 11-byte 通訊封包。 |
| `control_cmdvel.py` | 快速發送單次或定時速度測試指令。 |

~~~bash
# 範例：前進 0.2 m/s 持續 3 秒
python3 scripts/control_cmdvel.py --linear 0.2 --duration 3
~~~

---

## 9. 串口控制協議 (STM32 實體底盤)

> **前提**：`turn_on_wheeltec_robot` 套件已從 ROS1（catkin）移植為 ROS2（ament_cmake）。詳見[移植紀錄](#11-移植紀錄-ros1--ros2--fortress)。

* **串口規格**：115200 Baud, 8 Data Bits, 1 Stop Bit, No Parity (8N1)，硬體預設串口 3。
* **udev symlink**：預設裝置路徑 `/dev/wheeltec_controller`；若不存在，使用 `--ros-args -p usart_port_name:=/dev/ttyUSB0` 覆寫。

### 下行通訊封包結構 (11 Bytes)

| Byte | 內容 | 說明 |
| :---: | :--- | :--- |
| [0] | `0x7B` | 幀頭 |
| [1] | `0x00` | 預留 |
| [2] | `0x00` | 預留 |
| [3] | X MSB | X 軸線速度高位元組（mm/s，Big-Endian） |
| [4] | X LSB | X 軸線速度低位元組 |
| [5] | Y MSB | Y 軸線速度高位元組 |
| [6] | Y LSB | Y 軸線速度低位元組 |
| [7] | Z MSB | Z 軸角速度高位元組（rad/s × 1000） |
| [8] | Z LSB | Z 軸角速度低位元組 |
| [9] | Checksum | 前 9 Bytes XOR 校驗碼 |
| [10] | `0x7D` | 幀尾 |

### 上行資料（下位機 → ROS2）

* 24-byte 封包，包含底盤 XYZ 速度、MPU6050 六軸 IMU 原始資料、電源電壓。
* IMU 轉換比率：加速度計 `ACCEl_RATIO = 1671.84`（±2g / ±32768），陀螺儀 `GYROSCOPE_RATIO = 0.00026644`（±500°/s / ±32768）。
* 四元數解算：Mahony 濾波器（`Quaternion_Solution.cpp`），取樣頻率 20Hz。

### 啟動實體串口橋接

~~~bash
# 先 build 套件
source ~/.bashrc
colcon build --packages-select turn_on_wheeltec_robot

# 啟動節點
ros2 run turn_on_wheeltec_robot wheeltec_robot_node \
    --ros-args -p usart_port_name:=/dev/wheeltec_controller \
               -p serial_baud_rate:=115200
~~~

---

## 10. 外部依賴與 Git 子模組 (ZED Description)

本專案使用官方 ZED 描述包作為機器人相機的 xacro/mesh 定義，放置於 `zed_description` 子模組。

~~~bash
# 剛 Clone 專案時更新子模組
git submodule update --init --recursive

# 或透過 apt 安裝備用描述包
sudo apt install ros-humble-zed-description
~~~

---

## 11. 移植紀錄 (ROS1 → ROS2 / Fortress)

### Gazebo Fortress 指令變更

原本 ROS1 時期使用 `gz sim`，Fortress 需改用 `ign gazebo`：

| 原指令（ROS1 / Garden+） | Fortress 對應指令 |
| :--- | :--- |
| `gz sim -s -r world.sdf` | `ign gazebo -s -r world.sdf` |
| `gz sim -g` | `ign gazebo -g` |
| `GZ_SIM_RESOURCE_PATH` | `IGN_GAZEBO_RESOURCE_PATH`（兩者皆設定以求相容） |

### `turn_on_wheeltec_robot` ROS2 移植重點

原始套件為 ROS1（catkin）架構，與 ROS2 Humble 不相容，已完成移植：

* `CMakeLists.txt`：`catkin` → `ament_cmake`
* `package.xml`：`<buildtool_depend>catkin</buildtool_depend>` → `ament_cmake`
* C++ API：`ros::NodeHandle` → `rclcpp::Node` 繼承；`ros::Publisher` / `Subscriber` → `create_publisher` / `create_subscription`
* TF：`tf::TransformBroadcaster` → `tf2_ros`
* 訊息型別：`nav_msgs::Odometry` → `nav_msgs::msg::Odometry`（所有訊息加 `::msg::`）
* 序列埠：`wjwwood/serial`（ROS1 生態，無 Humble apt 套件）→ `ros-drivers/transport_drivers` 中的 `serial_driver`

### RTX 5090（Blackwell）相容性設定

`launch_clean.sh` 已自動套用以下設定，無需手動處理：

~~~bash
# 強制 Gazebo 使用 NVIDIA EGL（避免 Mesa 不支援 PCI ID 0x7d67 導致 GPU LiDAR 全 inf）
export __EGL_VENDOR_LIBRARY_FILENAMES=/usr/share/glvnd/egl_vendor.d/10_nvidia.json

# 抑制 Ignition Transport ZMQ 連線統計（減少多節點同時啟動時的崩潰機率）
export IGN_TRANSPORT_TOPIC_STATISTICS=0
~~~

---

## 12. 常見問題與除錯 (Troubleshooting)

### 模擬環境

**按鍵盤車子不會動**
* 確認 `/cmd_vel` bridge 方向符號為 `]`（ROS→GZ）：`/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist`
* 確保鍵盤焦點在執行 `teleop_twist_keyboard` 的 Terminal 上。

**LiDAR `/scan` 全部輸出 `inf`**
* 原因：Mesa 不支援 RTX 5090（Blackwell）GPU，Gazebo 無法初始化 GPU LiDAR raycast。
* 修復：確認 `launch_clean.sh` 中有 `export __EGL_VENDOR_LIBRARY_FILENAMES=/usr/share/glvnd/egl_vendor.d/10_nvidia.json`。
* 驗證：`ls /usr/share/glvnd/egl_vendor.d/` 應看到 `10_nvidia.json`。

**RViz 畫面全黑 / 找不到 Displays 面板**
* 點擊 RViz 左上角 **Panels → Displays** 開啟面板。
* 確保 Topic 已正確選擇（`/camera/depth/points_reliable`、`/camera/color/image_raw` 等）。

**小車行駛時過度打滑**
* 在 `four_wheel_diff_bs_robot.urdf` 輪胎 `<gazebo>` 標籤中，將 `mu1` / `mu2` 調高至 20.0～50.0。
* 在 DiffDrive 插件中限制最大加速度：`<max_linear_acceleration>1.5</max_linear_acceleration>`。

**Gazebo server 崩潰（`libzmq abort`）**
* 原因：多個 ROS2 節點同時建立 ZMQ 連線時 Ignition Transport 有已知不穩定問題。
* 修復：確認 `IGN_TRANSPORT_TOPIC_STATISTICS=0` 已設定；重新執行前先清理殭屍程序（見下）。

### 殭屍程序清理

每次 `launch_clean.sh` 異常終止後需確認程序確實清除，否則下次啟動會衝突：

~~~bash
pkill -9 -f "static_transform_publisher" || true
pkill -9 -f "ros_gz_bridge"              || true
pkill -9 -f "ign gazebo"                 || true
pkill -9 -f "robot_state_publisher"      || true
# 確認歸零（輸出應為 0 或 1，那 1 個是系統 unattended-upgrades，不用管）
ps aux | grep -E "static_transform|ros_gz|ign" | grep -v grep | wc -l
~~~

### RL 訓練

**`launch_rl_training.sh` 等待 120 秒超時**
1. 確認無殭屍程序（見上方清理步驟）。
2. 查看 log：`tail -30 ~/Desktop/4wd/logs/launch_clean_rl.log`
3. 確認 `launch_clean.sh` 有 `--headless` 旗標：`grep "launch_clean" ~/Desktop/4wd/scripts/launch_rl_training.sh`

**`waypoints.json` 中有 `PLACEHOLDER` 卻跳過校準**
* 原因：腳本用 `grep -q "PLACEHOLDER"` 檢查，檔案不存在時 grep 回傳失敗誤判為「已校準」。
* 修復：刪除 `waypoints.json` → 重新執行腳本。

**`等待 /scan 或 /odom 超時`（訓練中途）**
* 原因：Gazebo 在訓練過程中崩潰，`/scan` topic 中斷。
* 查看：`tail -20 ~/Desktop/4wd/logs/gz_sim_server.log`
* 處理：Ctrl-C 停止訓練，清理程序後重新執行；上次 checkpoint 不會遺失，訓練會從最近一個 checkpoint 繼續。

**`ValueError: generator already executing`**
* 原因：多個環境物件共用同一個 rclpy executor。
* 修復：`pig_pen_env.py` 應使用獨立的 `SingleThreadedExecutor`（已修正）。

**`can't subtract times with different time sources`**
* 原因：`rclcpp::Time` 建構時未指定 clock source，與 `this->now()` 不相容。
* 修復：`turn_on_robot` 建構子開頭加入 `_Now = this->now(); _Last_Time = this->now();`（已修正）。
