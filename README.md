# Wheeltec 4WD 豬舍 ROS 2 / Gazebo 模擬專案

本專案基於 **ROS 2 Humble** 與 **Gazebo (Ignition Fortress / Sim)**，專為四輪驅動 (4WD) 巡檢機器人在豬舍環境中的感測器融合、深度學習（行為複製 BC / 強化學習 RL）、SLAM 建圖與自主導航所設計。支援 Gazebo 模擬環境與實體 Wheeltec STM32 底盤雙軌運行。

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

### 豬隻與 ArUco 視覺標記
* 每間豬舍配備 1 隻假豬模型 (`farm_pig`)，放置於飼料盆旁（偏移 0.7m 避開碰撞體）。
* 豬隻前端帶有 **ArUco Marker 貼圖**，統一朝向中央走道：
  * **左排 (`pig1` ～ `pig8`)**：ArUco 面朝向 +X 方向。
  * **右排 (`pig9` ～ `pig16`)**：ArUco 面旋轉 180 度朝向 -X 方向。

---

## 2. 機器人硬體與感測器配置

底盤基於 **Wheeltec 4WD 差速車 (`wheeltec_mini`)**，URDF 檔位於 `turn_on_wheeltec_robot/urdf/four_wheel_diff_bs_robot.urdf`：

* **底盤尺寸**：0.40m × 0.30m × 0.065m，離地高度 4.5cm，輪胎直徑 15cm（半徑 7.5cm）。
* **預設出生點**：`X=0.0, Y=0.0, Z=0.05`，朝向 Y 軸正方向（`yaw = 1.5708`），位於中央走道正起點。
* **感測器裝備**：
  1. **2D LiDAR (M10P-PHY)**：Topic `/scan`，位於 Z 軸 0.245m 處。
  2. **前方深度相機 (ZED X)**：
     * 深度圖 Topic：`/camera/depth/image_raw`
     * 彩色圖 Topic：`/camera/color/image_raw`
  3. **右側監視相機 (Right Camera)**：
     * 彩色圖 Topic：`/camera_right/image_raw`（朝右側拍攝豬欄）

---

## 3. 環境安裝與建置 (Prerequisites & Build)

### 依賴套件安裝
在新電腦或環境下，請先安裝 ROS 2 與 Gazebo 模擬相依套件：

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

### 工作區編譯
~~~bash
cd ~/Desktop/4wd
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to turn_on_wheeltec_robot
~~~

---

## 4. 快速開始 (Quick Start)

### 一鍵完整啟動 (推薦)
自動開啟 Gazebo 豬舍場景、機器人 Spawn、ROS 2 Bridge 通訊、點雲處理節點與 RViz2：

~~~bash
cd ~/Desktop/4wd
source /opt/ros/humble/setup.bash
source install/setup.bash
./scripts/launch_clean.sh
~~~

此腳本會自動啟動：
* Gazebo world 豬舍場景（自動相容 `ign gazebo` 與 `gz sim`）
* 機器人 URDF 自動 Spawn
* ROS 2 Bridge 通訊（包含 `/scan`、`/camera/depth/image_raw`、`/camera/color/image_raw`、`/camera_right/image_raw`、`/odom`、`/cmd_vel`）
* 深度點雲轉換節點 (`depth_image_proc`) 與 QoS Relay
* 豬隻隨機行為控制節點 (`pig_behavior.py`)
* RViz 預設配置 (`rviz/wheeltec.rviz`)

### 手動場景與車型控制
* **僅啟動 Gazebo + 機器人（不含感測器與 RViz）**：
  ~~~bash
  ./scripts/launch_pig_pen.sh
  ~~~
* **手動 Spawn 特定車型 (如旗艦版)**：
  ~~~bash
  ./scripts/spawn_robot.sh --file turn_on_wheeltec_robot/urdf/flagship_four_wheel_diff_bs_robot.urdf --model flagship_four_wheel_diff_bs --pos 0 0 0.05 --yaw 1.5708
  ~~~

### 鍵盤遙控小車
啟動新的終端機，執行鍵盤控制節點：

~~~bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/cmd_vel
~~~

**鍵盤按鍵對應：**
* `i`：前進
* `,`：後退
* `j` / `l`：左轉 / 右轉
* `u` / `o`：左前方轉向 / 右前方轉向
* `m` / `.`：左後方轉向 / 右後方轉向
* `k`：強制停止
* `q` / `z`：最大速度增加 / 減少 10%
* `w` / `x`：僅調整線速度 10%
* `e` / `c`：僅調整角速度 10%

---

## 5. 深度學習與數據收集 (Behavioral Cloning / DRL)

本專案提供基於 **PyTorch (CUDA GPU 加速)** 與 **Stable-Baselines3** 的模仿學習與走道巡邏工具：

### 1. 錄製手動駕駛資料 (Expert Demonstrations)
啟動模擬後，開啟錄製腳本，手動駕駛小車在走道巡邏以收集姿態與 `/scan` 雷達資料：
~~~bash
python3 scripts/teleop_recorder.py
~~~
*(資料將自動存為 `.npz` 格式於 `recorded_data/` 目錄中)*

### 2. GPU 加速行為複製訓練 (Behavioral Cloning Training)
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

## 6. SLAM 建圖與自主導航

若要啟動 SLAM 建圖（需先安裝 `ros-humble-slam-toolbox`）：

~~~bash
ros2 launch slam_toolbox online_async_launch.py
~~~

儲存建好的地圖：
~~~bash
ros2 run nav2_map_server map_saver_cli -f my_pigpen_map
~~~

---

## 7. 常用工具與轉接腳本 (`scripts/`)

* **`scripts/launch_clean.sh`**：整合主啟動腳本。
* **`scripts/pointcloud_qos_relay.py`**：將 `BEST_EFFORT` 點雲轉換為 `RELIABLE` 的 `/camera/depth/points_reliable` 供 RViz 穩定顯示。
* **`scripts/republish_odom.py`**：將 `/model/wheeltec_mini/odometry` 轉發至標準 `/odom` Topic，方便導航與 SLAM 模組對接。
* **`scripts/cmdvel_to_wheels.py`**：依差速車運動學將 `/cmd_vel` 換算為左右輪角速度，發布至 `/wheeltec_mini/left_wheel_velocity` 及 `right_wheel_velocity`。
* **`scripts/cmdvel_to_stm32.py`**：將 `/cmd_vel` 轉為實體 STM32 串口 11-byte 通訊封包 (115200 8N1)。
* **`scripts/control_cmdvel.py`**：快速發送單次或定時速度測試指令：
  ~~~bash
  python3 scripts/control_cmdvel.py --linear 0.2 --duration 3
  ~~~

---

## 8. 串口控制協議 (STM32 實體底盤)

本專案提供與 Wheeltec STM32 底盤相容的串口控制橋接：

* **串口規格**：115200 Baud, 8 Data Bits, 1 Stop Bit, No Parity (115200 8N1)，硬體預設串口 3。
* **資料型態與單位**：
  * X, Y 軸速度：有號 16-bit Short，單位 mm/s (Big-Endian)。
  * Z 軸角速度：有號 16-bit Short，等於 angular_z (rad/s) × 1000 (Big-Endian)。

### 下行通訊封包結構 (11 Bytes)

| 索引 (Byte) | 內容 | 說明 |
| :--- | :--- | :--- |
| **[0]** | `0x7B` | 幀頭 (Header) |
| **[1]** | `0x00` | 預留 |
| **[2]** | `0x00` | 預留 |
| **[3]** | X MSB | X 軸線速度高位元組 |
| **[4]** | X LSB | X 軸線速度低位元組 |
| **[5]** | Y MSB | Y 軸線速度高位元組 |
| **[6]** | Y LSB | Y 軸線速度低位元組 |
| **[7]** | Z MSB | Z 軸角速度高位元組 |
| **[8]** | Z LSB | Z 軸角速度低位元組 |
| **[9]** | Checksum | 前 9 Bytes 之 XOR 校驗碼 |
| **[10]** | `0x7D` | 幀尾 (Tail) |

啟動實體串口橋接：
~~~bash
python3 scripts/cmdvel_to_stm32.py --ros-args -p port:=/dev/ttyUSB0 -p baudrate:=115200
~~~

---

## 9. 外部依賴與 Git 子模組 (ZED Description)

本專案使用官方 ZED 描述包作為機器人相機的 xacro/mesh 定義，放置於 `zed_description` 子模組。

若剛 Clone 本專案，請更新子模組：
~~~bash
git submodule update --init --recursive
~~~

或透過系統 apt 安裝備用描述包：
~~~bash
sudo apt install ros-humble-zed-description
~~~

---

## 10. 常見問題與除錯 (Troubleshooting)

1. **按鍵盤車子不會動**：
   * 檢查 `ros_gz_bridge` 中 `/cmd_vel` 的方向符號必須為 `]` (`/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist`)。
   * 確保鍵盤焦點維持在執行 `teleop_twist_keyboard` 的終端機視窗上。
2. **RViz 畫面全黑 / 找不到 Displays 面板**：
   * 點擊 RViz 左上角 **Panels -> Displays** 勾選開啟面板。
   * 確保 Topic 已正確選擇（如 `/camera/depth/points_reliable`、`/camera/color/image_raw` 或 `/camera_right/image_raw`）。
3. **小車行駛時過度打滑**：
   * 可在 `four_wheel_diff_bs_robot.urdf` 的輪胎 `<gazebo>` 標籤中，將摩擦係數 `mu1` 與 `mu2` 調高至 `20.0 ~ 50.0`。
   * 可在 DiffDrive 插件中限制最大加速度（如 `<max_linear_acceleration>1.5</max_linear_acceleration>`）。
