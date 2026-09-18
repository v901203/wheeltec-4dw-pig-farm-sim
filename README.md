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
./launch_rl_training.sh --mode patrol --timesteps 300000
~~~

自動執行：開啟 Gazebo 視窗 → 等待環境就緒 → 啟動 TensorBoard → 開始整圖 PPO＋狀態機巡邏訓練。
`--mode patrol` 在同一回合巡邏六條支線並返回；省略時使用走道片段訓練。
動作支援原地左右旋轉，路口轉向與末端掉頭由狀態機執行。不需要校準路徑點。
預設開窗，只有指定 `--headless` 才使用無視窗模式。詳細流程見 [整圖訓練說明](rl_pig_pen/README.md)。
訓練也會另開終端機，每 15 秒顯示進度與數據，每回合列出中文重生原因與累計次數。
按 Ctrl-C 可中止並存檔；輸出同步存到 `logs/rl_training_*.log`。
加 `--same-terminal` 可使用原本的終端機。

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

## 5. 強化學習與完整巡邏

採用 **38 維感測 PPO ＋巡邏狀態機**。RL 負責一般走道前進、置中與避障；
狀態機負責路口進出、穿越、掉頭，以及 3 個路口兩側共 6 個端點的巡邏記憶。
不需要 `waypoints.json` 或互動式路徑點校準。`/odom` 仍用於實測速度、相對轉角和短程距離。

| 項目 | 定義 |
| :--- | :--- |
| 觀察 `0:36` | 36 條均勻取樣 LiDAR，截斷至 10 m 並除以 10 |
| 觀察 `36` | 左側 60°～120° 平均距離減右側 −120°～−60° 平均距離，除以 10，範圍 [-1,1] |
| 觀察 `37` | 前方 ±30° 最小距離除以 10，範圍 [0,1] |
| 動作 | 線速度 [0,1] m/s、角速度 [-2,2] rad/s |
| 一般走道獎勵 | `2 × odom 實測前向速度 − abs(左右平均距離差，公尺) − 0.05` |
| 碰撞代理判定 | LiDAR 點侵入含輪子的矩形車體及裕量：獎勵精確為 −100、終止並停車 |

新特徵與安全檢查使用完整掃描。前進獎勵不使用命令速度。
到達開口即結束訓練片段，該步不扣置中分；狀態機動作不進入 PPO 訓練。
支線末端使用「已走過最低距離＋兩側欄舍結束」辨識，前方障礙物不算完成。
只有所有端點都到達且返回、再抵達主幹道末端，才算整體成功。

### 訓練與試跑

~~~bash
# 一鍵啟動模擬、TensorBoard 與走道 PPO 訓練
bash scripts/launch_rl_training.sh

# 或在已啟動 launch_clean.sh 的模擬中訓練
python3 rl_pig_pen/train_ppo.py --timesteps 3000000

# 訓練結束後，在已啟動的模擬中測試完整巡邏
python3 rl_pig_pen/test_model.py --episodes 3
~~~

預設只從 `rl_pig_pen/checkpoints_corridor38/` 自動選取最新新版模型續訓；
舊的 39 維模型保留在原資料夾，不直接續訓。可用 `--resume PATH` 指定相容模型。
TensorBoard 目錄為 `rl_pig_pen/logs_corridor38/`，每 20,000 步存一次模型。
維持 PPO `[256,256]` 網路、2048 rollout steps、512 batch size、CPU 執行。
訓練模式預設目標 6 倍速、停用三個相機並使用單執行緒 MLP；物理步長與 LiDAR
取樣率維持原設定。可用 `bash scripts/launch_rl_training.sh --rtf 6` 調整倍率，
實際速度仍須以 PPO 的 `fps` 衡量。
訓練和測試不可同時控制同一個模擬。

感測步進依雷達時間戳，名義週期為 1/12 模擬秒，不使用固定三倍速假設。
自由運行的 Gazebo 仍可能有排程延遲，`info["actual_dt"]` 提供實際間隔；
每次取樣後送出停車命令，避免模型更新期間持續行駛。

完整參數、限制與測試方式見 [RL 設計說明](rl_pig_pen/README.md)。

### 最新模型存檔

目前最新的整圖 PPO 實驗存檔為：

```text
rl_pig_pen/checkpoints_patrol38/ppo_corridor38_interrupted.zip
```

這是中斷時保存的 checkpoint，目標為 300,000 步，但尚未完成訓練與成功巡邏驗證；
最近回合完成率為 `0/6`，主要結束原因是 `stuck`。載入測試：

```bash
python3 rl_pig_pen/test_model.py \
  --model rl_pig_pen/checkpoints_patrol38/ppo_corridor38_interrupted.zip \
  --episodes 3
```

模型檔已納入 GitHub repository；訓練日誌與示範資料仍由 `.gitignore` 排除。

---

## 6. 手動示範與行為複製 (Behavioral Cloning)

已支援「手動錄製 → 離線 BC 預訓練 → PPO 續訓」。使用同一套 38 維觀察與
`[線速度, 角速度]` 動作，BC 模型可直接由現有 PPO 載入。

先在原訓練終端按 Ctrl-C 並等待存檔、清理完成。從專案根目錄，在三個終端依序執行：

~~~bash
# 終端 1：手動示範用 1 倍目標速度，開啟 Gazebo 畫面
bash scripts/launch_clean.sh --rtf 1

# 終端 2：只讀取 /scan 與人工 /cmd_vel，不會自行控制車子
python3 rl_pig_pen/record_demonstrations.py

# 終端 3：鍵盤駕駛；持續按住 i/u/o 等移動鍵以持續送出指令
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p speed:=0.2 -p turn:=0.5
~~~

示範走道內的直行、偏左／偏右後的修正與減速避障。`i` 向前，`u/o` 向前左／右轉，
`k` 停車。倒車、原地轉向與路口開放區域不納入這個局部策略，路口選向仍由狀態機處理。
錄完先停車、結束鍵盤，再於錄製終端按 Ctrl-C 儲存尾段。建議分成數段獨立示範，
包含兩側偏移與不同走道；有效樣本數會顯示在終端。資料放在 `rl_pig_pen/demonstrations/`。

~~~bash
# 離線預訓練，不需要啟動 Gazebo
python3 rl_pig_pen/train_bc.py

# 停止手動控制、保持 Gazebo 運行，從 BC 模型接續 PPO
python3 rl_pig_pen/train_ppo.py --resume rl_pig_pen/checkpoints_bc/ppo_corridor38_bc.zip \
  --checkpoint-dir rl_pig_pen/checkpoints_from_bc --log-dir rl_pig_pen/logs_from_bc \
  --timesteps 300000
~~~

BC 只預訓練 actor，critic 由後續 PPO 學習。驗證誤差改善不代表完整巡邏已成功；
需另外跑 `test_model.py --checkpoint-dir rl_pig_pen/checkpoints_from_bc` 評估。
完整操作、資料篩選、搖桿與續訓方式見 [模仿學習指南](rl_pig_pen/IMITATION.md)。

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
| `launch_rl_training.sh` | 一鍵 RL 訓練腳本：預設開啟 Gazebo 視窗 → TensorBoard → 38 維 PPO；`--mode patrol` 啟用整圖巡邏訓練。 |

### RL 訓練相關（`rl_pig_pen/`）

| 腳本 | 說明 |
| :--- | :--- |
| `calibrate_waypoints.py` | 舊版路徑點校準工具，新版不使用。 |
| `pig_pen_env.py` | 38 維 Gymnasium 環境；訓練片段與完整巡邏測試分開。 |
| `navigation.py` | 感測特徵、獎勵、車體碰撞代理與速度安全檢查。 |
| `patrol_controller.py` | 以相對運動與拓樸記憶完成六個端點來回的狀態機。 |
| `patrol_config.json` | 出生姿態、路口數、偵測與控制參數。 |
| `test_model.py` | 統計覆蓋率、返回完成率、碰撞率及全程成功率。 |
| `train_ppo.py` | PPO 訓練主程式：Stable-Baselines3 + TensorBoard + checkpoint，可從 BC 模型續訓。 |
| `record_demonstrations.py` | 記錄人工速度指令與 38 維雷達觀察，分段存檔。 |
| `train_bc.py` | 離線行為複製預訓練，輸出相容 PPO 的模型。 |
| `waypoints.json` | 保留的舊版座標，新版不讀取。 |

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
3. 預設開啟 Gazebo 視窗；在沒有桌面顯示的環境才加入 `--headless`。

新版 38 維訓練不使用 `waypoints.json`，不需執行路徑點校準。

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
