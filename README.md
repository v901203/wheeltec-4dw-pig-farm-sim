Wheeltec 4WD 豬舍 ROS 2 / Gazebo 模擬專案
## 專案簡介

本專案基於 ROS 2 Humble 與 Gazebo (Ignition/Sim)，專為四輪驅動 (4WD) 巡檢機器人在豬舍環境中的感測器融合、深度學習 (RL / 行為複製 BC)、SLAM 與自主導航所設計。支援 Gazebo 模擬器與實體 Wheeltec STM32 底盤雙軌運行。

## 1. 網格與場景規格 (Pig Pen World)
目前預設使用的豬舍世界檔為 worlds/pig_pen_8units(lv4).world（舊版可參考 worlds/pig_pen_8units.world），包含 8 間單獨豬舍與 8 隻帶有 ArUco 貼圖的假豬模型：

### 豬舍結構規格
- 單間尺寸：長 3.30m x 寬 2.55m 長方形。

- 四面欄杆牆：

- 長邊牆 (left_wall / right_wall)：3.30m，各含 32 根圓柱欄杆。

- 短邊牆 (front_wall / back_wall)：2.55m，各含 24 根圓柱欄杆。

- 欄杆規格：總高 83cm、直徑 1.5cm、中心間距 10cm（淨空隙 8.5cm）。

- 高度校正：下橫桿 0.035m、欄杆中心 0.415m、上橫桿 0.795m，完全貼合地面無空隙。

### 長邊相鄰佈局 (Long-Side Adjacent)
- 左排 (X = -2.4m)：pen1 -> pen2 -> pen3 -> pen4，沿 Y 軸長邊相鄰排列（間距 4cm）。

- 右排 (X = +2.4m)：pen5 -> pen6 -> pen7 -> pen8，沿 Y 軸長邊相鄰排列（間距 4cm）。

- 中央巡檢走道：兩排之間預留 1.5m 寬走道（沿 Y 軸延伸，X 範圍約在 -0.75m ~ +0.75m）。

### 豬隻與 ArUco 視覺標記
- 每間豬舍配備 1 隻假豬模型 (farm_pig)，放置於飼料盆旁（偏移 0.7m 避開碰撞體）。

豬隻前端帶有 ArUco Marker 貼圖，統一朝向中央走道：

- 左排 (pig1 ~ pig4)：ArUco 面朝向 +X 方向。

- 右排 (pig5 ~ pig8)：ArUco 面旋轉 180 度朝向 -X 方向。

## 2. 機器人硬體與感測器配置
底盤基於 Wheeltec 4WD 差速車 (wheeltec_mini)，URDF 檔位於 turn_on_wheeltec_robot/urdf/four_wheel_diff_bs_robot.urdf：

底盤尺寸：0.40m x 0.30m x 0.065m，離地高度 4.5cm，輪胎直徑 15cm（半徑 7.5cm）。

預設出生點：X=0.0, Y=0.0, Z=0.05，朝向 Y 軸正方向（yaw = 1.5708），位於中央走道正起點。

感測器裝備：

2D LiDAR (M10P-PHY)：Topic /scan，位於 Z 軸 0.245m 處。

前方深度相機 (ZED X)：

深度圖 Topic：/camera/depth/image_raw

彩色圖 Topic：/camera/color/image_raw

右側監視相機 (Right Camera)：

彩色圖 Topic：/camera_right/image_raw（朝右側拍攝豬欄）

## 3. 快速開始
### 一鍵完整啟動 (推薦)
自動開啟 Gazebo 豬舍場景、機器人 Spawn、ROS 2 Bridge 通訊、點雲處理節點與 RViz2。
打開終端機，執行：

cd /home/vito/Desktop/4wd
source /opt/ros/humble/setup.bash
source install/setup.bash
./scripts/launch_clean.sh
(此腳本會自動啟動：Gazebo 豬舍場景、機器人自動 Spawn、ROS 2 Bridge 通訊包含雷達/相機/里程計/控制訊號、深度點雲轉換節點、點雲 QoS Relay 以及 RViz 預設配置)

### 手動場景與車型控制
若只需啟動 Gazebo + 機器人（不含感測器與 RViz）：

./scripts/launch_pig_pen.sh
手動 Spawn 特定車型 (如旗艦版)：

./scripts/spawn_robot.sh --file turn_on_wheeltec_robot/urdf/flagship_four_wheel_diff_bs_robot.urdf --model flagship_four_wheel_diff_bs --pos 0 0 0.05 --yaw 1.5708
### 鍵盤遙控小車
啟動新的終端機，執行鍵盤控制節點：

ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/cmd_vel

鍵盤控制方式：

```

text
移動控制：

u    i    o
j    k    l
m    ,    .

        ↑
   左轉  前進  右轉
        ↓
      後退

按住 Shift 可啟用全向移動（側移）：

U    I    O
J    K    L
M    <    >

其他控制：

t：上升（+Z）
b：下降（-Z）

其他未定義按鍵：停止

q / z：最大速度增加 / 減少 10%
w / x：僅增加 / 減少線速度 10%
e / c：僅增加 / 減少角速度 10%

Ctrl-C：結束鍵盤控制

目前預設速度：
speed 0.50
turn  1.00

注意：
- 執行 teleop_twist_keyboard 的終端機必須保持鍵盤焦點，小車才會接收到按鍵指令。
- 如果小車沒有反應，請先確認 Gazebo、ROS 2 Bridge 與 /cmd_vel 都已正常啟動。
```

## 4. 深度學習與數據收集 (Behavioral Cloning / DRL)
本專案提供基於 PyTorch (CUDA GPU 加速) 與 Stable-Baselines3 的模仿學習與走道巡邏工具：

### 錄製手動駕駛資料 (Expert Demonstrations)
啟動模擬後，開啟錄製腳本，手動駕駛小車在走道巡邏以收集姿態與 /scan 雷達資料（資料自動存為 .npz 格式於 recorded_data/ 目錄中）：

python3 scripts/teleop_recorder.py
### GPU 加速行為複製訓練 (Behavioral Cloning Training)
讀取錄製的專家資料，於 GPU 上預訓練神經網路 Policy：

python3 scripts/train_bc_model.py
### 走道巡邏控制節點
測試純雷達相對距離的走道自動巡邏與到底轉向邏輯：

python3 scripts/hallway_patrol.py
## 5. SLAM 建圖與自主導航
啟動 SLAM 建圖（需先安裝 slam_toolbox）：

ros2 launch slam_toolbox online_async_launch.py
儲存建好的地圖：

ros2 run nav2_map_server map_saver_cli -f my_pigpen_map
## 6. 常用工具與轉接腳本 (scripts/)
launch_clean.sh：整合主啟動腳本。

pointcloud_qos_relay.py：將 BEST_EFFORT 點雲轉換為 RELIABLE 的 /camera/depth/points_reliable 供 RViz 穩定顯示。

republish_odom.py：將 /model/wheeltec_mini/odometry 轉發至標準 /odom Topic，方便導航與 SLAM 模組對接。

cmdvel_to_wheels.py：依差速車運動學將 /cmd_vel 換算為左右輪角速度，發布至 /wheeltec_mini/left_wheel_velocity 及 right_wheel_velocity。

cmdvel_to_stm32.py：將 /cmd_vel 轉為實體 STM32 串口 11-byte 通訊封包 (115200 8N1)。

control_cmdvel.py：快速發送單次或定時速度測試指令。

python3 scripts/control_cmdvel.py --linear 0.2 --duration 3
## 7. 串口控制協議 (STM32 實體底盤)
本專案提供與 Wheeltec STM32 底盤相容的串口控制橋接：

串口規格：115200 Baud, 8 Data Bits, 1 Stop Bit, No Parity (115200 8N1)，硬體預設串口 3。

資料型態與單位：

X, Y 軸速度：有號 16-bit Short，單位 mm/s (Big-Endian)。

Z 軸角速度：有號 16-bit Short，等於 angular_z (rad/s) * 1000 (Big-Endian)。

下行封包結構 (11 Bytes)：

索引 (Byte)	描述
[0]	0x7B (幀頭)
[1]	預留 (0x00)
[2]	預留 (0x00)
[3]	X 軸速度 MSB
[4]	X 軸速度 LSB
[5]	Y 軸速度 MSB
[6]	Y 軸速度 LSB
[7]	Z 軸角速度 MSB
[8]	Z 軸角速度 LSB
[9]	校驗碼 (前 9 Bytes 之 XOR 校驗)
[10]	0x7D (幀尾)

啟動實體串口橋接：

python3 scripts/cmdvel_to_stm32.py --ros-args -p port:=/dev/ttyUSB0 -p baudrate:=115200
## 8. 外部依賴與 Git 子模組 (ZED Description)
本專案使用官方 ZED 描述包作為機器人相機的 xacro/mesh 定義，放置於 zed_description 子模組。
若剛 Clone 本專案，請更新子模組：

git submodule update --init --recursive
或透過系統 apt 安裝備用描述包：

sudo apt install ros-humble-zed-description
## 9. 常見問題與除錯 (Troubleshooting)
按鍵盤車子不會動：

檢查 ros_gz_bridge 中 /cmd_vel 的方向符號必須為 ] (/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist)。

確保鍵盤焦點維持在執行 teleop_twist_keyboard 的終端機視窗上。

RViz 畫面全黑 / 找不到 Displays 面板：

點擊 RViz 左上角 Panels -> Displays 勾選開啟面板。

確保 Topic 已正確選擇（如 /camera/depth/points_reliable、/camera/color/image_raw 或 /camera_right/image_raw）。

小車行駛時過度打滑：

可在 four_wheel_diff_bs_robot.urdf 的輪胎 <gazebo> 標籤中，將摩擦係數 mu1 與 mu2 調高至 20.0 ~ 50.0。

可在 DiffDrive 插件中限制最大加速度（如 <max_linear_acceleration>1.5</max_linear_acceleration>）。
