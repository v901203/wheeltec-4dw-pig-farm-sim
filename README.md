簡短說明：ROS2 Humble + Gazebo 模擬環境，用於四輪機器人的 Gazebo 模擬、感測器融合、SLAM 與導航。

主要世界場景
-----------
目前使用的豬舍世界檔為 [worlds/pig_pen_8units(lv4).world](worlds/pig_pen_8units(lv4).world)，共 8 間單獨豬舍：
- 上排 4 間：`pen1` ~ `pen4`
- 下排 4 間：`pen5` ~ `pen8`
- 每間包含：地面、牆面、飼料桶等

快速開始
-------

**推薦方式：使用 `launch_clean.sh` 一鍵啟動**（包括 Gazebo、bridge、depth 管道、RViz）

```bash
cd /home/vito/Desktop/4wd
./scripts/launch_clean.sh
```

此腳本會自動啟動：
- Gazebo world（豬舍場景）
- 機器人 URDF spawn
- ROS 2 bridge（topics：`/scan`、`/camera/depth/image_raw`、`/camera/depth/camera_info`、`/odom`、`/cmd_vel`）
- 深度點雲轉換（depth_image_proc）
- 點雲 QoS relay（BEST_EFFORT → RELIABLE）
- RViz 配置預覽

### 手動場景啟動（若需更多控制）

1. 只啟動 Gazebo + 機器人（不含感測器/RViz）：
```bash
./scripts/launch_pig_pen.sh
```

2. 手動 spawn 特定 URDF：
```bash
./scripts/spawn_robot.sh --file turn_on_wheeltec_robot/urdf/four_wheel_diff_bs_robot.urdf --model wheeltec_mini --pos -7 0 0.01 --yaw 0
```

### SLAM / 導航範例

若要手動啟動 SLAM（假設已安裝 slam_toolbox）：
```bash
ros2 launch slam_toolbox online_async_launch.py
```

手動遙控機器人移動：
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/cmd_vel
```

串口控制 (STM32) — 實體底盤
---------------------------
本專案提供與 STM32 底盤兼容的串口控制橋接。主要要點：

- 波特率：115200, 資料位 8, 停止位 1, 無校驗 (115200 8N1)
- 串口：機器人板上 ROS 控制預設使用串口 3（實際請以硬體標示為準）
- 下行封包長度：11 bytes，格式如下：

	[0]  0x7B (帧頭)
	[1]  預留 (0x00)
	[2]  預留 (0x00)
	[3]  X MSB
	[4]  X LSB
	[5]  Y MSB
	[6]  Y LSB
	[7]  Z MSB
	[8]  Z LSB
	[9]  校驗 (前 9 bytes XOR)
	[10] 0x7D (帧尾)

- 資料型態與單位：
	- X, Y：有號 16-bit short，單位 mm/s（big-endian，高位先傳）
	- Z：有號 16-bit short，等於 angular_z (rad/s) * 1000（big-endian）

- 範例（向前 100 mm/s）：
	- 十六進位：7B 00 00 00 64 00 00 00 00 1F 7D
	- 其中校驗 0x1F = XOR(前 9 bytes)

使用專案提供的橋接程式
---------------------
我已新增腳本 `scripts/cmdvel_to_stm32.py`，功能：訂閱 ROS2 的 `/cmd_vel`（`geometry_msgs/Twist`），將速度轉為手冊封包並寫入指定序列埠。使用方式：

1. 安裝依賴（若環境尚未提供）：
```bash
pip install pyserial
```

2. 範例啟動（假設序列埠為 `/dev/ttyUSB0`）：
```bash
# 在已啟動 ROS2 環境下
python3 scripts/cmdvel_to_stm32.py --ros-args -p port:=/dev/ttyUSB0 -p baudrate:=115200
```

3. 測試發佈 `cmd_vel`：
```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {x:0.0, y:0.0, z:0.0}}" -r 10
```

安全提示：在第一次下發命令前，建議將底盤懸空或降低速度，並確認序列埠正確，避免車體意外移動造成損傷。

若要改為直接用 PX4 / MAVLink 控制，請參考上方的 `START_PX4` 範例與 PX4 啟動流程。

輪子速度橋接
-------------
如果你是要把 `/cmd_vel` 轉成左右輪轉速，則可使用 `scripts/cmdvel_to_wheels.py`。這支程式會依差速車運動學，把線速度與角速度換算成左右輪的角速度，再發布到：

- `/wheeltec_mini/left_wheel_velocity`
- `/wheeltec_mini/right_wheel_velocity`

這個版本適合搭配模擬器或輪子控制介面使用，不需要直接寫 STM32 序列封包。

里程計轉接
----------
程式 `scripts/republish_odom.py` 用來把 Gazebo 模型發出的里程計訊息轉發到標準 ROS topic。

功用：
- **訂閱**：從 `/model/wheeltec_mini/odometry` 聽（Gazebo 模型里程計）
- **發布**：轉發到 `/odom`（ROS 標準里程計 topic）

這樣做是為了讓其他 ROS 模組（導航、SLAM、定位等）可以直接訂閱通用的 `/odom` topic，而不用特別知道 Gazebo 模型的名稱。

使用方式：
```bash
python3 scripts/republish_odom.py
```

8 單間豬舍世界
----------------
豬舍主世界檔在 [worlds/pig_pen_8units.world](worlds/pig_pen_8units.world)；目前已整理成可讀性較高的排版，並補上註解標示每一間單間與各牆面、飼料桶的用途。

結構概覽：
- 上排 4 間：`pen1` 到 `pen4`
- 下排 4 間：`pen5` 到 `pen8`
- 每間都包含：`floor`、左右牆、前/後牆或半牆、以及 `feeder`

若要手動調整布局，建議先修改這個 world 檔，再重新啟動 Gazebo 檢查間距、牆面與通道是否符合需求。

近期變更與快速驗證
-----------------
- 已修正 `turn_on_wheeltec_robot/urdf/four_wheel_diff_bs_robot.urdf` 中的座標偏移問題：
	- `base_link` 的 visual/collision/inertial 原點由 `-0.2 0 0.16` 調為 `0 0 0.0775`（對應車體離地 4.5 cm、總高 6.5 cm）。
	- `laser_joint` 已為 `0.0 0 0.245`。
	- `depth_camera_joint` 已移到車頭邊緣並置中為 `0.2 0 0.095`。

今日更新：Gazebo / 深度相機 / RViz 整合
--------------------------------
- 車體數據：
	- 車體方形離地 4.5 cm
	- 車體總高 6.5 cm
	- 輪子直徑 15 cm（半徑 7.5 cm）
	- 車體箱體尺寸約為 0.40 m × 0.30 m × 0.065 m
- 新增並整理了乾淨啟動腳本 `scripts/launch_clean.sh`，可一次啟動：
	- Gazebo world
	- `ros_gz_bridge` 的 `/scan`、`/camera`、`/camera_info`、`/odom`、`/cmd_vel` bridge
	- `robot_state_publisher`
	- 深度點雲產生節點 `depth_image_proc/point_cloud_xyz_node`
	- 點雲 QoS relay `scripts/pointcloud_qos_relay.py`
	- RViz 預設配置 `rviz/wheeltec.rviz`
- 已把深度相機與雷達的 TF 外參整理到 `launch_clean.sh` 與 `scripts/launch_with_bridge.sh`，避免感測器 frame 混亂：
	- `base_link -> wheeltec_mini/base_link/depth_camera`
	- `base_link -> wheeltec_mini/base_link/lidar`
- 已固定 RViz 觀看設定：
	- `Fixed Frame: base_link`
	- 深度影像顯示 `/camera/depth/image_raw`
	- 點雲顯示 `/camera/depth/points_reliable`
- 為了讓 RViz 能順利顯示深度點雲，新增 `scripts/pointcloud_qos_relay.py`，把 `BEST_EFFORT` 的 `/camera/depth/points` 轉成 `RELIABLE` 的 `/camera/depth/points_reliable`。
- 目前的建議啟動流程：
	1. 執行 `./scripts/launch_clean.sh`
	2. 打開 `rviz/wheeltec.rviz`
	3. 若需要檢查資料流，先看 `/scan`、`/camera/depth/image_raw`、`/camera/depth/points_reliable` 是否都有訊息

快速驗證
--------

### 檢查 Topics 是否正確發佈

啟動 `launch_clean.sh` 後，驗證核心 topics：
```bash
# 檢查雷達掃描
ros2 topic info -v /scan

# 檢查深度影像
ros2 topic info -v /camera/depth/image_raw

# 檢查相機內参
ros2 topic info -v /camera/depth/camera_info

# 檢查點雲（RELIABLE 版本）
ros2 topic info -v /camera/depth/points_reliable

# 檢查里程計
ros2 topic info -v /odom
```

### 測試速度控制

```bash
# 發送前進指令
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}, angular: {z: 0.0}}"

# 在 RViz 中應能看到機器人移動、點雲與雷達掃描同步更新
```
