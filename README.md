簡短說明：在 Gazebo（ROS2 Humble）中建制場景、製圖（SLAM）並後續放入 URDF 的流程與範例指令

這次新增的豬舍世界檔在 [worlds/pig_pen.world](worlds/pig_pen.world#L1)，尺寸對應如下：
- 內部可用空間：長 2.89 m、寬 2.75 m
- 水泥牆高：0.81 m
- 欄杆總高：1.22 m
- 門寬：0.59 m
- 門高：0.77 m

先決條件
- 已安裝 ROS2 Humble 與 `ros_gz_sim`、SLAM 套件、遙控套件
- 有可用的雷射掃描與機器人控制 topic（例如 `/scan` 與 `/cmd_vel`）

步驟範例
1) 載入 ROS2 環境
```bash
source /opt/ros/humble/setup.bash
```
2) 啟動 Gazebo（可選 world 檔）
```bash
./scripts/launch_pig_pen.sh          # 直接開啟豬舍場景
```
3) 在 Gazebo 中放入可移動的機器人（或使用模擬的雷射/控制節點），再啟動 SLAM
```bash
# 範例：啟動你選用的 ROS2 SLAM 節點（依實際套件調整）
# 例如 slam_toolbox（若已安裝）
# ros2 launch slam_toolbox online_async_launch.py

# 手動遙控讓機器人移動以建立地圖
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/cmd_vel
```
4) 儲存地圖
```bash
ros2 run nav2_map_server map_saver_cli -f ~/my_map
```
5) 後續將 URDF 放入 Gazebo
```bash
# 若已在 ROS2 中發布 robot_description topic：
./scripts/spawn_robot.sh --model mybot --pos 0 0 0 --yaw 0

# 或直接從檔案載入：
./scripts/spawn_robot.sh --file path/to/robot.urdf --model mybot --pos 0 0 0 --yaw 0
```

預設預先載入的測試機器人
-----------------
- 本專案 `scripts/launch_pig_pen.sh` 預設會嘗試 spawn 檔案：
	- `turn_on_wheeltec_robot/urdf/four_wheel_diff_bs_robot.urdf`
- 預設啟動規則：
	- 只會開 Gazebo 與這台自訂四輪車，不會自動啟動 PX4 預設的 `gz_rover_differential`
	- 若要同時啟動 PX4 / MicroXRCEAgent，請明確加上 `START_PX4=1`
- 如需改回其他模型，請編輯 `scripts/launch_pig_pen.sh` 中的 `DEFAULT_URDF`，或在執行 `spawn_robot.sh` 時用 `--file` 指定。

範例（只啟動 Gazebo + 自訂四輪車）:
```bash
./scripts/launch_pig_pen.sh
```

範例（同時啟動 PX4 / MicroXRCEAgent）:
```bash
START_PX4=1 ./scripts/launch_pig_pen.sh
```

範例（使用專案內的四輪差速車）：
```bash
# 手動 spawn（等 Gazebo 啟動後執行）
./scripts/spawn_robot.sh --file turn_on_wheeltec_robot/urdf/four_wheel_diff_bs_robot.urdf --model my_four_wheel --pos 0 0 0 --yaw 0
```

備註
- 若使用 xacro，可先產生 urdf：
```bash
ros2 run xacro xacro robot.urdf.xacro > robot.urdf
```
- 為了方便，建議由 `robot_state_publisher` 發布 `robot_description`，再用 `spawn_robot.sh` 從 topic 載入。

豬舍場景已先做成一個可直接打開的 world，你可以再把豬模型、感測器或導航節點接進去。

串口控制 (STM32)
-----------------
本專案提供與我司 STM32 底盤兼容的串口控制橋接說明，參考手冊「串口控制 (ROS 控制模式)」。主要要點：

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

近期變更與快速驗證
-----------------
- 已修正 `turn_on_wheeltec_robot/urdf/four_wheel_diff_bs_robot.urdf` 中的座標偏移問題：
	- `base_link` 的 visual/collision/inertial 原點由 `-0.2 0 0.16` 調為 `0 0 0.16`（車殼回到車體中心）。
	- `laser_joint` 已為 `0.0 0 0.26`。
	- `depth_camera_joint` 已移到車頭邊緣並置中為 `0.37 0 0.23`。

快速驗證（模擬）
- 從專案根目錄啟動 world 與 spawn（注意路徑）：
```bash
cd /home/vito/Desktop/4wd
./scripts/launch_pig_pen.sh
```
- 啟 `ros_gz_bridge` 範例（橋接 `/cmd_vel`、相機、odom）：
```bash
ros2 run ros_gz_bridge parameter_bridge \
	/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist \
	/model/wheeltec_mini/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry \
	/camera/color/image_raw@sensor_msgs/msg/Image@gz.msgs.Image \
	/camera/depth/image_raw@sensor_msgs/msg/Image@gz.msgs.Image
```
- 發送測試指令並觀察：
```bash
# 發送 cmd_vel (0.2 m/s, 5 秒)
timeout 5s ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}, angular: {z: 0.0}}"

# 檢查 Gazebo model 位移
gz model -m wheeltec_mini -p

# 或檢查 odom
ros2 topic echo /model/wheeltec_mini/odometry
```

如需我幫你跑一次驗證測試（自動發 /cmd_vel、擷取 `gz model -p` 與 odom 範例），告訴我即可開始。
