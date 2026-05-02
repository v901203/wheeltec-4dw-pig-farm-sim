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

備註
- 若使用 xacro，可先產生 urdf：
```bash
ros2 run xacro xacro robot.urdf.xacro > robot.urdf
```
- 為了方便，建議由 `robot_state_publisher` 發布 `robot_description`，再用 `spawn_robot.sh` 從 topic 載入。

豬舍場景已先做成一個可直接打開的 world，你可以再把豬模型、感測器或導航節點接進去。
