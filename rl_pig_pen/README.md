# LiDAR 傳統直行與巡邏控制

目前可執行的入口是 [fsm_patrol.py](fsm_patrol.py)：讀取 LiDAR，使用比例控制修正車身方向與左右置中，前方障礙過近時停車。控制器不讀取底盤 odom／IMU，也不需要載入模型。

舊 PPO／BC 訓練、評估程式及其專用測試已移除。原有的模型、日誌與 `waypoints.json` 不會被目前控制器載入。

原本的 `fsm_patrol.py` 保持直行功能。新增的 [fsm_lidar_patrol.py](fsm_lidar_patrol.py) 提供傳統演算法完整巡邏，兩個入口擇一執行。

## 完整巡邏（新增獨立入口）

巡邏順序為：主幹道前進 → 路口置中 → 左轉 90° → 左支線前進到底 → 保持車頭方向倒退，穿過路口直到右支線底端 → 前進返回路口中心 → 右轉回主幹道 → 前往下一個路口。主幹道前方出現封牆時停車。

路口數使用設定檔的 `junctions`（目前為 3）。完成全部路口並回到主幹道後，進入 `FINAL_MAIN`：「支線巡邏完成，沿主幹道前進至終點」。此狀態不再辨識或進入新支線，仍須由 LiDAR 連續確認前方終點牆才進入 `DONE`；不會只因支線數達標就原地結束。若路口／端點尚未巡完就遇到主幹道封牆，停車並回報 `main_end_before_patrol_complete`，不算成功完成。更換世界時需同步確認路口數設定。

先依下節啟動 Gazebo，建議使用 `--lidar-only --rtf 1`。車輛應從預設起點、面朝主幹道前進方向開始。另一個終端執行：

```bash
cd /home/an/Desktop/4wd
source /opt/ros/humble/setup.bash
/usr/bin/python3 rl_pig_pen/fsm_lidar_patrol.py
```

執行前先停止 `fsm_patrol.py` 或鍵盤控制，確保只有一個速度發布者。巡邏控制不會自動重設車子；中途失敗後，需先確認原因並將車子恢復至適當的主幹道起始位置，再重新啟動。

此入口使用 [lidar_patrol_controller.py](lidar_patrol_controller.py) 管理路線，並使用現有路口幾何與牆線工具：

- 主幹道與支線以局部平行牆面修正方向、左右置中；倒車時反轉置中修正方向。
- 路口以四面牆的開口估計中心；90° 轉彎追蹤同一條進入走道的方向，避免誤選垂直走道。
- 首次確認路口仍需四面牆的開口。確認後，若完整開口辨識暫時失敗，可利用已量到的走道寬度及至少三面、分屬兩個方向的可見牆線追蹤中心；只有平行牆、配對不明確或位置／方向跳變時仍停止。追蹤不使用命令速度推算位置。
- 完整開口辨識若出現位置或方向跳變，不直接覆蓋原本路口；先以同一筆掃描的牆線驗證既有追蹤。若牆線結果連續且通過原安全檢查便採用，否則停止。時間中斷不能用這個機制略過。
- 置中接近目標時降低速度；若小幅越過中心，在 `junction_turn_centre_limit`（預設 0.15 m）範圍內以最高 0.05 m/s 倒退修正，仍檢查後方淨空。回到中心容許範圍且朝向對準、連續確認後才開始轉彎；超出回正範圍會停止。
- 支線端點需同時符合近距離與橫向牆線條件；路口出入以幾何位置確認。
- 不讀取 odom／IMU、世界座標或路徑點，也不使用命令速度積分或固定秒數判定動作完成。
- 僅處理新時間戳的掃描；重複資料停止輸出運動。接收中斷超過 0.5 秒、時間倒退、追蹤跳變或障礙淨空不足時，進入鎖定停止狀態。
- 看不清走道或路口時先停車，持續無法完成狀態會判定失敗。計時器只用於資料監測／失敗保護，不用來判定轉彎完成。

若出現 `obstacle_clearance`，下一行「停止診斷」會保留失敗前狀態、速度命令、觸發雷達點的距離／角度及安全門檻。`footprint` 表示車體範圍內有障礙點，`travel` 表示行進方向煞停空間不足，`rotation` 表示旋轉淨空不足；`raw_valid=false` 則表示該點源於無效量測。停止後新讀到的掃描未必能代表觸發當下，請保留這兩行完整日誌。

若出現 `junction_geometry_lost`，診斷會列出幾何資料中斷秒數、最後一次有效中心位置及方向。可用幾何的來源會標示為 `four_openings`（完整開口辨識）或 `tracked_walls`（已確認路口的牆面追蹤）。

若新辨識被拒絕，`rejected_geometry` 會保留前後中心座標、估計位移／轉角及門檻。成功改用當前牆線時來源為 `tracked_walls_after_rejection`；若牆線也無法支持連續位置，仍會回報 `junction_position_jump` 或 `junction_axis_jump`。

目前以世界檔的靜態射線與理想差速運動完成 3 個路口、6 個支線端點至主幹道終點的測試；這不等同於 Gazebo 物理模擬、動態豬隻或實車驗收。巡邏假設有可辨識的直角十字路口與封閉端牆，不具備繞過障礙或動態改道功能。

## 啟動 Gazebo 與控制器

以下指令以本機專案位置 `/home/an/Desktop/4wd` 為例。需要 ROS 2 Humble、Gazebo Fortress、ROS–Gazebo bridge，以及能匯入 `rclpy`、NumPy 和 ROS 訊息套件的 Python。現有直行控制不需要 PyTorch、Gymnasium 或 Stable-Baselines3。

終端 1：開啟 Gazebo 視窗、世界與車子。

```bash
cd /home/an/Desktop/4wd
bash scripts/launch_clean.sh
```

預設世界為 `worlds/pig_pen_16units.world`，機器人使用 `turn_on_wheeltec_robot/urdf/four_wheel_diff_bs_robot.urdf`。啟動腳本會建立 `/scan`、`/cmd_vel` 等橋接，但不會自動啟動直行控制器。

若只需要 LiDAR，可改用下列指令；仍會開啟 Gazebo 視窗，並停用相機相關功能：

```bash
bash scripts/launch_clean.sh --lidar-only
```

`--headless` 才會關閉 Gazebo 視窗。上述兩種啟動方式擇一使用。

終端 2：在模擬就緒後啟動直行控制。

```bash
cd /home/an/Desktop/4wd
source /opt/ros/humble/setup.bash
/usr/bin/python3 rl_pig_pen/fsm_patrol.py
```

收到第一筆掃描後，控制器會開始發布速度。請讓同一個 ROS domain 中只有一個行車控制器發布 `/cmd_vel`，避免和遙控或其他控制節點互相覆蓋指令。

結束時先在控制器終端按 `Ctrl-C`，程式會發布一次零速度並退出；再停止 `launch_clean.sh`，由它清理模擬子程序。

舊的 `scripts/launch_rl_training.sh` 保留在工作區，但它引用的 PPO 程式已刪除，目前不能用來啟動訓練。請使用 `launch_clean.sh` 開啟模擬。

## 控制方式與參數

控制器訂閱 `/scan`（`sensor_msgs/msg/LaserScan`），發布 `/cmd_vel`（`geometry_msgs/msg/Twist`）。

每次控制迴圈執行以下判斷：

1. 尚未收到掃描時，等待資料。
2. 前方 ±30° 扇區的最短距離小於 0.55 m 時，發布零速度。
3. 取左前 35°～65°、右前 −65°～−35° 扇區的最短距離，計算左右距離差。
4. 將牆線平行誤差與左右距離差組合成角速度，配合設定的巡航速度前進。

```text
center_error = forward_left - forward_right
angular_z = clip(1.4 × wall_parallel_error + 1.0 × center_error, -0.3, 0.3)
linear_x = clip(cruise_speed, 0, MAX_LIN)
```

平行誤差以弧度表示，距離差以公尺表示。牆線由 [lidar_geometry.py](lidar_geometry.py) 分別擬合，再配對平行牆面。

| 項目 | 目前值 | 設定位置 |
| --- | --- | --- |
| 巡航速度 `cruise_speed` | 0.2 m/s | [patrol_config.json](patrol_config.json) |
| 控制週期 `control_dt` | 1/12 秒，約 12 Hz | `patrol_config.json` |
| LiDAR 安裝偏角 `lidar_yaw` | 0 rad | `patrol_config.json` |
| 平行修正增益 `kp_heading` | 1.4 | `fsm_patrol.py` |
| 置中修正增益 `kp_center` | 1.0 | `fsm_patrol.py` |
| 直行時角速度上限 | ±0.3 rad/s | `fsm_patrol.py`，另受 `MAX_ANG` 限制 |
| 前方停車門檻 | 0.55 m | `fsm_patrol.py` |
| 共用速度上限 `MAX_LIN`／`MAX_ANG` | 0.5 m/s／0.6 rad/s | [navigation.py](navigation.py) |

設定檔固定從 `navigation.py` 同目錄的 `patrol_config.json` 載入；目前入口沒有自訂 `--config` 參數。修改後需重啟控制器。

`control_dt` 用於 ROS timer 排程。現有程式會讀取最新快取掃描，未做到每筆新掃描只控制一次，也沒有以計時器判定轉彎或路口置中完成的流程。

JSON 仍保留 `junctions`、`start`、`turn_speed` 等舊巡邏欄位。它們仍接受共用設定驗證，但不會讓目前直行節點執行巡邏、重設車子或轉彎。Gazebo 出生位置由 `launch_clean.sh` 的生成指令設定。

## 雷達診斷

在已啟動模擬的情況下，可另開終端執行：

```bash
cd /home/an/Desktop/4wd
source /opt/ros/humble/setup.bash
/usr/bin/python3 rl_pig_pen/check_lidar.py
```

[check_lidar.py](check_lidar.py) 收到掃描後會列出前方距離、左右邊界距離、平行誤差與擬合信心度，接著退出。它不發布速度指令。

也可檢查掃描是否持續更新：

```bash
ros2 topic hz /scan
```

若啟動模擬時使用 `--env-id N`，控制器與診斷終端都需設定相同的 `ROS_DOMAIN_ID=N`。若找不到 `rclpy`，確認已載入 ROS 環境，並使用上述 `/usr/bin/python3`。

Gazebo 啟動日誌位於專案的 `logs/`，可查看 `gz_sim_server.log`、`gz_sim_gui.log`、`spawn.log` 和 `ros_gz_bridge.log`；使用 `--env-id N` 時改存於 `logs/env_N/`。

## 現有檔案與功能範圍

| 檔案 | 用途 |
| --- | --- |
| [fsm_patrol.py](fsm_patrol.py) | 可執行的傳統直行控制節點 |
| [fsm_lidar_patrol.py](fsm_lidar_patrol.py) | 獨立的完整巡邏 ROS 節點及掃描失聯監測 |
| [lidar_patrol_controller.py](lidar_patrol_controller.py) | LiDAR 路線狀態機、前進／倒車／路口閉迴路控制 |
| [check_lidar.py](check_lidar.py) | LiDAR 特徵診斷 |
| [navigation.py](navigation.py) | 設定、掃描特徵及共用工具；仍含未由直行節點使用的舊獎勵／安全函式 |
| [lidar_geometry.py](lidar_geometry.py) | 牆線擬合與走道方向估計 |
| [junction_localization.py](junction_localization.py) | 路口中心與方向追蹤，供新巡邏控制器使用 |
| [patrol_config.json](patrol_config.json) | 共用設定 |
| [tests/lidar_scene.py](tests/lidar_scene.py) | 世界檔靜態碰撞幾何的測試射線工具 |
| [tests/test_training_scene.py](tests/test_training_scene.py) | 驗證啟動用暫存場景、LiDAR 與碰撞幾何的保留方式 |

原本直行腳本的停車判斷只使用前方距離門檻，並未呼叫 `navigation.safe_command()` 或完整車體碰撞檢查，也沒有掃描過期檢查；前方障礙離開後會恢復前進。上述限制描述的是原本直行腳本，新巡邏入口另有淨空與資料失聯保護。

保留的測試可從專案根目錄執行：

```bash
source /opt/ros/humble/setup.bash
OPENBLAS_NUM_THREADS=1 /usr/bin/python3 -B -m unittest discover -s rl_pig_pen/tests -v
```

測試包含場景準備、靜態雷達完整巡邏、初始位置偏差、修正方向及感測異常停止。ROS 節點測試使用模擬發布者，不會發送車輛命令；未載入 ROS 環境時會略過該部分。

## 後續驗證方向

目前以傳統閉迴路控制及 FSM 完成路線邏輯，接下來需在 Gazebo 實測滑動、延遲、感測雜訊與動態障礙對控制的影響，再進行 Xavier／實體車驗證。

RL 轉彎仍未實作，可在傳統控制基準完成實測後，依需求評估。
