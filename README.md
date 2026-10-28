# Wheeltec 4WD 豬舍模擬與 LiDAR 控制

本專案提供 ROS 2 Humble／Gazebo Ignition Fortress 豬舍模擬場景、四輪差速機器人模型，以及 LiDAR 傳統走道控制程式。

目前可使用 `scripts/launch_clean.sh` 開啟 Gazebo，再獨立執行 `rl_pig_pen/fsm_patrol.py` 進行走道直行。控制器僅讀取 `/scan` 並發布 `/cmd_vel`，不依賴底盤 odom／IMU，也不需要載入模型。

完整巡邏入口為 `rl_pig_pen/fsm_lidar_patrol.py`，以傳統控制完成左支線前進到底、倒退穿過路口至右端、前進回中心、轉回主幹道並繼續下一個路口的循環，直到主幹道終點。原本直行腳本維持不變。

完成設定的 3 個路口後，進入 `FINAL_MAIN`，只沿主幹道行駛並確認終點牆，不再接受第四個支線入口；確認終點後才停止並回報 `DONE`。

一般走道也支援障礙反方向退避：前進受阻時低速倒車，倒車受阻時低速前進。反方向安全連續確認後才移動，淨空恢復或監測逾時就停下；左右支線在保護停車期間仍辨識端牆，確認端點後自動切換返回。退避不是繞障，反方向也受阻時仍等待。

截至 2026-09-30，控制器以 5 cm 作為理想置中目標，進入目標走道採「路口控制 → 短暫加權融合 → 牆面確認」切換。深度與成對牆擬合條件首次滿足後解除舊路口追蹤，再由牆面連續確認並低速邊走邊修正；交接不再以仍可能穿過路口開口的固定正側方射線重複否決牆線結果。前進／倒車中的方向修正使用弧形車身掃掠檢查，原地轉向仍使用完整外接圓保護。一般障礙會先停車，反方向安全且符合退避條件時進行一次有界低速退避，否則等待；障礙移開後自動恢復。LiDAR 短暫中斷可在預設 3 秒期限內經三筆有效掃描恢復。本次融合交接修改依使用者要求未執行測試，待自行驗證及 Gazebo 完整重跑。整趟完成的驗收條件是 `DONE`、3 個完成路口、6 個已到達端點及終點停車。

## 目前功能範圍

| 功能 | 狀態 |
| --- | --- |
| Gazebo 世界、車子生成、LiDAR 與 ROS 橋接 | 保留啟動流程 |
| 相機、深度點雲、RViz 與豬隻行為 | 保留相關腳本，依啟動選項載入 |
| LiDAR 走道平行修正、左右置中、前方過近停車 | 現有直行控制器 |
| 支線巡邏、倒車、路口轉 90° | 已實作；使用者回報 Gazebo 路口測試完成 |
| 主幹道末端與禁止第四個支線入口 | 已實作，相關自動測試通過；現場驗收需確認最終 `DONE` |
| 遮擋追蹤、越位回正與辨識跳變處理 | 已實作並有回歸測試，仍需重複運行統計 |
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

收到第一筆掃描後，車子便會開始控制。預設巡航速度為 0.2 m/s，直行修正角速度上限為 ±0.3 rad/s；前方 ±10° 內的最短距離小於 0.55 m 時停車，障礙離開後會恢復前進。

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

### 與攝影機／機械手臂協作

完整巡邏新增兩個 ROS topic；豬臉辨識與拍攝方向由對方程式負責，本車只發布路線狀態、接受停車請求。

| Topic | 方向與型別 | 用途 |
| --- | --- | --- |
| `/patrol/state` | 本車發布 `std_msgs/msg/String` | 現有 FSM 英文狀態，例如 `MAIN`、`LEFT_OUTBOUND`、`RIGHT_OUTBOUND`；切換時發布，並每 0.5 秒重送 |
| `/pig_face/stop_requested` | 本車訂閱 `std_msgs/msg/Bool` | 對方持續以 5～10 Hz 發布；`true` 請求停車、`false` 解除請求 |

主幹道／支線一般直行或倒車時接受拍攝停車；接近路口、已取得有效路口幾何並開始減速／停車確認後，就延後接受拍攝停車，即使對外狀態仍是 `MAIN`、`REVERSE_LEFT` 或 `RETURN_RIGHT`。置中、轉彎及進出路口期間也維持延後，完成交接、恢復一般直線控制後再依當時請求判斷。停車後保留路線進度；連續收到 `false` 至少 1.5 秒，且新 LiDAR 幾何確認完成後才允許繼續，障礙與雷達保護仍有效。**路段狀態不等於可接受停車或已停穩的回報**，不能拿來授權機械手臂伸入車體移動範圍。

目前暫時停用「等待攝影機／攝影機失聯停車」（`vision_required=false`）：預設直接依路線巡邏，沒有訊息或訊息中斷不會額外觸發停車。**停車指令仍有效**；收到 `true` 後依上述狀態規則停車，之後失聯不會自行解除，仍需持續 `false` 與雷達確認。直接用原本命令啟動即可：

```bash
/usr/bin/python3 rl_pig_pen/fsm_lidar_patrol.py
```

日後要恢復攝影機失聯保護，可加 `--ros-args -p vision_required:=true`。此參數現在只控制是否強制要求攝影機心跳，不會關閉停車指令。原本 `fsm_patrol.py` 直行腳本與 UART bridge 不受影響。狀態對照、QoS 與聯調步驟見 [攝影機協作介面](rl_pig_pen/README.md#攝影機協作介面)。

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

實體 LiDAR 曾完成 60 秒靜止量測，共取得 721 幀；四個角柱的原遮罩總寬 43°。目前依使用者要求保留各段中心、每段擴為 15°：車體角度 `[30.5°,45.5°]`、`[128.5°,143.5°]`、`[-147°,-132°]`、`[-43.5°,-28.5°]`，總盲區 60°，已達設定上限。以 `0°～360°` 表示則為 `30.5°～45.5°`、`128.5°～143.5°`、`213°～228°`、`316.5°～331.5°`。這是待實車核對的設定，尚未證實障礙來源；需對照診斷的 `angle_deg`／`range_m`，不能藉此忽略真正的前方障礙。

這些射線不參與近障礙、牆線、路口或行車淨空判斷；本次未修改安全距離及辨識門檻，前後與左右側主要量測區仍保留，但不能保證斜向點減少完全不影響幾何辨識。安全車身尺寸仍須包含立柱外緣，盲區內障礙不能保證被偵測。若改變雷達或立柱安裝位置，必須重新量測，不能直接沿用。詳見 [角柱遮罩設定與實車確認](rl_pig_pen/README.md)。

## Xavier 實車啟動入口

新增可選的 `start-modes` 入口：巡邏 AUTO／手控 MANUAL／停止 STOP，啟動預設 STOP，
以 ROS 2 服務切模式並由 `/manual/cmd_vel` 接收手控指令。不包含網頁；原本 `start-patrol`
維持原行為。介面、部署與操作詳見 [三模式控制說明](rl_pig_pen/README.md#可選的三模式控制巡邏手控停止)。
巡邏途中可請求切手控，但雷達或車體淨空不符合時會保持停止；進入手控後，返回自動
需要回起點明確重設。詳見 [切手控的注意事項與指令順序](rl_pig_pen/README.md#巡邏途中切換手控條件與操作順序)，
包含 Gazebo／Xavier 操作環境、八種移動組合、STOP 與重新開始巡邏的指令。

使用 [scripts/start_robot.sh](scripts/start_robot.sh)，在 **Xavier 主機**操作既有的
`robot-dev` 容器。完整巡邏使用 `fsm_lidar_patrol.py → /cmd_vel → wheeltec_uart_bridge`，
不再啟動舊 `mecanum_driver`／`obstacle_avoidance`。腳本明確載入
`/workspace/pig_patrol/wheeltec_uart_bridge/config/uart_bridge.yaml`，並顯示巡邏設定與實際限速。
目前採用板上版本的掛載配置：`/home/yu/Desktop/pig_patrol:/workspace/pig_patrol`，
可用 `ROBOT_PROJECT_CONTAINER` 指定其他專案位置；workspace 仍是 `/workspace`。

先依 [部署與啟動說明](wheeltec_uart_bridge/README.md#新版-xavier-啟動腳本2026-10-15)
搬入原始碼並在容器內編譯 UART bridge。使用者提供的 Dockerfile 只包含雷達驅動，
尚未包含本巡邏程式與 UART bridge；原 Dockerfile 本次未修改。

```bash
# 在 Xavier 主機，路徑依實際搬移位置調整
bash /home/yu/ros2_ws/start_robot.sh status
bash /home/yu/ros2_ws/start_robot.sh test-safe
# 下列指令會啟動實際巡邏，先確認急停與測試區安全
bash /home/yu/ros2_ws/start_robot.sh start-patrol
# 另一終端請求停止本腳本啟動的控制程序
bash /home/yu/ros2_ws/start_robot.sh stop
```

`test-safe` 的速度輸出改到 `/test_cmd_vel`，不啟動 bridge；仍需隔離馬達，防止其他
訂閱者使用測試指令。`stop` 嘗試先停 FSM、送零速度，再關閉 bridge，不直接停止
Docker，且不會停止舊／手動啟動的程序。它沒有實體停穩回授，不能取代急停。
巡邏與 bridge 日誌存於掛載 workspace 的 `logs/robot/`，可檢查「障礙診斷」。

### 開機自動設定雷達網卡

將本專案新版 `scripts/start_robot.sh` 複製到 Xavier 的
`/home/yu/ros2_ws/start_robot.sh`。`start`、`test-safe`、`start-patrol` 會先確認容器使用
host 網路，再設定主機雷達網卡；`status`、`stop`、`start-base` 不設定網卡。
新增的 `setup-network` 僅設定主機網路，不需要 Docker，也不啟動雷達或車子。

預設使用 `eth0`、主機 `192.168.1.100/24`，並保留雷達舊目的位址
`192.168.1.102/24`，雷達本身預期為 `192.168.1.200`。腳本最多等待 30 秒讓網卡及實體
連線就緒，只補上缺少的 IP，不刪除既有位址、不變更預設路由，也不修改雷達內部 IP。
雷達路由若未經指定網卡，會停止啟動並顯示原因。這是在每次啟動時重新套用，並非修改
NetworkManager 的永久設定；仍須確認網路管理服務不會在稍後移除這些位址。

可先單獨測試，這個指令不會控制車子：

```bash
sudo /usr/bin/bash /home/yu/ros2_ws/start_robot.sh setup-network
```

既有 `pig-pen-patrol.service` 使用 `User=yu`，開機時無法互動輸入 sudo 密碼。
在 Xavier 執行 `sudo systemctl edit pig-pen-patrol.service`，新增：

```ini
[Service]
ExecStartPre=+/usr/bin/bash /home/yu/ros2_ws/start_robot.sh setup-network
```

這行會追加在原有 Docker 啟動前置命令之後、巡邏主程序之前；不要加入空白的
`ExecStartPre=` 清掉原有命令。`+` 只讓這個網路前置步驟以 root 執行，巡邏仍使用
原本的 `User=yu`，不需要設定免密碼 sudo。修改後執行：

```bash
sudo systemctl daemon-reload
systemctl cat pig-pen-patrol.service
```

準備好測試區後才執行 `sudo systemctl restart pig-pen-patrol.service`，因為這會啟動
完整巡邏。檢查 `journalctl -u pig-pen-patrol.service -b -n 100 --no-pager`，應先看到
「雷達網路已就緒」，接著還要確認實際收到 LaserScan；網卡設定成功不代表雷達已送資料。
外部停車／放行行為由 `vision_required` 決定，本次不修改這項設定。

非預設環境可在同一個 systemd override 的 `[Service]` 加入 `Environment=`：

| 變數 | 預設值 | 用途 |
| --- | --- | --- |
| `RADAR_INTERFACE` | `eth0` | 雷達連接的實體網卡 |
| `RADAR_HOST_IP` | `192.168.1.100/24` | 主機接收位址 |
| `RADAR_COMPAT_IP` | `192.168.1.102/24` | 相容舊目的位址；明確設為空字串可略過新增，但不會移除已有位址 |
| `RADAR_DEVICE_IP` | `192.168.1.200` | 用於核對主機路由，不會改寫驅動或雷達設定 |
| `RADAR_LINK_WAIT_SEC` | `30` | 等待網卡與實體連線的秒數，範圍 0～300 |

例如 `Environment=RADAR_INTERFACE=enp1s0`。手動執行時也可用同名環境變數指定。
若有另一個容器也在執行雷達驅動，仍須另行處理；本腳本不會自動停止其他容器。

若先前版本顯示「已有本腳本的啟動／控制工作執行中」，但啟動程序已退出，可能是
ROS CLI daemon 繼承了 `control.lock` 的 fd 9。新版會在所有 ROS CLI 呼叫（含 timeout
包裝）關閉這個描述符，保留主控制程序的互斥鎖。更新腳本不會解除已在執行的舊 daemon
所持有的鎖：先取消等待中的手動 `/cmd_vel` 發布指令、停止巡邏服務，再執行：

```bash
docker exec robot-dev bash -lc '
source /opt/ros/humble/install/setup.bash
source /workspace/install/local_setup.bash
ros2 daemon stop
'
```

這只停止 ROS CLI 的背景查詢 daemon，不會停止雷達節點，也不是底盤停車指令。
不要刪除控制鎖檔或取消互斥檢查。清除舊 daemon 後，可先用 `start-base` 驗證 UART
bridge 開啟成功，再單獨測試底盤。這個修正解決鎖殘留，不代表雷達 UDP 或 `/x10/scan`
收不到資料的問題也已排除。

## 鐳神 M10P-PHY 實體連接

本節依專案內的 [《鐳神智能 M10P-PHY 使用者手冊 V1.0.2》](<1. 镭神智能_M10P-PHY_用户手册_V1.0.2_20250311.pdf>) 整理。M10P-PHY 是 360°、12 Hz、20,000 點／秒的乙太網路 LiDAR；它不是 USB 雷達，資料不會出現在 `/dev/ttyUSB*`。實車需分別連接電源與有線網路，再由 ROS 驅動把 UDP 資料轉成 `/scan`。

### 接頭版本與接線

手冊列出兩種出線形式：帶 GPS 的 M12 A-code 8-pin 公頭，以及不帶 GPS 的一分二出線版本。線材可客製，手冊也提醒客製線可能與標準顏色／定義不同；接線前應核對實際線材標籤與供應商資料，不能只看線色猜測。

M12 A-code 8-pin 標準定義如下：

| Pin | 訊號／標準線色 | 用途 |
| --- | --- | --- |
| 1 | `VIN`／紅 | 電源正極 |
| 2 | `TD_P`／白 | 雷達送往外部設備的 Ethernet TX+ |
| 3 | `TD_N`／綠 | 雷達送往外部設備的 Ethernet TX− |
| 4 | `RD_P`／棕 | 外部設備送往雷達的 Ethernet RX+ |
| 5 | `RD_N`／藍 | 外部設備送往雷達的 Ethernet RX− |
| 6 | `GPS_PPS`／黃 | GPS 秒脈衝；不用 GPS 時保持未連接 |
| 7 | `GPS_REC`／橙 | GPS 經緯度時間；不用 GPS 時保持未連接 |
| 8 | `GND`／黑 | 電源負極 |
| 外殼 | `PE`／屏蔽 | 屏蔽地／機殼地 |

一分二線版本會分成網路 `CN2` 與電源 `CN3`：`CN2` 使用 Ethernet pin 1=`TD_P`、2=`TD_N`、3=`RD_P`、6=`RD_N`；`CN3` 的 `1+` 是 VIN，`2−` 是 GND。使用原廠分線時，實際操作就是把網路端接到 Xavier 的 Ethernet 網口或同一交換器，電源端接到獨立穩壓電源。

### 供電要求

手冊的規格表與電氣參數表標示 `9–32 V`，典型 12 V、典型電流 200 mA、最大工作電流 350 mA，允許的電壓紋波上限為 80 mV；但接頭腳位表另寫 `9–36 V`。兩處上限不一致，因此本專案採較保守的 **9–32 V**，建議先使用穩壓 12 V，未經原廠書面確認不要使用 32 V 以上電源。

- 關閉電源後再插拔 M12、分線或裸線接頭。
- 不可從 Xavier Ethernet 網口或一般 USB 連接埠直接供電。
- VIN 與 GND 不可接反，建議在電源正極加入適當保險絲與獨立開關。
- PE／屏蔽依整車接地設計接至機殼地，避免把屏蔽線誤當成 VIN 或資料線。
- GPS PPS／REC 不是普通 UART 控制線，不使用時不要接到 Xavier GPIO 或 STM32 UART。

### Xavier 網路設定

手冊標示雷達的出廠預設「目的 IP」為 `192.168.1.102`、UDP 目的埠為 `2368`，因此 Xavier 連接雷達的有線網卡應設成同一目的位址，例如：

```text
Xavier Ethernet IPv4：192.168.1.102/24
UDP 接收埠：2368
```

手冊封包擷取範例顯示資料由 `192.168.1.200:2369` 傳往主機的 `2368`，可把 `192.168.1.200` 當作出廠雷達位址的檢查起點；若設備曾被改過設定，應以實際封包來源與原廠設定工具為準。雷達目的 IP、Xavier IP 與驅動設定必須一致，網路上也不能有另一台設備占用 `192.168.1.102`。第一次修改 IP 後，手冊要求重新啟動雷達電源。

在 Xavier 主機先確認網卡名稱與位址：

```bash
ip -br link
ip -br addr
```

假設雷達網卡是 `eth0`，完成主機的靜態 IP 設定並重新上電後，可依序檢查：

```bash
ping 192.168.1.200
sudo tcpdump -ni eth0 udp port 2368
```

`tcpdump` 能看到持續的 UDP 封包，才表示電源、網線、目的 IP 與網卡基本正常。手冊的 M10 範例封包長度為 92 bytes，但仍應以實際型號與通訊協議為準，不能只靠封包長度判斷所有資料都正確。

### Docker 與 ROS 2 Humble

雷達透過 Ethernet 傳輸，不需要 Docker 的 `--device=/dev/ttyUSB*`；建議讓容器使用主機網路：

```bash
docker run -it --rm --network host <LiDAR/巡邏映像名稱>
```

靜態 IP 應在 Xavier 主機的實體網卡上設定。容器使用 `--network host` 後接收主機 `2368/UDP`；若改用 bridge network，還需另外處理 UDP 埠與目的 IP 路由，不建議用於第一次實車測試。

手冊第 6 章提供的是 ROS 1 `catkin_make`、`roslaunch lslidar_driver lslidar_net.launch` 範例，**不能直接當作 ROS 2 Humble 指令使用**。Xavier Docker 必須安裝或移植支援 M10P-PHY 的 ROS 2 驅動，並確認其參數至少符合雷達 IP、主機／目的 IP、UDP 2368 埠、距離上下限及角度裁切。驅動最後必須發布：

```text
/scan    sensor_msgs/msg/LaserScan
```

啟動巡邏前檢查：

```bash
source /opt/ros/humble/setup.bash
ros2 topic list
ros2 topic hz /scan
ros2 topic echo /scan --once
```

M10P-PHY 規格掃描頻率為 12 Hz，因此 `/scan` 應接近 12 Hz；仍需檢查時間戳是否前進、距離值是否有效、`0°` 是否朝車頭，以及四個角柱是否落在目前設定的忽略區間。確認 `/scan` 穩定後，才啟動 `fsm_lidar_patrol.py`，最後才啟動 UART bridge 讓 `/cmd_vel` 真正送到底盤。

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

已新增三項改善：純左右偏移時低速退開並重新置中、短暫無效雷達量測的安全暫停與連續確認恢復，以及串口接收端的速度指令逾時停止。保留原本直行腳本與 Gazebo 啟動方式；詳見 [改善結果與待驗證項目](rl_pig_pen/README.md#檢查發現與待改善項目)。新增行為有自動測試，但尚未完成新版 Gazebo 物理與實車驗收。

原本直行控制器尚未具備掃描過期停車、低可信度停車、完整車體碰撞檢查及路線完成判斷。新巡邏入口另加入感測失聯、追蹤異常與障礙淨空保護，但仍需實測；目前也沒有避障繞行功能。模擬啟動中的靜態 `odom → base_link` TF 不等同於移動中的定位結果，尚未提供完整驗證的 SLAM／Nav2 啟動流程。

實體部署時，LiDAR 驅動、速度介面、指令逾時停車與急停需要分別確認。串口轉接在上層指令逾時後會持續送零速度，但必須實際使用該程序作為速度接收端；它不會由 Gazebo 啟動腳本自動啟用。Xavier 斷電、轉接程序自身失效或串口斷線，仍需底盤端的獨立保護。

下一階段先在 Gazebo 驗證新版完整路線與故障停止／恢復，再記錄多回合的成功率、完成時間及失敗原因，逐步驗證滑動、延遲與動態障礙的影響，之後再進行 Xavier／實車測試。

## 未來工作：機器學習與碩士論文方向

以下為尚未實作的研究規劃，並非目前已完成的功能。現有 LiDAR 傳統控制與 FSM 將保留作為基準，先統計失敗原因，再選擇一項主要學習任務：

- **優先評估局部控制修正**：以殘差強化學習微調路口置中、90° 轉彎及進入走道時的動作；長距離直行與路線順序仍由傳統方法管理。
- **替代方向為學習調參**：依路寬、感測與行駛狀態選擇速度及修正增益，評估是否減少人工調參。
- **若主要瓶頸是感知**：研究 LiDAR 路口辨識、中心／方向估計與可信度，改善遮擋和假路口；不必強制使用強化學習。

學習模型不能取消碰撞、感測失效及指令逾時保護，部署仍不依賴底盤 odom／IMU。驗證須比較純傳統、規則式自適應與學習方法，包含消融、未見場景、完整路線成功率及 Xavier 延遲量測；現有理想運動測試不能取代 Gazebo 物理與實車驗證。

暫定論文方向為「基於 LiDAR 與狀態機之窄走道巡邏機器人：結合殘差強化學習的路口操作控制」。重點是證明改善條件與限制，不只是加入模型；正式貢獻仍需文獻回顧與實驗確認。詳細架構、訓練與比較規劃及參考研究見 [機器學習未來工作](rl_pig_pen/README.md#未來工作機器學習與碩士論文方向)。
