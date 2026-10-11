# WheelTec ROS 2 UART 轉接節點

本套件適用於 ROS 2 Humble，將 `/cmd_vel` 的
`geometry_msgs/msg/Twist` 速度命令轉換成 WheelTec STM32 使用的
11-byte UART 封包。

本節點只有下行控制功能，不會發布里程計、IMU、輪速或電池資訊。

一般套件使用方式見下方；小車端的既有 Docker、M10P 雷達連接紀錄及待辦，
請見本文末的「Xavier Docker 與 M10P 整合紀錄」。

## 功能

- 訂閱 `/cmd_vel`。
- 將 `linear.x`、`linear.y` 與 `angular.z` 乘以 1000 後轉成有號 16 位元整數。
- 使用 Big-Endian 編碼並計算 XOR checksum。
- 拒絕包含 `NaN` 或 `Inf` 的速度命令。
- 依 ROS 參數限制線速度與角速度。
- 四輪差速底盤預設忽略 `linear.y`，避免要求底盤橫向移動。
- 只有固定頻率 timer 可以寫入串口，避免 callback 與 timer 交錯封包。
- 啟動後先送零速度，收到新的有效 `/cmd_vel` 才允許移動。
- 超過 0.5 秒未收到新命令時，持續傳送零速度。
- 正常關閉及 Ctrl-C 結束時嘗試送出零速度。
- 獨占鎖定串口，避免兩個底盤節點同時寫入。
- UART 寫入失敗後關閉串口並定期重新連線。
- 重新連線後清除舊命令、先送零速度，必須收到重連後的新命令才允許移動。

## 安全限制

若 Xavier 斷電、Docker 被強制終止、USB 線被拔除或 UART 已經中斷，
ROS 2 節點便無法把零速度送到 STM32。因此 STM32 韌體仍須具備獨立的
通訊逾時停止功能，實體車也必須有可直接切斷馬達動力的緊急停止裝置。

第一次測試必須架高車輪，確認前進、後退、左右旋轉、命令逾時及 Ctrl-C
的停止行為後，才可讓車輪接觸地面。測試期間不得讓人或動物進入車輛附近。

## 封包格式

| Byte | 內容 |
| --- | --- |
| 0 | `0x7B` 封包開頭 |
| 1～2 | 保留，固定為 `0x00` |
| 3～4 | X 線速度，mm/s，有號 int16 Big-Endian |
| 5～6 | Y 線速度，mm/s，有號 int16 Big-Endian |
| 7～8 | Z 角速度乘以 1000，有號 int16 Big-Endian |
| 9 | Byte 0～8 的 XOR checksum |
| 10 | `0x7D` 封包結尾 |

串口設定固定為 `115200 Baud、8 Data Bits、No Parity、1 Stop Bit`（8N1）。

## 搬到 Xavier

請搬移完整的 `wheeltec_uart_bridge/` 原始碼資料夾：

```text
wheeltec_uart_bridge/
├── CMakeLists.txt
├── package.xml
├── README.md
├── config/
│   └── uart_bridge.yaml
├── include/wheeltec_uart_bridge/
│   └── packet.hpp
├── launch/
│   └── uart_bridge.launch.py
├── src/
│   ├── packet.cpp
│   └── wheeltec_uart_bridge_node.cpp
└── test/
    └── test_packet.cpp
```

不要搬移桌上型電腦產生的 `build/`、`install/` 或 `log/`。桌上型電腦通常
是 x86_64，而 Xavier 是 ARM64，必須在 Xavier 的 Docker 容器內重新編譯。

從目前電腦傳送到 Xavier：

```bash
scp -r /home/an/Desktop/4wd/wheeltec_uart_bridge \
  nvidia@XAVIER_IP:/home/nvidia/ros2_ws/src/
```

請把 `XAVIER_IP` 改成 Xavier 的實際 IP 位址。

## 在 Xavier Docker 內建置

```bash
source /opt/ros/humble/setup.bash
cd /home/nvidia/ros2_ws
colcon build --packages-select wheeltec_uart_bridge
source install/setup.bash
```

只能搬原始碼，不能直接使用在 x86_64 電腦編譯出的執行檔。

## 設定

預設參數位於 `config/uart_bridge.yaml`：

```yaml
wheeltec_uart_bridge:
  ros__parameters:
    port: /dev/wheeltec_controller
    baudrate: 115200
    command_timeout: 0.5
    transmit_rate: 20.0
    reconnect_interval: 1.0
    max_linear_x: 0.30
    max_linear_y: 0.0
    max_angular_z: 0.60
    reject_linear_y: true
    cmd_vel_topic: /cmd_vel
```

若尚未建立固定 udev 名稱，可以暫時把 `port` 改成實際裝置，例如
`/dev/ttyUSB0`。正式部署建議使用 `/dev/serial/by-id/` 或固定的 udev symlink，
避免重新開機後 LiDAR 與 STM32 的 `ttyUSB` 編號互換。

Docker 啟動時必須將實際串口裝置傳入容器，並確保容器使用者有權限開啟：

```bash
docker run -it --rm \
  --network host \
  --device=/dev/ttyUSB0 \
  <映像名稱>
```

同一時間只能執行一個 UART bridge 或原廠底盤節點。

## 執行

使用設定檔啟動：

```bash
source /opt/ros/humble/setup.bash
source /home/nvidia/ros2_ws/install/setup.bash
ros2 launch wheeltec_uart_bridge uart_bridge.launch.py
```

也可以直接覆寫串口參數：

```bash
ros2 run wheeltec_uart_bridge wheeltec_uart_bridge_node --ros-args \
  -p port:=/dev/ttyUSB0
```

## 架高車輪測試

正向測試：

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.05}, angular: {z: 0.0}}" --once
```

反向測試：

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: -0.05}, angular: {z: 0.0}}" --once
```

測試完成後需確認：

- 正 `linear.x` 是前進，負 `linear.x` 是後退。
- 正負 `angular.z` 的旋轉方向符合 ROS 座標慣例。
- 停止發布超過 0.5 秒後，底盤持續收到零速度並停止。
- Ctrl-C 後底盤停止。
- 拔除並重新接回串口後，不會自動恢復斷線前的移動命令。

## 單元測試

```bash
cd /home/nvidia/ros2_ws
colcon test --packages-select wheeltec_uart_bridge
colcon test-result --verbose
```

目前封包測試涵蓋正值、負值、零速度、二補數 Big-Endian、XOR checksum、
物理速度限制及非有限值拒絕。這些測試不會取代真實 STM32、USB 斷線、
底盤方向及實際停止距離測試。

## Xavier Docker 與 M10P 整合紀錄

本節整理自使用者提供的 `README.md` 與 `建議代辦事項.txt`（2026-10-11）。
「文件回報」表示原文件記錄的結果，不代表本專案已在該 Xavier 上重新驗證。
以下部署與操作指令供人工確認後使用，本次整理沒有執行 Docker、修改網路或啟動馬達。

### 環境與路徑

| 項目 | 文件記錄 |
| --- | --- |
| 既有 Docker image | `dustynv/ros:humble-desktop-l4t-r35.4.1` |
| 另建的 LiDAR image | `lslidar-ros2:humble-m10p`；待辦文件勾選建置成功，仍需確認新容器的可重現啟動流程 |
| Dockerfile | 主機 `/home/yu/ros2_ws/Dockerfile`；未隨這兩份文件提供內容 |
| 主要 workspace 掛載 | 主機 `/home/yu/ros2_ws` → 容器 `/workspace` |
| 舊底盤 workspace 掛載 | 主機 `/home/yu/robot_ws` → 容器 `/robot_ws` |
| 原 LiDAR driver 原始碼 | `/home/yu/robot_ws/src/Lslidar_ROS2_driver` |
| 準備納入 Docker build context 的 driver | `/home/yu/ros2_ws/src/Lslidar_ROS2_driver` |
| 已編譯的主機 LiDAR workspace | `/home/yu/lslidar_ws` |
| 主機 driver 執行檔 | `/home/yu/lslidar_ws/install/lslidar_driver/lib/lslidar_driver/lslidar_driver_node` |
| ROS packages | `lslidar_msgs`、`lslidar_driver` |
| 雷達 | M10P，IP `192.168.1.200` |
| 主機網卡／預期 IP | `eth0`／`192.168.1.100` |
| 雷達目的 IP | 曾確認為 `192.168.1.102`；原文件建議正式改為 `192.168.1.100` |
| 臨時網路配置 | 雷達目的 IP 未改正前，主機 `eth0` 暫時保留 `192.168.1.102` |
| UDP ports | `2368`、`2369`；仍須對照現場 driver 設定 |
| LiDAR topic | `/x10/scan`，文件回報約 12 Hz |
| 底盤串口 | `/dev/ttyACM0`；不同於本套件預設 `/dev/wheeltec_controller` |

上述 `/home/yu/...` 是小車端路徑，不是目前開發電腦的 `/home/an/Desktop/4wd`。
不要直接套用先前範例中的 `/home/nvidia/...`，應依容器實際掛載路徑調整。

### 保留既有容器與環境

原文件記錄保留 `awesome_khayyam` 執行 LiDAR driver，並已停止 `modest_banzai`，
避免兩個 driver 同時占用雷達 UDP ports；`robot-dev` 則是建議的固定開發容器名稱。
這些是當時的紀錄，先用下列命令查看現況，不能假設它們現在仍在執行：

```bash
docker ps -a
```

若 `robot-dev` 已存在，先確認其 image、掛載、網路及裝置設定，再重用：

```bash
docker start robot-dev
docker exec -it robot-dev bash
```

只有名稱不存在、主機路徑與串口都已確認時，才參考下列建立範例：

```bash
docker run -d \
  --name robot-dev \
  --network host \
  --device=/dev/ttyACM0:/dev/ttyACM0 \
  -v /home/yu/ros2_ws:/workspace \
  -v /home/yu/robot_ws:/robot_ws \
  dustynv/ros:humble-desktop-l4t-r35.4.1 \
  tail -f /dev/null
```

此開發容器不加 `--rm`，以保留容器內安裝的工具；掛載目錄內的原始碼與編譯結果
保存在主機。容器保留不等於 image 已包含後來安裝的依賴：正式可重現部署仍應將
driver、套件及依賴寫入 Dockerfile。原文件使用的 `--privileged`、`--ipc host`
未放入上述基本範例；若實際環境需要額外權限或 IPC 設定，應先確認原因再加入。

`docker stop robot-dev` 可停止並保留容器，但**不是硬體急停操作**。
先停止速度發布者、確認車子已停止，再處理容器；不要為了清環境直接刪除既有容器。

### 在同一套 Humble 環境編譯

建議部署位置如下；需先搬入原始碼，並非表示這些檔案已存在於小車：

```text
/home/yu/ros2_ws/                  # 容器內為 /workspace
├── src/
│   ├── Lslidar_ROS2_driver/
│   └── wheeltec_uart_bridge/
└── rl_pig_pen/                    # 巡邏 Python 腳本及設定檔
```

依提供文件，這個 `dustynv` image 的 ROS 環境檔位於
`/opt/ros/humble/install/setup.bash`；其他 Humble 環境可能是
`/opt/ros/humble/setup.bash`。每個容器終端都應 source 實際存在的版本。

```bash
# 在 robot-dev 容器內，依上述部署位置操作
source /opt/ros/humble/install/setup.bash
cd /workspace
colcon build --symlink-install --packages-up-to lslidar_driver wheeltec_uart_bridge
source /workspace/install/setup.bash
```

原文件列出的 driver 開發依賴包括 `build-essential`、`libpcap-dev`、
`libboost-thread-dev`、`libeigen3-dev`、`libpcl-dev`、`libyaml-cpp-dev`
以及 ROS 的 `pcl_conversions`；待辦另列 `libpcl-all-dev`。實際安裝項目應以
所用 driver 的 `package.xml` 與容器發行版為準，不把文件中的清單當成已完成安裝。
原文件也列出 `python3-serial`，但本套件是 C++ 串口程式，不依賴它。

注意 `/robot_ws/install` 可能含有 ROS Foxy 的舊環境設定。若出現
`/opt/ros/foxy/local_setup.bash` 不存在，不要硬接到 Humble；應在 Humble／ARM64
環境重新編譯所需套件，勿混用舊 `build/`、`install/` 或 x86_64 編譯產物。

若 apt 簽章或金鑰報錯，先確認 image 對應的作業系統、ROS repository 與金鑰配置。
原文件中的更新金鑰命令僅是當時排錯紀錄，不在此當作每次部署必做步驟。

### 接入本專案的啟動順序

資料流為：`M10P → lslidar_driver → /x10/scan → 巡邏 FSM → /cmd_vel → UART bridge → STM32`。
攝影機協作另由 FSM 的 `/patrol/state` 與 `/pig_face/stop_requested` 處理，
不需修改 UART 封包。

1. **確認網路與唯一 LiDAR driver。** 在主機檢查路由；若已有
   `awesome_khayyam` 或其他容器發布 `/x10/scan`，先使用它，不要再啟動第二個 driver。

   ```bash
   ip -4 route get 192.168.1.200
   ```

   預期使用 `eth0`。主機路由來源 IP 正確，並不代表雷達目的 IP 已正確；
   仍需確認雷達實際送往 `.100` 還是 `.102`。遇到 timeout 時可在主機檢查：

   ```bash
   sudo tcpdump -ni eth0 -vv 'host 192.168.1.200 and udp'
   ```

2. **必要時才啟動 driver。** 在已 source Humble 與 `/workspace/install/setup.bash`
   的容器終端執行；此 launch 名稱來自提供文件，driver 不包含於本 UART 套件。

   ```bash
   ros2 launch lslidar_driver lslidar_m10p_launch.py
   ```

   在另一個已 source 的容器終端確認：

   ```bash
   ros2 topic type /x10/scan
   ros2 topic hz /x10/scan
   ros2 topic echo /x10/scan --once
   ```

   本巡邏腳本需要 `sensor_msgs/msg/LaserScan`。如果實際輸出為 PointCloud2，
   不能只改 topic 名稱，須先配置適當的 LaserScan 轉換。另須檢查 `frame_id`、
   角度、量測範圍、時間戳、車頭零度方向及角柱遮擋設定。跨容器節點須使用一致的
   `ROS_DOMAIN_ID` 與相容 QoS，網路可通不代表 ROS 資料已可互通。

3. **先不接通馬達控制，檢查巡邏輸出。** 確認 UART bridge、原廠底盤 driver
   及其他會把 `/cmd_vel` 送到馬達的節點都已停止，再啟動本專案 FSM：

   ```bash
   python3 /workspace/rl_pig_pen/fsm_lidar_patrol.py \
     --ros-args -r /scan:=/x10/scan
   ```

   可用 `ros2 topic echo /cmd_vel` 與 `ros2 topic echo /patrol/state` 查看輸出。
   目前 FSM 預設不等待攝影機連線，只要雷達條件符合就可能發布非零速度；
   「不驅動馬達」必須由下游隔離確保，不能把這條啟動命令當成內建安全模式。
   本 FSM 要求主幹道起點及可辨識的走道幾何，空曠處能收到雷達不代表完整巡邏可運作。

4. **獨立驗證 UART，再整合。** 先停止 FSM 及其他速度發布者，架高車輪、備妥急停，
   確認本車底盤使用本文的 11-byte 協議，再啟動本套件並依「架高車輪測試」低速驗證。

   ```bash
   ros2 run wheeltec_uart_bridge wheeltec_uart_bridge_node --ros-args \
     -p port:=/dev/ttyACM0
   ```

   文件中的 `ros2 run mecanum_controller mecanum_driver` 是另一套底盤節點，
   不是本專案套件；不能與本 bridge 同時開啟串口。文件中的 `obstacle_avoidance`
   也不是本巡邏 FSM，不得與 FSM 同時發布速度。本 bridge 預設拒絕橫向速度，
   不因對方套件名稱含 `mecanum` 就改成麥克納姆控制。

   若之前的 FSM 仍在發布速度，bridge 接通後便可能收到新指令而動作；因此不要
   在不知情的情況下把 UART bridge 接到正在運行的 FSM。整合時先讓 bridge
   在無速度來源下保持零輸出，再於安全條件具備後啟動唯一的 FSM。

5. **停止與驗收。** 先停止 FSM／手動速度發布者，再確認 bridge 的 0.5 秒命令
   逾時停止確實生效。必要時可送一次零速度作為輔助，但仍在執行的控制器可能
   立即覆蓋它；不能用單次零指令取代急停。確認實車停止後才關閉 bridge／容器。

### 文件回報與剩餘待辦

提供文件勾選的進度包括：driver packages 已編譯、新 LiDAR image 建置成功、
`/x10/scan` 持續發布資料，以及狀態機能接收資料並運行／切換狀態。
但文件也使用另一套 `mecanum_controller` 作為測試示例，不能因此認定本套件
`wheeltec_uart_bridge` 已完成實車驗收，也不能當成完整豬舍巡邏成功紀錄。

下列項目仍保留為待驗收，不因本次整理而勾選完成：

- [ ] 將 driver 原始碼、依賴及編譯步驟納入 Dockerfile，從乾淨環境重建 image。
- [ ] 確認 Docker 權限、掛載、串口權限及新容器的 host network／ROS 通訊。
- [ ] 統一雷達目的 IP 與主機正式 IP，驗證 UDP 接收且只運行一個 driver。
- [ ] 驗證 `/x10/scan` 型別、約 12 Hz 更新、時間戳、座標方向與角柱遮擋。
- [ ] 在下游隔離時確認 FSM 收到資料，並檢查路線狀態與速度輸出。
- [ ] 注入雷達中斷／異常，驗證停止與恢復，不只查看正常資料。
- [ ] 本 UART bridge 架高車輪測試：方向、命令逾時、Ctrl-C、串口斷線／重連。
- [ ] 確認 STM32 自身失聯保護與實體急停；Xavier 或容器失效時仍能停止。
- [ ] 於無人畜、可立即斷電且幾何適合的場地做低速整合，再驗收完整巡邏。
- [ ] 保存 driver、FSM、UART 節點日誌、使用版本、啟動順序與最終驗收結果。
