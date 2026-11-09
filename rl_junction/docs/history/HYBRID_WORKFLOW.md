# 混合架構：路口 MLP＋BC，再以 PPO 改善

目前提供 MLP 模仿學習打底、幾何驗證及 PPO 初始微調流程。既有巡邏入口仍使用傳統 FSM；
這個資料夾提供獨立 Gazebo 示範、離線 BC、閉迴路評估與載入 BC 的 PPO 入口。
短版訓練與資料收集用於確認流程，不能當作模型已學會或已可實車部署的證據。

目前為 v6：新增階段、LiDAR 局部誤差、階段平衡 BC、受控偏移恢復示範及 DAgger。
完整指令、舊資料轉換與限制見 [走偏修正與新版 BC](RECOVERY_BC.md)。
本輪補收資料、候選比較與 Gazebo 結果見 [BC 路口修正紀錄](BC_IMPROVEMENT.md)。
假路口診斷、停車重取中心及 PPO 啟動條件見 [幾何修正與 PPO](GEOMETRY_AND_PPO.md)。
接續訓練、critic 暖身與完整學習指標檔案見 [PPO 接續訓練](PPO_CONTINUATION.md)。

## 控制權的範圍

| 路段／功能 | 負責者 |
| --- | --- |
| 主幹道、支線長距離直行 | 傳統 LiDAR 控制／FSM |
| 路口首次取得幾何後的減速與接近 | 學習策略 |
| 路口置中、左轉／右轉、倒退直穿 | 學習策略 |
| 離開路口、取得下一段走道牆面 | 學習策略 |
| 路線順序、任務選擇、成功交接判定 | FSM |
| 障礙淨空、感測有效性、故障停止 | 保留檢查 |

`junction_env.py` 在 reset 後先讓 FSM 直行。首次取得路口幾何時，立即把
**第一個減速動作**起的控制權交給策略，結束於 FSM 確認走道牆面並切回直行狀態。
減速前的長距離直行不進 BC 資料；不是只學原地轉 90°，也不是整條巡邏改成學習。

同一環境用於教師與模型評估。收集時由原 FSM 產生策略區間的專家動作；
模型評估時改用模型輸出 `v,w`，FSM 繼續處理狀態、幾何與交接。
原 FSM 也會檢查其建議動作的安全性，觸發保護或重新定位時中止本回合；
未來 PPO 若需放寬此保守的教師檢查，需另外分離事件判斷與建議動作的安全檢查。

## 地圖與三種技能

使用原本 `worlds/pig_pen_16units.world` 第一個路口 `(0,-6.18)`。
reset 位置為巡航暖身起點，**不是策略起點或成功終點**。

| 任務代號 | 暖身起點 (x,y,yaw) | FSM／策略區間 | 交回直行狀態 |
| --- | --- | --- | --- |
| main_to_left | (0,-8.18,90°) | MAIN 路口接近 → CENTER → TURN_LEFT → ENTRY_LEFT | LEFT_OUTBOUND |
| left_to_right_reverse | (-2,-6.18,180°) | REVERSE_LEFT 路口接近 → CROSS_REVERSE | RIGHT_OUTBOUND |
| right_to_main | (2,-6.18,180°) | RETURN_RIGHT 路口接近 → RETURN_CENTER → TURN_MAIN → EXIT_MAIN | MAIN |

起點隨機位置 ±0.08 m、朝向 ±0.08 rad。reset 時 FSM 分別由 MAIN、
REVERSE_LEFT、RETURN_RIGHT 開始，因此可獨立訓練技能，不需每回合走完整條巡邏。
本版還沒有把訓練模型接回實車完整巡邏，也沒有跨路口、斜坡泛化驗證。

## 模型與資料

- 每幀 161 維：原本 151 維雷達／速度／任務／前次動作，加 4 維階段、
  2 維車身座標的路口中心、1 維目標朝向誤差、2 維有效旗標、1 維重置中模式。
  雷達沿用角柱遮罩，局部誤差只取 LiDAR FSM，不使用世界真值。
- 預設取最近 **4 幀，共 644 維**，按由舊到新串接給 MLP；
  起始不足 4 幀時重複第一幀，不讀未來、不跨回合。可用 `--history 1` 做單幀基準。
- Actor 預設兩層 128 個單元，輸出兩維連續動作。MLP 使用 SB3 的 MlpPolicy，
  可直接保存給後續 PPO 載入。BC 不訓練價值網路。
- 正常任務：`v=.2*a[0]`、`w=.6*a[1]`；倒退穿越：
  `v=-.1*(a[0]+1)`，第一維 -1 停止、+1 最大倒退。所有策略動作經安全層。
- 教師為 `fsm_teacher.py` 包裝的 **TraditionalPatrolController**，
  專家只讀 LiDAR；巡航速度上限配合策略設為 0.2 m/s、轉速上限 0.6 rad/s。
  原巡邏配置不改。歷史航點教師 `teacher.py` 不再用於正式收集。
- 每回合保存基礎觀察、下一幀觀察、動作、實際命令、雷達原始距離、感測時間戳、
  實測速度、FSM 狀態與階段。世界座標只存於診斷欄，不進入模型。
  完整基礎資料可供後續 Transformer 使用同一段歷史。
- 正常／受控偏移示範只接受成功且無障礙介入／重新定位的完整回合；
  短暫無效教師標籤用 label_valid 排除。DAgger 可保存失敗前的有效教師標籤，
  實際模型動作另存，這些片段強制 training-only，不列入導航成功數。
- 資料檢查包括動作映射、前次動作與下一幀對齊、任務標記、時間同步、
  技能狀態邊界、重複任務／種子、數值範圍及環境版本。
- 約 60%／20%／20% 按任務及正常／受控偏移來源以完整回合切分，
  每種存在的來源至少 3 個成功回合。DAgger 只入 train；重用舊資料可用
  --split-from 保持原本的 train／validation／test 任務種子分配。
- 以 validation 選最佳 checkpoint，最後只對選定模型計算離線 test 誤差。
  預設依任務、階段、正常／修正標籤平衡抽樣，並以任務內階段平均、再任務平均的
  validation macro_phase_mse 選 checkpoint。模型與切分、種子、版本、雜湊一起記錄。

## 操作：先收集，再訓練

本機已完成的功能驗證見 [驗證記錄](VERIFICATION.md)。

### 自動收集後開始 BC

先依下面終端 A 啟動模擬，再於已設定 ROS 環境變數的終端 B 執行：

```bash
/usr/bin/python3 rl_junction/pipeline.py --successes-per-task 30 --max-attempts-per-task 90 --epochs 100 --seed 2000 --detach
```

每種任務收集到 30 個成功回合即停止該任務；最多各嘗試 90 次。
v6 另外預設各收集 12 個成功的受控偏移恢復回合，可用 --recovery-successes-per-task 0 停用。
全部達標且資料檢查通過後，會自動訓練 4 幀、128×128 的 MLP＋BC，最多 100 epochs。
任何階段失敗會記錄原因並停止，不會以不足或錯誤資料接著訓練。
`--detach` 在背景運行，不加則在前景顯示。Gazebo 必須持續運行。

日誌在 `logs/rl_junction_pipeline/`：啟動器會印出背景 PID 與總日誌。
每次工作目錄含 `status.json`（collect／validate／train／complete／failed）、
`collect.log`、`collect_recovery.log`、`validate.log`、`train.log`，
以及 `demos/normal/progress.json`、`demos/recovery/progress.json` 的各任務成功數。
模型在該次工作的 `bc/<時間>/bc_model.zip`。不要同時啟動另一個收集／評估／訓練程式。

需要 ROS 2 Humble、Gazebo Fortress／ign、ros_gz_bridge、ros_gz_interfaces、
ros_gz_sim，以及可由同一 Python import 的 numpy、gymnasium、torch、stable-baselines3、rclpy。

### 1. 終端 A：原地圖模擬

```bash
cd /home/an/Desktop/4wd
bash scripts/launch_rl_junction.sh --gui
```

省略 `--gui` 就是無視窗模式。使用獨立 ROS domain 42、Gazebo partition，
不啟動原巡邏節點或 UART。不要同時開第二個相同訓練實例。
收集或評估時由程式控制暫停／步進，不要按 GUI 播放或重設。

### 2. 終端 B：收集 FSM 路口示範

```bash
cd /home/an/Desktop/4wd
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=42
export IGN_PARTITION=4wd_rl_junction
export GZ_PARTITION=4wd_rl_junction

# 預設先練主幹道轉左支線
/usr/bin/python3 rl_junction/collect_demos.py --task main_to_left --episodes-per-task 30 --seed 0
```

`--episodes-per-task` 是嘗試數，不保證成功數。首次可設 1 觀察流程；
正式訓練先收 30 次並確認接受率。終端會印出
`Policy takeover: ... (junction acquired)`，此後才是要錄製的區間。

三種技能一起收集：

```bash
/usr/bin/python3 rl_junction/collect_demos.py --task all --episodes-per-task 30 --seed 0
```

檔案：`logs/rl_junction_fsm_demos/<時間>/`，每回合 JSON 診斷、成功回合 NPZ、
總結 summary.json。補收時用不同種子，例如 `--seed 1000`。
舊的航點教師／純原地轉彎示範保留在日誌內，版本不同，不能混入正式資料。

### 3. 檢查是否可以訓練

把路徑換成剛才終端印出的目錄；也可以指定包含多個新版本收集目錄的父目錄。

```bash
/usr/bin/python3 rl_junction/train_bc.py --demos '<FSM示範目錄>' --task main_to_left --check-only
```

會列出各任務的回合、有效標籤與訓練專用片段數。各來源有足夠獨立回合才顯示 READY FOR BC。
所有資料都通過檢查後才能訓練；請勿用整個包含舊版資料的 logs 當資料目錄。

### 4. 離線訓練 MLP＋BC

```bash
/usr/bin/python3 rl_junction/train_bc.py --demos '<FSM示範目錄>' \
  --task main_to_left --history 4 --hidden-sizes 128 128 --epochs 100 --seed 0
```

訓練所有已收集的任務可省略 `--task`。BC 不需 Gazebo。
輸出：`logs/rl_junction_bc/<時間>/`：

- `bc_model.zip` 與同名 `.json`：最佳模型與環境／輸入介面。
- `split.json`：回合切分、來源雜湊、模型設定與套件版本。
- `metrics.csv`：逐 epoch 的 train／validation loss 及階段宏平均誤差。
- `results.json`：最佳 epoch、離線 test 的整體／分階段／來源動作誤差。

預設 validation 連續 20 個 epoch 沒改善即提早停止。Ctrl-C 保留已完成的最佳 checkpoint。
離線誤差低不代表能完成路口。

### 5. 在 Gazebo 比較 FSM 和 BC

終端 A 保持模擬開啟，終端 B 保持上面的環境變數。每次只運行一個控制程式。

```bash
/usr/bin/python3 rl_junction/evaluate.py --teacher --task main_to_left --episodes-per-task 10 --seed 100000
/usr/bin/python3 rl_junction/evaluate.py --model '<BC目錄>/bc_model.zip' \
  --task main_to_left --episodes-per-task 10 --seed 100000
```

兩者在長距離直行都由 FSM 控制，差異只在路口技能區間。模型預設只評估其訓練過的任務。
報告：`logs/rl_junction_eval/<時間>/evaluation.json`，包含成功率、成功回合時間、
航向／側向誤差、停止時間、安全介入、碰撞代理與失敗原因。
`--max-steps` 可縮短功能檢查，但截斷回合不能當作完整成功驗證。

### 後續階段：從 BC 載入 PPO

BC 打底後，先驗證目前模型的幾何門檻，再以小預算啟動 PPO；正式長期訓練另做比較。

```bash
/usr/bin/python3 rl_junction/train.py --init-model '<BC目錄>/bc_model.zip' \
  --geometry-gate '<評估目錄>/geometry_gate.json' --timesteps 1024 --seed 70000
```

沿用相同混合架構、歷史幀與 actor 權重，只對訓練過的任務抽樣。
獎勵使用到出路口參考位置的進展、航向進展、時間／動作變化／安全介入成本；
成功依 FSM 牆面交接而非固定座標點。評估預設 1200 步（240 秒模擬時間）截斷；
初始 PPO 預設每回合 300 步（60 秒），三項任務輪替，避免單一停滯回合佔滿短訓練。
獎勵仍需正式校準；PPO 不保證改善。

## 驗證範圍與限制

- 5 Hz 暫停步進，等待新 scan、odom、帶時間戳 PoseStamped 與 clock；
  使用常駐 ROS–Gazebo control/set_pose 服務，reset 不倒退模擬時間。
- 原地圖幾何碰撞代理只計算高度 0.245 m 的 box／cylinder，
  不是完整接觸感測，未含 include 豬模型、全車高度、翻覆與坡道驗收。
- 安全層、FSM 路線事件仍保留；實車底盤回授的時間戳與單位尚需獨立接入驗證。
- `.zip` 與同名 `.json` 需一起保存；環境、地圖、配置或 FSM 變動會拒絕混用。
- 日誌、示範、模型已加入 gitignore。此訓練環境不能連接實車 UART。
- 資料與歷史視窗的離線回歸檢查：
  `/usr/bin/python3 -m unittest discover -s rl_junction/tests -v`。
  實際 Gazebo 示範、短版 BC 與評估結果應分別記錄，不能以單元測試代替物理驗證。

## 模型連續控制路口的新實驗

[連續路口控制](../../CONTINUOUS_CONTROL.md)：模型從接手減速至進入正確走道自行控制，
取消路口內 FSM 置中／轉彎關卡，保留雷達感知、安全層與最終交接判斷。
使用獨立資料／模型 contract；已提供離線 BC 轉換與短輪 PPO 入口，尚待 Gazebo 閉迴路驗證。
