# 用手動駕駛示範預訓練 PPO

這個流程先教策略模仿人類的走道控制，再交給 PPO 學習獎勵。
觀察仍是 36 條 LiDAR＋左右平衡＋前方淨空；模型不會取得路線記憶。
因此示範只訓練走道前進、置中與避障，路口選向、掉頭仍交給巡邏狀態機。

## 1. 開啟模擬與錄製

若 PPO 正在跑，先在其終端按 Ctrl-C，等模型存檔和程序清理完成。
不要同時讓 PPO、狀態機和手動控制器發布速度。

以下命令從專案根目錄執行，使用已具備 ROS2 與 Python 相依套件的環境。

```bash
# 終端 1：保留畫面，目標 1 倍速便於人類操作
bash scripts/launch_clean.sh --rtf 1

# 終端 2：開始只讀錄製
python3 rl_pig_pen/record_demonstrations.py

# 終端 3：鍵盤控制；i 前進、u/o 向前左/右修正、k 停車
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p speed:=0.2 -p turn:=0.5
```

持續按住移動鍵，讓鍵盤重複發送指令。現有 `teleop_twist_keyboard` 在沒有按鍵事件時
不會持續發布；錄製器也不會把很久以前的速度指令當作新的專家決策。
搖桿可替代終端 3，只要控制器持續發布 `geometry_msgs/Twist` 到 `/cmd_vel`。
如使用不同的速度 topic，以 `--cmd-topic /你的速度topic` 指定；目前不接受 TwistStamped。

錄製器不發布速度、不傳送 teleport、不更改模擬。
每個新指令配對到接收它之前最新的一幀雷達，預設接收時間差最多 0.25 秒；
同一幀掃描最多一筆樣本。Twist 沒有時間戳，因此這是接收時間配對，不是硬體同步。
可調整 `--max-scan-age`，但不要靠放大它掩蓋感測延遲。

## 2. 示範內容與資料品質

先錄數段獨立的示範，每段涵蓋不同走道、方向、輕微偏左及偏右後的修正。
不要只錄一路直行；缺少修正範例時，模型也無從學到如何回到中央。
可以先以 2～3 段、每段 5～10 分鐘作為起步實驗，再看實際有效樣本數與驗證誤差。
至少 100 筆才允許預訓練；這只是程式最低限制，不代表資料已足夠。

自動排除的資料包括：

- 過期或無效掃描、同一幀重複取樣。
- `/cmd_vel` 同時有多個發布者，無法分辨是否為人工指令。
- 倒車、超出既有動作上限、側移或非平面轉動指令；不會悄悄截斷成另一個動作。
- 碰撞代理判定、會被現有安全層阻止的命令。
- 路口／開放區域、原地轉向、前方很空卻長時間停車。

前方近距離障礙物前的停車可以保留。自動篩選不能保證剩下每筆示範都是好決策，
錄製時仍需平穩操作。轉彎進出路口可以正常手動完成，篩選器會排除不適合局部策略的部分。

每 500 筆或 15 秒寫入一個 NPZ，Ctrl-C 會寫入不足一段的剩餘資料。
每次錄製建立獨立資料夾，不覆蓋前一次：

```text
rl_pig_pen/demonstrations/<session>/
  session.json        # 參數、topic、採用／排除原因統計
  chunk_000000.npz    # observations[N,38]、actions[N,2]、時間與格式資訊
  chunk_000001.npz
```

結束時先用 `k` 停車，再關閉鍵盤程式，最後在錄製終端按 Ctrl-C。
錄製器退出本身不會讓車子停下來。
若 `accepted` 幾乎不增加，查看 `counts`：`no_scan` / `stale_scan` 表示感測問題；
`multiple_or_unknown_command_sources` 表示還有其他速度控制器；`idle` / `turn_in_place`
表示目前動作不屬於這個前進策略。

## 3. 離線行為複製

```bash
python3 rl_pig_pen/train_bc.py --epochs 30

# 也可選擇某一段資料，或另存第二個實驗
python3 rl_pig_pen/train_bc.py --data rl_pig_pen/demonstrations/<session> \
  --output rl_pig_pen/checkpoints_bc/ppo_corridor38_bc_v2.zip
```

不會啟動 Gazebo 或 ROS node。預設 CPU 單執行緒，也支援 `--device cuda`。
只訓練 actor 的平均速度輸出，以線速度／角速度各自上限正規化誤差。
多段示範時保留整段 session 作驗證；只有一段時使用最後 20% 並留 12 筆間隔，
避免把緊鄰影格隨機混入訓練與驗證。

每個 epoch 印出線速度 MAE（m/s）、角速度 MAE（rad/s）與驗證損失。
使用驗證損失最好的 epoch，連續 5 個 epoch 未改善則提前停止。
若完全沒有改善，會報錯而不把初始隨機策略當成成功結果。

輸出 `checkpoints_bc/ppo_corridor38_bc.zip` 與旁邊的 JSON 訓練報告。
錄製 NPZ 並不會直接產生模型；必須看到 `Saved best BC actor as a PPO checkpoint` 才能進行下一步。
若提示 `Only ... samples`，請繼續錄製至至少 100 筆有效樣本（建議多段示範），再重跑本步驟。
已存在的輸出不會自動覆蓋，重新實驗請換 `--output`。
模型保留小幅探索標準差（0.1 m/s、0.2 rad/s），避免接回 PPO 時立刻使用過大的隨機動作。
critic 沒有專家價值標籤，因此保持未預訓練狀態，交給 PPO 學習。

## 4. PPO 續訓與評估

停止鍵盤／搖桿與錄製器，確保只有 PPO 控制模擬。在已運行的 Gazebo 中：

```bash
# 第一次：明確選擇 BC 起始模型，使用獨立實驗資料夾
python3 rl_pig_pen/train_ppo.py \
  --resume rl_pig_pen/checkpoints_bc/ppo_corridor38_bc.zip \
  --checkpoint-dir rl_pig_pen/checkpoints_from_bc \
  --log-dir rl_pig_pen/logs_from_bc --timesteps 300000

# 中斷後：從該實驗最新 PPO 模型接續，不要再次回到原始 BC 模型
python3 rl_pig_pen/train_ppo.py \
  --checkpoint-dir rl_pig_pen/checkpoints_from_bc \
  --log-dir rl_pig_pen/logs_from_bc --timesteps 300000

# 訓練結束後，再於已啟動的模擬評估完整巡邏
python3 rl_pig_pen/test_model.py --checkpoint-dir rl_pig_pen/checkpoints_from_bc --episodes 3
```

若要使用加速的一鍵啟動，先結束原模擬，再從專案根目錄執行；路徑用絕對路徑，
因為啟動腳本會切換工作目錄：

```bash
bash scripts/launch_rl_training.sh --rtf 6 \
  --resume "$PWD/rl_pig_pen/checkpoints_bc/ppo_corridor38_bc.zip" \
  --checkpoint-dir "$PWD/rl_pig_pen/checkpoints_from_bc" \
  --log-dir "$PWD/rl_pig_pen/logs_from_bc" --timesteps 300000
```

一鍵腳本的 TensorBoard 會跟隨 `--log-dir`。手動訓練時可另開
`tensorboard --logdir rl_pig_pen/logs_from_bc`。

行為複製提供較好的起始策略，不保證縮短特定倍數的訓練時間。
是否有幫助，應比較相同 PPO 步數下的碰撞率、支線返回率及完整巡邏成功率。

## 驗證範圍

2026-09-18：40 項自動測試通過，包含資料篩選、分段存取、驗證資料隔離、
actor 擬合、模型存取後一致性及接續 32 步 PPO 更新。
另外在隔離 ROS domain 實際錄製兩段共 220 筆合成資料，通過離線 BC CLI
與 PPO 載入／推論流程。這些測試資料只放在 `/tmp`，不是人工專家示範。
人工資料的品質與實際收斂改善，仍需錄製後再評估。
