# v6：階段、局部誤差與走偏修正

維持混合架構。FSM 控制路線與長距離直行；模型輸出路口減速到走道交接的 v/w。
原巡邏入口、UART 與原 FSM 程式不改。舊 v5 模型及 90 回合原始資料保留作基準。

## 輸入與時序

原始每幀 151 維後接 10 維，共 **161 維**；4 幀為 **644 維**：

| 欄位 | 定義 |
| --- | --- |
| phase one-hot，4 維 | APPROACH／CENTER／TURN／ENTRY，倒退直穿使用 ENTRY |
| centre_x／centre_y | LiDAR 路口中心在車身座標的前後／左右位置，除以 2 m 並截限 |
| heading_error | 當前階段目標相對朝向，wrap 到 [-π,π] 後除以 π |
| centre_valid／heading_valid | 當前掃描的幾何是否可用；無效數值填 0 |
| repositioning | FSM 是否處於低速倒退重新置中的修正模式 |

多出重置中模式一位，是為了區分同樣位置下的「正在退開」與「正在重新接近」。
policy_context.py 只讀 FSM、LiDAR 幾何；不讀 Gazebo 位姿。
同一幀先更新 FSM，再產生觀察與教師標籤；更新後的觀察供下一次決策使用。
每個 scan stamp 只處理一次，避免確認計數被重複更新。

## 三種資料來源

1. normal：成功、無障礙保護的原 FSM 示範。
2. perturbed：在技能執行中施加小幅模擬偏移，再由教師恢復並完成；沒有真的施加偏移，
   或無法完成恢復的回合不算成功修正示範。
3. dagger：教師與既有 BC 模型混合控制。每個狀態仍用教師動作作標籤，另存模型實際動作。
   即使本回合最後失敗，只要有至少 5 個有效教師標籤，可保存有效片段；全部 training-only。

DAgger 的 accepted 指「標籤片段可用」，不是導航成功。定位／教師判斷無效的影格以
label_valid=False 排除，不把保護停車當成正確修正。歷史視窗仍保留這些影格，
避免刪除影格後把不連續時間誤接在一起。

受控偏移提供側向 2–4 cm、朝向 2–4°，或接近中心時移到估計中心後方 6–8 cm。
偏移在單一步進內注入，不額外推進時間；老師在下一個實際雷達掃描上決定修正。
世界位姿只用於模擬施加擾動及候選碰撞代理檢查，不用來計算教師標籤。
這是測試擾動，不是經校準的真實滑動物理模型，也不保證教師能處理所有偏移。
修正示範的教師先回正至 2.5 cm，再轉彎，保留轉向時的中心偏移餘裕；
原巡邏配置與正常示範仍使用原本 5 cm 容許，教師參數另存於回合 metadata。
越位示範還必須包含有效的 CENTER 倒退修正標籤，不能只在容許範圍內繼續轉彎就算恢復。

## 操作環境

終端 A：

```bash
cd /home/an/Desktop/4wd
bash scripts/launch_rl_junction.sh --gui
```

終端 B：

```bash
cd /home/an/Desktop/4wd
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=42
export IGN_PARTITION=4wd_rl_junction
export GZ_PARTITION=4wd_rl_junction
```

以下模擬指令每次只執行一個。收集／評估時不要在 GUI 播放或重設。

## 重用舊的 90 回合

```bash
/usr/bin/python3 rl_junction/upgrade_demos.py \
  --source logs/rl_junction_pipeline/20261102_175711_954426/demos \
  --output '<新轉換目錄>'
```

以原始雷達及時間戳逐幀重放未改動的 FSM，核對每個教師命令與狀態後才輸出。
不使用 poses 欄位補算誤差，不修改來源檔。不一致的回合報錯，不能直接換版本標記。
v6 在新掃描上提交 FSM 交接，因此移除 v5 最後多出的一個靜止交接步，
以該掃描提供最後的 next_observation。

本機已完成轉換：`logs/rl_junction_v6_data/replayed_normal_r4/`。
正式比較建議保留原分割：
`logs/rl_junction_pipeline/20261102_175711_954426/bc/20261102_200628_988153/split.json`。

## 收集走偏恢復示範

```bash
/usr/bin/python3 rl_junction/collect_demos.py --task all --mode perturbed \
  --target-successes 12 --episodes-per-task 90 --seed 102000
```

預設 mixed 每回合選一種擾動；倒退穿越沒有 CENTER，會在 CROSS_REVERSE 注入側向或角度擾動。
指定某種情況：

```bash
/usr/bin/python3 rl_junction/collect_demos.py --task main_to_left --mode perturbed \
  --perturb-case overshoot --perturb-phase center --episodes-per-task 10 --seed 103000
```

overshoot 只適用轉彎任務的 CENTER，不能對倒退直穿要求不存在的置中／旋轉階段。

## 從模型實際到達的狀態取得教師標籤

```bash
/usr/bin/python3 rl_junction/collect_demos.py --task all --mode dagger \
  --behavior-model '<BC模型.zip>' --teacher-probability 0.5 \
  --episodes-per-task 3 --seed 202000
```

教師機率 0.5 表示每步有一半機率由教師操作，其餘由模型操作；標籤始終是教師動作。
actions／expert_commands 是教師標籤，behavior_actions／requested／executed 是實際行為。
v5 的 151 維舊模型可透過明確的原始特徵適配器作為 behavior policy；
其地圖、雷達、底盤、FSM、動作單位必須相容。這不代表可以把 v5 權重直接當 v6 模型載入。

DAgger 請用獨立訓練種子。不要拿已保留的測試起點收標籤，再繼續當作未見測試。
目前是單輪 DAgger 式資料聚合；可用重新訓練後的模型再收下一輪，尚未自動執行多輪迭代。

## 混合資料與階段平衡 BC

```bash
/usr/bin/python3 rl_junction/train_bc.py \
  --demos '<v6正常目錄>' '<成功修正目錄>' '<DAgger目錄>' \
  --split-from logs/rl_junction_pipeline/20261102_175711_954426/bc/20261102_200628_988153/split.json \
  --balance phase --correction-fraction 0.3 --epochs 100
```

不使用某種來源時省略該目錄。先加 --check-only 檢查資料。
抽樣先平衡任務，再平衡該任務實際存在的階段；同一階段同時有正常／修正標籤時，
預設給 70%／30% 抽樣權重。這是可調初始值，不是驗證過的最佳比例。
只有一種標籤時使用該種，不補造不存在的階段。

### 路口閉迴路失敗後的修正

若 BC 在路口提早停住、轉彎時仍前進，或倒退偏向側牆，可用該模型執行 DAgger：

```bash
/usr/bin/python3 rl_junction/collect_demos.py --mode dagger \
  --behavior-model '<目前BC模型.zip>' --teacher-probability 0 \
  --task all --episodes-per-task 1 --max-steps 160 --seed 5000
```

模型負責實際動作，FSM 只提供對應狀態的專家標籤；安全保護仍有效。
失敗回合的有效前綴只作訓練補充，不能算成功示範或測試成果。
收集種子不得與 validation／test 重疊，補收後必須重訓並重新閉迴路測試。

新增可選 `--balance phase-action`：保留任務、階段及正常／修正來源的外層權重，
再於各組內平衡實際存在的前進、倒退、旋轉、停止標籤。
分類先還原物理速度，倒退任務的 normalized action=-1 代表停止。
此選項不改示範動作、不添加最低速度、不放寬 FSM 置中或安全條件；
是否有改善仍須比較未見回合，不能只看離線 MSE。預設仍是 `--balance phase`。

`--geometry-gain 10` 可在策略網路內放大每個歷史幀的中心 x/y 與航向誤差：
例如 5 公分原本編碼為 0.025，網路收到的對應特徵變成 0.25。
雷達、階段與有效性旗標不變，觀察仍為 161×4=644 維。
固定倍率作為模型 buffer 一起存檔，推論及後續 PPO 會使用相同轉換；
不需要修改環境、重寫舊資料，也不使用真實世界座標。預設倍率 1 保留原尺度。

正常與受控偏移各自做完整回合 holdout，每種任務／來源至少 3 個成功回合；
DAgger 只進 train，不能單靠 DAgger 當驗證集。--split-from 可保留原正常資料分割，
也會拒絕以參考 validation／test 種子產生的 DAgger 資料。
驗證／測試維持自然分布，另報各階段與各來源誤差；選模先平均任務內階段，再平均任務，
避免階段較少的倒退任務被低估。--balance task 可作抽樣消融比較。

少量修正樣本若只用來補充訓練，可改用 `--augmentation-demos '<修正目錄>'`：
這些有效的恢復回合全部只進 train，不要求各來源 3 回合；正常資料仍須有獨立
validation／test。此選項不提供偏移恢復能力的驗證，仍需另跑未見偏移測試。
`split.json` 會明確記錄補充資料路徑與 training_only，不能把訓練用修正回合再報成測試。

## 一次完成收集與訓練

```bash
/usr/bin/python3 rl_junction/pipeline.py \
  --normal-demos logs/rl_junction_v6_data/replayed_normal_r4 \
  --split-from logs/rl_junction_pipeline/20261102_175711_954426/bc/20261102_200628_988153/split.json \
  --recovery-successes-per-task 12 --dagger-model '<舊或新版BC模型.zip>' \
  --epochs 100 --seed 2000 --detach
```

會驗證重用正常資料 → 收修正示範 → 收 DAgger 標籤 → 驗證全部資料 → 訓練。
省略 --dagger-model 就跳過 DAgger；--recovery-successes-per-task 0 可跳過受控偏移。
失敗會停在該階段並保留診斷，不能宣稱資料或模型已完成。

## 評估

正常情況與偏移恢復要分開評估，例如：

```bash
/usr/bin/python3 rl_junction/evaluate.py --model '<新版模型.zip>' --episodes-per-task 10 --seed 100000
/usr/bin/python3 rl_junction/evaluate.py --model '<新版模型.zip>' --task main_to_left \
  --recovery-case overshoot --episodes-per-task 10 --seed 110000
```

recovery_success_rate 只計真的施加偏移的回合；未觸發擾動不冒充恢復成功。
此版擴充的功能驗證與少量恢復示範，不代表正式新模型已優於 v5。PPO 尚未啟動。

本機本次驗證結果與實際資料位置見 [v6 驗證紀錄](V6_VERIFICATION.md)。
