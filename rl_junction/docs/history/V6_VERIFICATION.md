# v6 功能驗證（2026-11-03）

## 完成項目

- 15 項離線回歸檢查通過，包括局部誤差／過期旗標、歷史視窗、階段與修正抽樣權重、
  教師標籤和行為分離、DAgger 訓練專用分割、原 holdout 保留、真實倒退修正檢查。
- 舊的 90 個成功回合全部由原始 LiDAR 重放 FSM；每一步狀態與命令核對通過。
  轉換後共 11,789 步、11,761 個有效教師標籤；沒有用世界位姿補算輸入。
- 每幀 161 維、4 幀 644 維，新 actor 為 99,330 個參數。
- 原有 v5 模型、原始資料與原巡邏程式保留。

## Gazebo 修正示範

`logs/rl_junction_v6_smoke/recovery_precise_teacher/main_to_left_0000.npz`：

- 在 CENTER 注入越位，目標為中心後方 6–8 cm。
- 修正教師使用 2.5 cm 回正容許，原巡邏的 5 cm 配置不變。
- 擾動後錄得 15 個有效 CENTER 標籤，其中 11 個為倒退修正。
- 最大倒退修正速度約 -0.03449 m/s；之後完成轉彎與支線交接。
- 本回合 202 步、40.4 秒模擬時間，未觸發碰撞代理或障礙保護。

其他測試也有教師無法完成、觸發保護的案例，沒有當作成功恢復示範。
這是一個成功的功能驗證，不是恢復成功率統計；修正資料尚不足以獨立切分 holdout，
因此本次使用 --augmentation-demos，明確只放入 train。

## DAgger 式標籤收集

`logs/rl_junction_v6_smoke/dagger_final/main_to_left_0000.npz`：

- 使用原 v5 模型的原始特徵適配器，教師機率為 0.5。
- 收集 80 步標籤片段；回合到步數上限，沒有完成導航。
- actions／expert_commands 為教師標籤，behavior_actions／executed 為行為。
- 片段標為 training_only，沒有混入 validation 或 test。

## 三種來源合併 BC 驗證

資料為 90 個轉換後正常回合、一個完整越位恢復回合及一個 DAgger 片段。
正常資料保留原 train／validation／test 的任務與種子；新增兩個片段均只用於訓練。
最終回合分配為 56／18／18。

- 完成 3 epochs 的混合 BC 功能訓練。
- validation macro_phase_mse：0.23187 → 0.09657。
- 保留正常 test 的整體 MSE：0.02725；沒有將此數值宣稱為導航成功率。
- 模型保存、重新載入、644 維批次推論與 PPO 空間相容性檢查通過。
- 此為功能驗證，尚未執行新版 100-epoch 正式訓練或完整閉迴路比較，沒有啟動 PPO。

輸出：`logs/rl_junction_v6_smoke/bc_all_final/20261103_131838_304265/`。
正常資料：`logs/rl_junction_v6_data/replayed_normal_r4/`。
這些本機資料、模型及日誌均被 Git 忽略。

## 可重現指令

```bash
/usr/bin/python3 rl_junction/train_bc.py \
  --demos logs/rl_junction_v6_data/replayed_normal_r4 logs/rl_junction_v6_smoke/dagger_final \
  --augmentation-demos logs/rl_junction_v6_smoke/recovery_precise_teacher \
  --split-from logs/rl_junction_pipeline/20261102_175711_954426/bc/20261102_200628_988153/split.json \
  --epochs 3 --output logs/rl_junction_v6_smoke/bc_all_final
```
