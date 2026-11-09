# PPO 接續訓練與學習數據（2026-11-03）

本輪保留原獎勵與幾何／安全條件，從完成 1024 步的 PPO checkpoint 接續，
不重新初始化 actor，也不替換實車巡邏程式。目的先取得更完整基準，
檢查 critic 暖身與增加經驗是否有幫助；階段式獎勵、停滯懲罰仍需另做比較。

## 接續前檢查

模型：`logs/rl_junction_geometry_work/ppo_round_robin/20261103_162522_142524/model.zip`。
以新種子 410000～410005 各任務 2 回合，皆完成，無碰撞旗標或保護介入。
左轉／倒退／右轉平均模擬時間分別為 32.0／20.4／27.0 秒；
最大有效中心誤差約 5.4 cm，通過同一模型的 10 cm 幾何診斷門檻。
這是六回合驗證，不是穩定成功率 100% 的證明。
報告：`logs/rl_junction_ppo_continuation/precheck/20261103_170151_759438/`。

## 本輪設定

| 項目 | 設定 |
| --- | --- |
| 新增步數 | 2048，預定累計 3072 |
| rollout／batch | 256／64 |
| critic 暖身 | 前 2 個 rollout（512 步），actor 與 log_std 凍結 |
| 暖身 critic 學習率／epochs | 0.0003／16 |
| 暖身後 actor、critic 學習率／epochs | 0.00001／4 |
| clip_range／target_kl | 0.1／0.01 |
| 任務 | 左轉、倒退、右轉輪替 |
| 回合上限 | 300 步＝60 模擬秒，只是訓練截斷條件 |
| 種子 | 71000 |

原始初版暖身也使用 0.00001、4 epochs，critic 的回報預測仍弱。
本輪增加的學習率只在 actor 凍結期間使用；切換回策略更新時恢復原小學習率。
已測試有既有 Adam 狀態的 actor／log_std 在暖身期間保持完全相同，
也驗證中途 checkpoint 可以重新載入、暖身率不會變成永久 actor 學習率。

## 檔案與指標

目錄：`logs/rl_junction_ppo_continuation/training/20261103_171350_448638/`。

- `continuation.json`：來源 checkpoint 與起始 1024 步、16 次 epoch 更新。
- `metrics/progress.csv`、`metrics/progress.json`：各次更新的 explained_variance、value_loss、
  approx_kl、clip_fraction、learning_rate 等；不再只有終端機輸出。
- `episodes.csv.monitor.csv`：本輪完整回合的總獎勵、步數、任務、是否完成。
- `episode_details.jsonl`：在自動重生前保存終止／截斷原因與 FSM 診斷。
- `failure_geometry/`：失敗或截斷的原始雷達、牆線／候選中心與拒絕原因，對齊失敗後掃描。
- `latest.zip`／`latest.json`、`progress.json`：最近完成更新的 checkpoint 與進度。
- `model.zip`／`model.json`、`completion.json`：只有正常完成後才有；不能把 latest 當已完成。

`time/total_timesteps` 與 `actual_timesteps` 為累計步數；
`time/run_timesteps`、`additional_timesteps` 為本輪新增步數。
學習指標另在每次更新完成後寫出，避免沿用上一輪更新值卻對應下一輪步數；
同一累計步數可能有「採樣完成」與「更新完成」兩列，畫曲線時取指標非空列。

### TensorBoard 與終端機查看

模型使用 PyTorch／Stable-Baselines3；TensorBoard 只負責顯示圖表，不需要把模型改成 TensorFlow。
在第一個已切換到專案根目錄的終端機執行：

```bash
/usr/bin/python3 rl_junction/view_training.py \
  --run logs/rl_junction_ppo_continuation/training/20261103_171350_448638 --follow
```

此指令只讀 CSV、顯示目前步數與最新學習指標，並持續轉成 `tensorboard/` 的 scalar events。
Ctrl-C 只停止查看，不會停止訓練。訓練完成或中止時會自動停止追蹤。
第二個終端機也切換到專案根目錄，再執行：

```bash
/usr/bin/python3 -m tensorboard.main \
  --logdir logs/rl_junction_ppo_continuation/training/20261103_171350_448638/tensorboard \
  --host 127.0.0.1 --port 6006
```

瀏覽器開啟 `http://localhost:6006`，在 Scalars 看
`rollout/ep_rew_mean`、`train/explained_variance`、`train/value_loss`、`train/approx_kl`。
這是另外開啟的數據查看終端機，不是重新啟動訓練或連接到原來的訓練 stdin。
本輪累計 2816 步的靜態圖已保存為目錄內的 `convergence_snapshot.png`；
靜態圖不會自動更新，TensorBoard events 需讓上述 `--follow` 指令持續執行才會刷新。

訓練含隨機探索，完成比例不能與 deterministic 獨立評估混用；
不同任務距離不同，回合原始獎勵也不能直接當跨任務好壞排名。
暖身的價值指標改善不等於導航策略已改善，後續仍需固定條件對照評估。

## 自動收尾

本次已安排 `logs/rl_junction_ppo_continuation/after_training.py` 等待正常訓練完成，
待訓練程序關閉後，再以新種子 420000～420005 執行六回合 deterministic 評估與幾何稽核。
不混入教師動作，不部署模型；若訓練中止、出現基礎設施錯誤或等待超時，會停止收尾流程。
狀態保存在本輪目錄的 `after_training_status.json`，評估輸出為
`logs/rl_junction_ppo_continuation/postcheck_171350/`。
`state=complete` 代表評估流程完成；需另外看成功回合數與幾何門檻，不能解讀成模型已收斂。

## 十萬步退化與後續保存

2026-11-04：十萬步訓練已完成，轉彎任務出現置中停滯與成功率下降。
詳見 [退化診斷與改善計畫](PPO_REGRESSION.md)。新訓練預設約每 5120 步進行
固定驗證、保存分段 checkpoint 與最佳模型；此次舊訓練的中途權重無法補回。
