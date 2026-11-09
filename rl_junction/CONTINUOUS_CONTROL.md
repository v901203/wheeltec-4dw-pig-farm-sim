# 模型自主完成路口（2026-11-04）

此為新的獨立實驗模式；傳統巡邏與原本 hybrid 訓練入口仍可使用。
尚未完成 Gazebo 閉迴路驗證，也未部署實車。

## 分工

FSM 只在 reset 後處理路口前直線行駛、取得當前路口；此時初始化一份雷達 tracker。
接手後 `ContinuousJunctionEnv.step()` 不呼叫 `teacher.command()`、
不讀取 CENTER/TURN/ENTRY 的切換、失敗或 label-valid 門檻。
模型連續輸出 v/w，決定減速、轉彎位置與進入走道的軌跡。

獨立雷達 tracker 每幀更新中心與目標出口方向，不要求先抵達路口中心，
也不要求在中心停車。幾何無效時停止並嘗試後續掃描重新取得同一 tracker 的有效更新，
最多 50 步（10 模擬秒）；若長期無法匹配則結束，不會盲走或直接換另一個路口。
目前仍依賴路口幾何可觀察性；並未實作 SLAM 重新定位。
`safe_command` 仍檢查 footprint、行進與旋轉掃掠區域。

完成條件是量測已沿指定出口方向進入至少 0.9m、橫向誤差不超過 0.12m、
車頭誤差不超過 10 度、雙側走道牆確認，連續三幀。
倒退穿越的行進方向與車頭方向相反，完成判斷分別處理。
這是最後交接條件，不是路口內逐階段的動作關卡。

## 觀察、資料與模型相容性

保留 161 維一幀、4 幀歷史與三任務輸入，以便沿用 MLP 架構。
四個舊 phase 維度設成 0；局部中心不變，heading 一律表示最終出口車頭方向，
最後一格是倒退任務旗標，不再是 FSM 的 centre-retreat。
世界座標仍只用於模擬 reward/collision 計分，不送進 actor。

新模型使用 `pig_pen_continuous_junction_v1` 獨立 contract。
不能使用舊 `evaluate.py`／`train.py` 直接驗證或載入新模式；它們會拒絕 contract。
`continuous_train.py --mode bc` 只轉換舊示範中的因果 context，
保留原 train/validation/test 分組、檢查來源雜湊，再對新的輸入語意做 BC 微調。
不會將世界位姿加入示範觀察。
目前舊示範仍含 FSM 的停車／分段轉彎，轉換不會憑空產生流暢的新示範。
仍需閉迴路確認 BC 可用；若失敗應補新模式下的安全示範，而非直接長時間 PPO。

已完成 40 epochs 離線轉換：
`logs/rl_junction_continuous/bc/20261104_142847_119472/bc_model.zip`。
驗證動作 MSE 最佳約 0.0141；這不是導航成功率，也不與舊輸入的 BC MSE 直接比較。

## 計分與實驗

PPO 計分使用連續的來向剩餘距離＋出口剩餘距離，
不要求精確抵達中心、沒有階段切換獎勵，保留時間、動作平滑及安全遮斷成本。
倒退使用出口距離。終端成功 +20、失敗 -20。
150 個有效量測且未被安全遮斷的步驟若無 1cm 最佳路徑進展則停止，
不以單純速度小判失敗；感知等待不耗此進展預算。
這個近似路徑進展不是完整的最短路徑距離，仍須依實驗調整。
PPO 預設 5120 新增步，初期兩 rollout 凍結 actor，只調 critic，
之後加入凍結轉換 BC 的輔助更新。

開始前先在舊訓練終端 Ctrl-C，確認它退出。既有 Gazebo 保持開啟：

```bash
cd /home/an/Desktop/4wd
bash scripts/train_continuous_junction.sh
```

啟動後先對轉換 BC 做各任務 3 回合 deterministic 驗證，約每 2560 步重測，
保存 `best_model.zip`、固定步數 checkpoint 與逐步 trace。
若起始驗證未能完成任務，應停止這個小實驗並檢查 trace，而非宣稱已可長訓練。
環境只允許 ROS domain 42 / partition `4wd_rl_junction`；並檢查僅有一個 cmd_vel publisher。
新腳本不會自動終止舊程序。

TensorBoard 沿用 `view_training.py --run <新訓練目錄> --follow` 的轉換方式，
服務 logdir 改為 `logs/rl_junction_continuous/training`；驗證成功率與完成時間比不同 reward 的回報更適合比較。
