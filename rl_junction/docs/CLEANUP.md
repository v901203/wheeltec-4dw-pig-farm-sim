# RL 資料夾整理（2026-11-04）

已刪除沒有程式引用的舊航點教師 `teacher.py`、舊簡化場景產生器 `make_scene.py`，以及 RL Python 快取。
目前使用原始豬舍地圖與 `prepare_scene.py`；正式示範教師為 `fsm_teacher.py`。

歷次 BC、幾何與 PPO 文件移至 `history/`；原本的 README 保留為 `history/HYBRID_WORKFLOW.md`。
訓練資料、模型、診斷日誌及現行程式未刪除。

另清理已失效的 `scripts/launch_rl_training.sh`，其呼叫的 `rl_pig_pen/train_ppo.py` 已不存在；刪除九月舊 PPO 的 52 份日誌／PID 檔，約 462 KiB。保留新模式、對照模型、示範切分引用與目前執行目錄。
