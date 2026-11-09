# 路口機器學習實驗

目標是混合導航：長距離直線由傳統控制處理，模型從路口減速開始，
自主完成左轉、右轉或倒退穿越，進入正確走道後再交回直線控制。
目前主要開發 **連續路口控制模式**，尚未完成 Gazebo 閉迴路驗證與實車部署。

## 目前入口

| 用途 | 程式／說明 |
| --- | --- |
| 新模式架構、操作、相容性限制 | [連續控制說明](CONTINUOUS_CONTROL.md) |
| 新模式離線 BC 轉換／短輪 PPO | `continuous_train.py` |
| 新模式模擬環境 | `continuous_env.py` |
| 新模式觀察轉換／模型版本檢查 | `continuous_common.py` |
| 從專案根目錄啟動短輪訓練 | `bash scripts/train_continuous_junction.sh` |
| 開啟原豬舍地圖的獨立訓練環境 | `bash scripts/launch_rl_junction.sh --gui` |
| TensorBoard 資料轉換與進度查看 | `view_training.py` |
| 定期驗證、最佳模型、分段保存 | `training_checkpoints.py` |

模型輸入不使用影像或世界座標；雷達、速度、任務與局部幾何提供觀察。
安全檢查和最終走道交接判斷保留。新模式取消路口內強制置中／轉彎的 FSM 關卡。

不要同時啟動多個控制器。切換實驗前，先停止舊訓練，讓其送零速度並退出。
初始 BC 驗證、訓練步數和導航成功率是不同指標；起始驗證期間 PPO 步數可能仍為 0。

## 仍保留的共用與比較程式

這些不是過期垃圾，尚有資料、測試或新模式依賴：

- `gazebo_env.py`、`tasks.py`、`prepare_scene.py`：Gazebo 通訊、地圖及計分幾何。
- `learning_common.py`、`policy_features.py`：歷史觀察與 MLP 支援。
- `fsm_teacher.py`、`junction_env.py`、`policy_context.py`：原始示範與新模式接手前的巡航／幾何取得。
- `collect_demos.py`、`dataset.py`、`train_bc.py`、`behavior_policy.py`：示範、資料檢查、BC／DAgger。
- `train.py`、`phase_training.py`、`evaluate.py`：原 FSM 分階段混合模式的比較基準。
- `geometry_recording.py`、`assess_geometry.py`：幾何診斷與原模式啟動門檻。
- `upgrade_demos.py`、`transfer_checkpoint.py`、`recovery.py`、`pipeline.py`：資料移轉、恢復示範與舊流程支援。
- `tests/`：學習模組測試。

新舊模式使用不同模型 contract，不能互相直接載入；BC 需明確轉換後才能用於新模式。

## 資料位置

| 目錄 | 用途 |
| --- | --- |
| `logs/rl_junction_continuous/` | 目前新模式 BC、驗證、PPO |
| `logs/rl_junction_ppo_improved/` | 已中斷的階段式改善實驗 |
| `logs/rl_junction_ppo_100k/` | 十萬步退化對照、曲線與最終模型 |
| `logs/rl_junction_geometry_work/` | 示範切分依賴、BC、早期 PPO 與幾何證據 |
| `logs/rl_junction_ppo_continuation/` | 接續 PPO 與幾何評估報告 |

歷史資料含來源雜湊與切分引用，尚未搬動或刪除，避免破壞重現與模型比較。

## 歷史紀錄

- [原分階段混合模式完整操作](docs/history/HYBRID_WORKFLOW.md)
- [十萬步退化診斷與階段式改善](docs/history/PPO_REGRESSION.md)
- [幾何修正與 PPO](docs/history/GEOMETRY_AND_PPO.md)
- [接續訓練與 TensorBoard](docs/history/PPO_CONTINUATION.md)
- [BC 改善](docs/history/BC_IMPROVEMENT.md)
- [恢復示範／DAgger](docs/history/RECOVERY_BC.md)
- [早期驗證](docs/history/VERIFICATION.md)、[v6 驗證](docs/history/V6_VERIFICATION.md)
- [此次整理內容](docs/CLEANUP.md)
