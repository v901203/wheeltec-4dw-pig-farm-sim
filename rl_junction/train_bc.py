#!/usr/bin/env python3
"""Offline MLP behavior cloning of LiDAR FSM demonstrations."""
import argparse
import csv
from datetime import datetime
import json
from pathlib import Path
import platform
import numpy as np
import torch
import stable_baselines3
from stable_baselines3 import PPO
from learning_common import OfflineSpaces, contract, save_model, DEFAULT_HISTORY, BASE_OBS_SIZE, check_model, model_spec
from dataset import load_episodes, summarize, split_episodes, arrays, label_metadata, sampling_weights
from policy_context import PHASE_NAMES
from tasks import TASKS
from policy_features import LocalGeometryFeatures


def metrics(policy, x, y, batch_size, groups=None):
    policy.set_training_mode(False)
    square, absolute = np.zeros(2), np.zeros(2)
    predictions = []
    with torch.no_grad():
        for start in range(0, len(x), batch_size):
            mean = policy.get_distribution(x[start:start+batch_size]).distribution.mean
            delta = (mean-y[start:start+batch_size]).cpu().numpy()
            predictions.append(delta)
            square += np.sum(delta.astype(np.float64)**2, axis=0)
            absolute += np.sum(np.abs(delta), axis=0)
    result = {"mse": float(square.sum()/(2*len(x))), "action_mae": (absolute/len(x)).tolist()}
    if groups is not None:
        delta = np.concatenate(predictions)
        tasks, phases, kinds = groups
        by_phase = {}
        for task in np.unique(tasks):
            for phase in np.unique(phases[tasks == task]):
                take = (tasks == task) & (phases == phase)
                by_phase[f"{task}/{PHASE_NAMES[phase]}"] = {
                    "mse": float(np.mean(delta[take].astype(np.float64)**2)), "labels": int(take.sum())}
        result["by_phase"] = by_phase
        task_means = [np.mean([value["mse"] for key, value in by_phase.items() if key.startswith(task+"/")])
                      for task in np.unique(tasks)]
        result["macro_phase_mse"] = float(np.mean(task_means))
        result["by_source"] = {str(int(kind)): {"mse": float(np.mean(delta[kinds == kind].astype(np.float64)**2)),
                                                  "labels": int(np.sum(kinds == kind))} for kind in np.unique(kinds)}
    return result


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--demos", type=Path, nargs="+", required=True,
                   help="One or more compatible normal/recovery/DAgger directories")
    p.add_argument("--augmentation-demos", type=Path, nargs="+", default=[],
                   help="Validated recovery/DAgger data used only for training; needs separate recovery evaluation")
    p.add_argument("--task", choices=[t.name for t in TASKS],
                   help="Omit to train all tasks present in this dataset")
    p.add_argument("--check-only", action="store_true", help="Validate real episodes and report readiness; no training")
    p.add_argument("--epochs", type=int, default=100)
    p.add_argument("--batch-size", type=int, default=256)
    p.add_argument("--history", type=int, default=DEFAULT_HISTORY)
    p.add_argument("--hidden-sizes", type=int, nargs="+", default=[128, 128])
    p.add_argument("--geometry-gain", type=float, default=1.,
                   help="Fixed policy-side centre/heading scaling, also saved for PPO/inference")
    p.add_argument("--init-model", type=Path, help="Fine-tune compatible BC weights; preserve initial model if validation worsens")
    p.add_argument("--learning-rate", type=float, default=3e-4)
    p.add_argument("--patience", type=int, default=20)
    p.add_argument("--balance", choices=["task", "phase", "phase-action"], default="phase")
    p.add_argument("--correction-fraction", type=float, default=.3)
    p.add_argument("--selection-metric", choices=["mse", "macro_phase_mse"], default="macro_phase_mse")
    p.add_argument("--split-from", type=Path, help="Preserve original normal-episode train/validation/test assignments")
    p.add_argument("--seed", type=int, default=0)
    p.add_argument("--output", type=Path, default=Path("logs/rl_junction_bc"))
    args = p.parse_args()
    if (min(args.epochs, args.batch_size, args.history, args.patience, *args.hidden_sizes) < 1 or
            not np.isfinite(args.learning_rate) or args.learning_rate <= 0 or args.seed < 0):
        p.error("sizes/counts/rate must be positive and seed nonnegative")
    if not np.isfinite(args.geometry_gain) or args.geometry_gain <= 0:
        p.error("geometry-gain must be finite and positive")
    if not 0 < args.correction_fraction < 1:
        p.error("correction-fraction must be in (0,1)")
    episodes = load_episodes(args.demos, args.task)
    if args.augmentation_demos:
        additions = load_episodes(args.augmentation_demos, args.task)
        if any(e.source == "normal" for e in additions):
            p.error("augmentation-demos accepts recovery/DAgger labels, not normal held-out data")
        for item in additions:
            item.training_only = True
        episodes.extend(additions)
        identities = [(e.task, e.seed) for e in episodes]
        if len(set(identities)) != len(identities):
            p.error("Duplicate task/seed between base and augmentation data")
    summary = summarize(episodes)
    print(json.dumps({"validated_data": summary,
                      "history": args.history, "policy_input_size": BASE_OBS_SIZE*args.history}, indent=2))
    try:
        splits = split_episodes(episodes, args.seed, args.split_from)
    except ValueError as exc:
        if args.check_only:
            print(f"NOT READY: {exc}")
            return 2
        raise
    if args.check_only:
        print("READY FOR BC")
        return 0
    tensors = {key: tuple(torch.from_numpy(a).float() for a in arrays(items, args.history))
               for key, items in splits.items()}
    groups = {key: label_metadata(items) for key, items in splits.items()}
    x, y = tensors["train"]
    vx, vy = tensors["validation"]
    # Same MLP actor class will be loaded by PPO; no architecture conversion.
    if args.init_model:
        prior = model_spec(args.init_model)
        if (prior["history"] != args.history or prior["hidden_sizes"] != args.hidden_sizes or
                prior.get("geometry_gain", 1.) != args.geometry_gain or
                set(prior["tasks"]) != set(summary)):
            p.error("init-model history/architecture/gain/tasks differ from the training specification")
        model = PPO.load(check_model(args.init_model), env=OfflineSpaces(args.history), device="cpu", seed=args.seed)
    else:
        model = PPO("MlpPolicy", OfflineSpaces(args.history), device="cpu", seed=args.seed,
                n_steps=1024, batch_size=64, learning_rate=args.learning_rate, gamma=.99,
                policy_kwargs={"net_arch": {"pi": args.hidden_sizes, "vf": args.hidden_sizes},
                    "features_extractor_class": LocalGeometryFeatures,
                    "features_extractor_kwargs": {"geometry_gain": args.geometry_gain}})
        with torch.no_grad():
            model.policy.log_std.fill_(-2.)
    actor = list(model.policy.mlp_extractor.policy_net.parameters()) + list(model.policy.action_net.parameters())
    optimizer = torch.optim.Adam(actor, lr=args.learning_rate)
    spec = {"architecture": "mlp", "history": args.history,
            "history_order": "oldest_to_newest_repeat_first_on_reset",
            "observation_size": BASE_OBS_SIZE*args.history, "hidden_sizes": args.hidden_sizes,
            "geometry_gain": args.geometry_gain,
            "tasks": sorted(summary), "actor_parameters": sum(p.numel() for p in actor)}
    run = args.output / datetime.now().strftime("%Y%m%d_%H%M%S_%f")
    run.mkdir(parents=True)
    def describe(items):
        return [{"path": str(e.path), "sha256": e.sha256, "task": e.task, "seed": e.seed,
                 "steps": len(e.actions), "valid_labels": int(e.mask.sum()),
                 "source": e.source, "training_only": e.training_only} for e in items]
    manifest = {"splits": {key: describe(items) for key, items in splits.items()},
                "seed": args.seed, "epochs": args.epochs, "batch_size": args.batch_size,
                "learning_rate": args.learning_rate, "patience": args.patience,
                "balance": args.balance, "correction_fraction": args.correction_fraction,
                "selection_metric": args.selection_metric,
                "split_from": str(args.split_from) if args.split_from else None,
                "augmentation_demos": [str(path) for path in args.augmentation_demos],
                "init_model": str(args.init_model) if args.init_model else None,
                "contract": contract(), "policy": spec, "versions": {
                    "python": platform.python_version(), "torch": torch.__version__,
                    "numpy": np.__version__, "stable_baselines3": stable_baselines3.__version__}}
    (run / "split.json").write_text(json.dumps(manifest, indent=2))
    best, best_epoch = float("inf"), 0
    # Equal task mass, then equal mass for that task's available phases.
    # Inside a phase clean/correction labels share the configured mass.
    task_codes = groups["train"][0]
    counts = {task: int(np.sum(task_codes == task)) for task in summary}
    weights = torch.tensor(sampling_weights(splits["train"], args.correction_fraction,
                           action_balance=args.balance == "phase-action") if args.balance != "task"
                           else [1./counts[t] for t in task_codes], dtype=torch.float64)
    initial = metrics(model.policy, vx, vy, args.batch_size, groups["validation"])
    if args.init_model:
        best = initial[args.selection_metric]
        save_model(model, run / "bc_model", spec)
    interrupted = False
    with (run / "metrics.csv").open("w", newline="") as log:
        writer = csv.DictWriter(log, fieldnames=["epoch", "train_mse", "validation_mse", "macro_phase_mse", "best_selection_score"])
        writer.writeheader()
        try:
            for epoch in range(1, args.epochs+1):
                model.policy.set_training_mode(True)
                order = torch.multinomial(weights, len(x), replacement=True)
                total_loss, examples = 0., 0
                for batch in order.split(args.batch_size):
                    prediction = model.policy.get_distribution(x[batch]).distribution.mean
                    loss = torch.nn.functional.mse_loss(prediction, y[batch])
                    optimizer.zero_grad()
                    loss.backward()
                    torch.nn.utils.clip_grad_norm_(actor, 1.)
                    optimizer.step()
                    total_loss += loss.item()*len(batch)
                    examples += len(batch)
                validation = metrics(model.policy, vx, vy, args.batch_size, groups["validation"])
                score = validation[args.selection_metric]
                if not np.isfinite(score):
                    raise RuntimeError("Nonfinite validation loss")
                if score < best:
                    best, best_epoch = score, epoch
                    save_model(model, run / "bc_model", spec)
                writer.writerow({"epoch": epoch, "train_mse": total_loss/examples,
                                 "validation_mse": validation["mse"], "macro_phase_mse": validation["macro_phase_mse"],
                                 "best_selection_score": best})
                log.flush()
                print(f"epoch={epoch} train_mse={total_loss/examples:.6f} "
                      f"{args.selection_metric}={score:.6f} best_epoch={best_epoch}", flush=True)
                if epoch-best_epoch >= args.patience:
                    print("Early stopping on validation loss")
                    break
        except KeyboardInterrupt:
            interrupted = True
            print("Interrupted: preserving the best completed validation checkpoint")
    has_checkpoint = (run/"bc_model.zip").exists()
    if has_checkpoint:
        best_model = PPO.load(run / "bc_model.zip", device="cpu")
        test_x, test_y = tensors["test"]
        report = {"initial_validation": initial, "best_selection_score": best,
                  "selection_metric": args.selection_metric,
                  "best_epoch": best_epoch, "interrupted": interrupted,
                  "held_out_test": metrics(best_model.policy, test_x, test_y, args.batch_size, groups["test"]),
                  "note": "Offline action errors are not closed-loop navigation success."}
        (run / "results.json").write_text(json.dumps(report, indent=2))
        print(json.dumps(report, indent=2))
        print(f"BC checkpoint: {run / 'bc_model.zip'}")
    else:
        print(f"No completed checkpoint; diagnostics: {run}")
    return 0 if has_checkpoint else 1


if __name__ == "__main__":
    raise SystemExit(main())
