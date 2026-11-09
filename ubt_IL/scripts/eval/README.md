# 策略离线评估

`eval_policy.py` 是机型无关的通用评估脚本：从 LeRobot 数据集元数据读取
`action.names`，对 checkpoint 逐帧/按块推理并计算 MSE。

## Walker C1 EDU 共创者 示例

```bash
# 容器内
python /ubt_IL/scripts/eval/eval_policy.py \
  --policy-path /ubt_IL/model/walker_c1_sim_26d_act_1RGB/checkpoints/last/pretrained_model \
  --dataset-path /ubt_IL/dataset/walker_c1_sim_26d_1RGB \
  --episodes 5 \
  --inference-freq 15 \
  --plot \
  --output /ubt_IL/scripts/eval/output/eval_walker_c1_sim_26d_act/results.json
```

- `--inference-freq 15`：模拟 15 Hz（采集帧率）的部署节奏，每块推理 1 次；
  `--inference-freq 1` 为逐帧推理（更慢，但便于观察单步误差）。
- `--plot` 会为每个关节输出 GT vs Pred 曲线到 `--plot-dir`（默认与 `--output` 同目录）。
- 输出 `results.json` 含 `summary.mean_mse / mean_mse_per_joint` 与逐 episode 明细。

## 结果目录约定

`output/eval_<机型>_<维度>_<策略>/`，例如 `output/eval_walker_c1_sim_26d_act/`。
