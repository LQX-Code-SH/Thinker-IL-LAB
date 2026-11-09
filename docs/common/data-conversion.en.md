# Data Conversion (HDF5 -> LeRobot)

The conversion scripts turn raw HDF5 data from each robot into a LeRobot v3 dataset (`meta/`, `chunk-000/` data parquet, `videos/`) for training, evaluation, and visualization.

## Script layout

| Robot | Script | Config |
|------|------|------|
| `tienkung_pro` | `ubt_IL/scripts/convert/tienkung_pro/` (`convert.sh`, `convert_grasp_bottle.sh`) | `ubt_IL/scripts/convert/tienkung_pro/configs/` |
| `walker_s2` | `ubt_IL/scripts/convert/walker_s2/` (`convert.sh`, `convert_real_to_lerobot_v3.py`) | `ubt_IL/scripts/convert/walker_s2/configs/` |

Shared converters (not robot-specific) live in `ubt_IL/scripts/convert/common/`:

| Script | Description |
|------|------|
| `convert_to_lerobot.py` | Generic HDF5 -> LeRobot converter (called by each robot's `*.sh`) |
| `isaaclab2lerobot.py` / `isaaclab2lerobotv3.py` | IsaacLab simulation conversion (v2/v3) |
| `lerobot2isaaclab.py` | Reverse conversion from LeRobot to a single HDF5 |
| `fix_stats_format.py` | Repair dataset stats formatting |
| `merge_datasets.sh` | Merge multiple dataset batches (e.g. several real-robot batches) |
| `hf_manager.py` | Hugging Face dataset/model management (see [HF Management](hf-manage.md)) |
| `all_robot_h5_info*.md` | Cross-robot HDF5 layout reference (see [HDF5 Layouts](hdf5-layouts.md)) |

## Environment variables

Each robot's `convert.sh` passes through these environment variables (defaults can be omitted; see `bash convert.sh -h` for the rest):

| Variable | Default | Description |
|------|--------|------|
| `SRC_ROOT` | Per robot | HDF5 source root (one subdirectory per episode containing the HDF5 file) |
| `TGT_PATH` | `/ubt_IL/dataset` | LeRobot output root |
| `CONFIG` | Per robot | Conversion config JSON: field filtering + joint/camera mapping |
| `REPO_ID` | - | Output dataset name (local directory name + HF repo name; keep it consistent with training) |
| `ROBOT_TYPE` | - | Robot type (`tienkung` / `walker_s2`, written to metadata) |
| `TASK_NAME` | - | Task name (written to each frame's `task` field and used as the language condition) |
| `FPS` | Per robot (15/13) | Target frame rate; `auto` uses the source HDF5 rate |
| `VCODEC` | `h264` | Video codec (`h264`/`jpeg`/`png`) |
| `HDF5_REL_PATH` | Per robot | HDF5 path relative to each episode directory |
| `RESAMPLE_FPS` | Empty (no resampling) | Target frame rate; enables resampling |
| `LABEL_ROOT` | Empty | Label root (used when task labels exist) |
| `TRIM_STATIONARY` | Empty (off) | `1` enables stationary-frame trimming (see below) |

> Pass-through options: `--overwrite` replaces an existing dataset; `--save_one true` uses single-file mode.

## Stationary-frame trimming

Stationary frames can trap the policy in a **local stationary** state (motion stops). Set `TRIM_STATIONARY=1` during conversion to truncate stationary runs longer than the cap.

| Variable | Default | Description |
|------|------|------|
| `STATIONARY_KEY` | `action` | Key used for stationary detection |
| `STATIONARY_WINDOW` | Auto (`0.3 × fps`) | Stationary detection window |
| `STATIONARY_THRESH` | `0.03` | Detection threshold (**normalized**, handling the scale difference between grip travel 0.05 and joint travel 0.5) |
| `STATIONARY_CAP` | `8` | Maximum consecutive stationary frames |
| `STATIONARY_MIN_RUN` | `3` | Minimum valid run length |
| `STATIONARY_RANGE_EPS` | `1e-3` | Stationary interval tolerance |
| `STATIONARY_DIAGNOSE` | Empty | `1` only reports the stationary distribution without writing data (for threshold calibration) |

```bash
# Usage: prefix the conversion command with TRIM_STATIONARY=1
TRIM_STATIONARY=1 \
CONFIG=... SRC_ROOT=... TGT_PATH=... REPO_ID=... \
bash /ubt_IL/scripts/convert/<robot>/convert.sh
```

## FAQ

| Problem | Fix |
|------|------|
| `FileNotFoundError` (HDF5) | Wrong `SRC_ROOT`; make sure HDF5 was synced to a container-accessible directory |
| "config does not exist" | Specify an existing config file under `configs/` |
| Real-robot data reports a missing depth stream (TienKung) | Real HDF5 has no depth; use a `*_real.json` config (RGB only) |
| Normalization explodes during training (QUANTILES NaN/Inf) | Constant dimensions (locked joints/grippers) give std=0 and division by zero; convert with a lower-dimensional config that drops dead dimensions |
| Dataset frame rate differs from expected | Use `FPS=auto` for the source rate or `RESAMPLE_FPS` to resample |

For robot-specific commands and config choices, see [Walker TienKung Pro](../tienkung/convert-train.md) / [Walker S2 EDU](../walker-s2/convert-train.md).
