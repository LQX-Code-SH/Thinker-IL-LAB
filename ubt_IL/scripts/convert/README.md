# convert — HDF5 -> LeRobot 数据转换

宿主机 conda 环境运行。

| 机型 | 脚本 | 配置 |
|------|------|------|
| `tienkung_pro` | [tienkung_pro/](tienkung_pro/)(`convert.sh`、`convert_grasp_bottle.sh`) | [tienkung_pro/configs/](tienkung_pro/configs/) |
| `walker_s2` | [walker_s2/](walker_s2/)(`convert.sh`、`convert_real_to_lerobot_v3.py`) | [walker_s2/configs/](walker_s2/configs/) |

通用转换器(非机型专属)在 [common/](common/):

- `convert_to_lerobot.py` — 通用 HDF5->LeRobot 转换器(被 `tienkung_pro/*.sh` 调用)
- `isaaclab2lerobot.py` / `isaaclab2lerobotv3.py` — IsaacLab 仿真数据转换(v2/v3)
- `lerobot2isaaclab.py` — LeRobot -> 单 HDF5 反向转换
- `hf_manager.py` — Hugging Face 数据集与模型管理(git 风格:push / pull / add / delete / list / card)。支持收藏他人开源仓库、一键上传/拉取所有本地子数据集。常用示例:

  ```bash
  # 查询(本地 + 远程, 默认数据集; --type model 看模型)
  python hf_manager.py list --bookmarks            # 查看收藏夹
  python hf_manager.py list --type model --local   # 本地模型

  # 上传到自己的 HF 账号(push = 旧 upload; --path 增量传单文件/子目录; --all 一键全传)
  python hf_manager.py push walker_pick_sort
  python hf_manager.py push --all
  python hf_manager.py push walker_pick_sort --path chunk-000/file-000.parquet

  # 下载到本地(pull = 旧 download; 支持 repo_id / URL / 收藏名; --all 一键全拉)
  python hf_manager.py pull qingxiangliu/walker_pick_sort
  python hf_manager.py pull --all

  # 收藏别人的开源仓库(本地操作, 无需登录)
  python hf_manager.py add https://huggingface.co/datasets/qingxiangliu/walker_pick_sort
  python hf_manager.py delete qingxiangliu/walker_pick_sort
  ```

  说明:
  - **本地路径**:数据集默认 `ubt_IL/dataset`,模型默认 `ubt_IL/model`(`--dataset-dir` / `--model-dir` 可覆盖)
  - **远程仓库**默认在登录账号命名空间(`<账号>/<名字>`), 可用全局参数 `--remote-user <账号/组织>` 访问他人/团队已上传的仓库
  - **仓库类型** `--type dataset|model`(默认 dataset), URL / 收藏命中时自动推断
  - **登录要求**:写操作(push / card set)需登录(`export HF_TOKEN=hf_xxx` 或 `hf auth login`);读操作(pull / list --remote / add / delete)访问公开仓库可不登录(匿名只读)
  - 旧名兼容:`upload` = `push`、`download` = `pull`;收藏夹存于 `ubt_IL/dataset/.dataset/bookmarks.json`
  - 国内网络可先设置 `export HF_ENDPOINT=https://hf-mirror.com`
- `all_robot_h5_info*.md` — 跨机型 HDF5 布局参考文档

`tienkung_pro` 的 shell 脚本通过 `$SCRIPT_DIR/../common/convert_to_lerobot.py` 调用通用转换器;`PROJECT_ROOT` 由 `$SCRIPT_DIR/../../..` 推导。
