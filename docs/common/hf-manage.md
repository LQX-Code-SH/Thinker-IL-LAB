# HF 数据/模型管理（hf_manager.py）

脚本：`ubt_IL/scripts/convert/common/hf_manager.py`

统一管理 Hugging Face **数据集（dataset）** 与 **模型（model）** 的查询、上传（push）、下载（pull）、收藏（add/delete）与卡片（card）等操作。数据集与模型共用同一套命令，通过 `--type` 区分（默认 `dataset`）。

## 安装与目录约定

依赖 Python 3.10+ 与 `huggingface_hub>=1.0`：

```bash
pip install "huggingface_hub>=1.0"
python ubt_IL/scripts/convert/common/hf_manager.py list --local   # 无需网络，可立即验证
```

| 类型 | 本地根目录（默认） |
|---|---|
| dataset | `ubt_IL/dataset` |
| model | `ubt_IL/model` |

可通过全局参数 `--dataset-dir` / `--model-dir` 覆盖。

## 登录与镜像

Token 来源（按优先级）：环境变量 `HF_TOKEN`（推荐）-> huggingface_hub 登录缓存（`hf auth login`）。

```bash
export HF_TOKEN=hf_xxxxxxxx     # 方式 A
hf auth login                   # 方式 B（交互式）
```

国内网络务必先设置镜像（脚本对 `huggingface.co` 有 5 秒连通性探测，不通会提示）：

```bash
export HF_ENDPOINT=https://hf-mirror.com
```

- 只读命令（`list` / `pull`）允许匿名访问公开仓库；写操作（`push` / `card set` / `rm` 等）必须登录。
- `--remote-user <账号>` 可显式指定远程命名空间，用于查看/操作他人公开仓库。

## 查询展示（list）

```bash
python hf_manager.py list              # 默认展示数据集（本地 + 远程 + 收藏）
python hf_manager.py list model        # 展示模型列表
python hf_manager.py list --bookmarks  # 只看收藏夹
python hf_manager.py list --type model --local   # 本地模型
```

| 参数 | 说明 | 默认 |
|---|---|---|
| `{model,dataset}`（位置参数） | 仓库类型简写，等价 `--type` | `dataset` |
| `--local` / `--remote` / `--bookmarks` | 只列本地 / 远程 / 收藏 | 关 |

## 上传（push）

将本地数据集/模型上传到自己的命名空间；仓库不存在时自动创建，目录结构原样保留。

```bash
python hf_manager.py push walker_pick_sort                        # 上传数据集
python hf_manager.py push --type model tienkung_pick_up_act       # 上传模型
python hf_manager.py push walker_pick_sort --sync                 # 增量同步（只推新增/变化）
python hf_manager.py push walker_pick_sort --path meta/info.json  # 只上传单个文件
python hf_manager.py push --all                                   # 一键全传
```

| 参数 | 说明 | 默认 |
|---|---|---|
| `names`（位置参数） | 本地目录名，可多个 | - |
| `--all` | 上传根目录下所有子目录 | 关 |
| `--path <rel>` | 只上传指定文件/子目录 | - |
| `--sync` | 增量同步，只推送本地新增/变化的文件 | 关 |
| `--prune` | 配合 `--sync` 同时删除远程多余文件（危险，慎用） | 关 |
| `--delete <path...>` | 删除远程文件/子目录 | - |
| `--description` / `--description-file` | 卡片简介文本 / 从文件读取 | - |
| `--revision <ref>` | 上传到指定分支 | `main` |

## 下载（pull）

目标支持四种形式，优先级：**URL -> 完整 repo_id -> 收藏命中 -> {账号}/短名**；仓库类型自动探测。

```bash
python hf_manager.py pull qingxiangliu/walker_pick_sort                    # 完整 repo_id
python hf_manager.py pull https://huggingface.co/datasets/qingxiangliu/walker_pick_sort   # HF 链接
python hf_manager.py pull walker_pick_sort --dry-run                       # 收藏名，先预览
python hf_manager.py pull --all --jobs 4                                   # 并发下载全部远程仓库
```

| 参数 | 说明 | 默认 |
|---|---|---|
| `--all` | 下载账号下所有远程仓库 | 关 |
| `--path <rel...>` | 只下载指定文件/子目录 | - |
| `--revision <ref>` | 指定分支/tag/commit（复现历史实验） | `main` |
| `--jobs N` | 并发下载仓库数 | 1 |
| `--show-card` | 下载前显示卡片简介 | 关 |

下载路径：dataset -> `ubt_IL/dataset/<name>`，model -> `ubt_IL/model/<name>`；本地已存在且非空时自动"合并下载"（同名文件复用，只补缺失/变更）。

## 收藏（add / delete）

收藏别人的公开仓库，便于快速查询/下载。收藏配置文件：`ubt_IL/dataset/bookmarks.json`。

```bash
python hf_manager.py add qingxiangliu/walker_pick_sort                    # 收藏仓库（URL / repo_id 均可）
python hf_manager.py add https://huggingface.co/qingxiangliu/datasets     # 收藏整个命名空间
python hf_manager.py delete walker_pick_sort --dry-run                    # 取消收藏
python hf_manager.py list --bookmarks                                     # 查看收藏
```

> 若想将收藏仓库放到自己的账号中，可用 `duplicate` 服务端永久克隆（见下）。

## 其他扩展

| 子命令 | 说明 | 示例 |
|---|---|---|
| `card init/get/set` | 卡片管理（`init` 本地生成 README；`get` 查看远程；`set` 修改远程，需登录） | `card init walker_pick_sort --dry-run` |
| `collection create/add/show/list` | 官方集合管理 | `collection add <slug> <items...> --create` |
| `rm`（危险） | 删除远程仓库 | `rm qingxiangliu/old_dataset --yes` |
| `visibility` | 公开/私有切换 | `visibility <repo_id> --private` |
| `duplicate` | 服务端复制仓库（无需本地中转） | `duplicate <repo_id> --to <新repo_id> --private` |

**旧写法兼容**：`upload` -> `push`、`download` -> `pull`、`delete-repo` -> `rm`、`copy` -> `duplicate`；`--list model` / `--add <url>` 等旧写法自动转为子命令。

## 全局参数

| 参数 | 说明 | 默认 |
|---|---|---|
| `--type {dataset,model}` | 仓库类型 | `dataset` |
| `--dataset-dir <dir>` / `--model-dir <dir>` | 本地根目录覆盖 | `ubt_IL/dataset` / `ubt_IL/model` |
| `--remote-user <账号>` | 远程命名空间（账号/组织） | 登录账号 |
| `--dry-run` | 只打印计划，不真正执行 | 关 |
| `--revision <ref>` | pull: 下载分支；push: 上传分支 | `main` |

## 常见问题

**Q: `list` 时远程显示"未登录或网络不可用"?**
先检查网络与镜像：`export HF_ENDPOINT=https://hf-mirror.com`、`export HF_TOKEN=hf_xxx`、`curl -I https://huggingface.co`。

**Q: 上传报 `create_commit() got an unexpected keyword argument 'branch'`?**
`huggingface_hub` 1.0+ 参数由 `branch` 改为 `revision`，请升级：`pip install -U "huggingface_hub>=1.0"`。

**Q: 本地模型目录没有 README.md?**
`pull` 会连同远程 `README.md` 一起下载；也可用 `card init <name>` 根据本地文件自动生成。
