# Hugging Face 数据/模型管理脚本使用指南

脚本: `ubt_IL/scripts/convert/common/hf_manager.py`

统一管理 Hugging Face **数据集(dataset)** 与 **模型(model)** 的查询、上传(push)、下载(pull)、收藏(add/delete)与卡片(card)等操作。数据集与模型共用同一套命令,通过 `--type` 区分(默认 `dataset`)。

---

## 1. 安装

### 依赖

- Python 3.10+(使用了 `str | None` 类型标注)
- `huggingface_hub`(核心依赖;已在 **1.21.0** 版本验证,`create_commit` 使用 `revision` 参数)

```bash
pip install "huggingface_hub>=1.0"
```

### 验证

```bash
python ubt_IL/scripts/convert/common/hf_manager.py --help
python ubt_IL/scripts/convert/common/hf_manager.py list --local   # 无需网络, 可立即验证
```

### 本地目录约定(默认)

| 类型 | 根目录 | 说明 |
|---|---|---|
| dataset | `ubt_IL/dataset` | 数据集根目录 |
| model | `ubt_IL/model` | 模型根目录 |

可通过全局参数 `--dataset-dir` / `--model-dir` 覆盖(见第 8 节)。

---

## 2. 登录

### Token 来源(按优先级)

1. 环境变量 `HF_TOKEN`(推荐,适合脚本/CI)
2. huggingface_hub 登录缓存 `~/.cache/huggingface/token`(由 `hf auth login` 写入)

```bash
# 方式 A: 环境变量
export HF_TOKEN=hf_xxxxxxxx

# 方式 B: 交互式登录(自动写入缓存目录)
hf auth login
```

### 国内网络:镜像端点

脚本对 `https://huggingface.co` 有 **5 秒连通性探测**,不通会直接提示设置镜像。国内环境务必先设置:

```bash
export HF_ENDPOINT=https://hf-mirror.com
```

> 注意:镜像 `hf-mirror.com` 是 HF 官方仓库的代理,**同一 `repo_id` 指向同一份数据**。收藏(`bookmarks.json`)只存 `repo_id`、不存域名,切换端点不会替换收藏内容。

### 匿名与降级

- `list` / `pull` 等只读命令允许匿名(`required=False`),可访问公开仓库;
- `push` / `card set` / `rm` / `visibility` / `duplicate` / `collection` 等写操作必须登录(`required=True`),未登录会报错退出;
- `--remote-user <账号>` 可显式指定远程命名空间(默认 = 登录账号),用于查看/操作他人公开仓库。

---

## 3. 查询展示(list)

`list` 默认同时展示 **本地 + 远程 + 收藏** 三块,按 `--type` 过滤。

```bash
python ubt_IL/scripts/convert/common/hf_manager.py list             # 默认展示数据集
python ubt_IL/scripts/convert/common/hf_manager.py list model       # 展示模型列表
```

| 参数 | 说明 | 默认 |
|---|---|---|
| `{model,dataset}`(位置参数) | 仓库类型简写,等价 `--type` | `dataset` |
| `--local` | 只列本地目录 | 关 |
| `--remote` | 只列远程仓库 | 关 |
| `--bookmarks` | 只展示收藏夹 | 关 |

三个视图参数均未指定时全部展示,指定任一后只显示所选;其余全局参数(`--type` / `--remote-user` / 目录覆盖)见第 8 节。

示例输出(模型,含简介与文件规模):

```text
[本地 model] /home/.../ubt_IL/model
    NAME                            SIZE    FILES
    Walker_S2_sim_10_2RGB_act     4.6 GB      134

[远程 model] 命名空间: qingxiangliu
    NAME                            SIZE    FILES
    Walker_S2_sim_10_2RGB_act     4.6 GB      134
        简介: walker-s2机器人仿真抓取ACT模型

[收藏] /home/.../ubt_IL/dataset/bookmarks.json
    dataset qingxiangliu/walker_pick_sort   收藏于 2026-08-16
```

说明:

- 本地 / 远程均显示 `SIZE`(总大小)与 `FILES`(文件数);远程另有 `简介`(取远程 `README.md` 首行);
- 远程查询需连通网络,未登录/网络不通时给出处理指引并跳过远程块;
- 收藏的**命名空间**条目会实时枚举该命名空间下公开仓库。

---

## 4. 上传(push)

> 将本地数据集/模型上传到 **自己的命名空间**(登录账号);仓库不存在时自动创建;目录结构原样保留。

```bash
python ubt_IL/scripts/convert/common/hf_manager.py push walker_pick_sort       # 上传数据集（type默认dataset）
python ubt_IL/scripts/convert/common/hf_manager.py push --type model tienkung_pick_up_act       # 上传模型
python ubt_IL/scripts/convert/common/hf_manager.py push walker_pick_sort --sync                 # 增量同步(只推新增/变化)
python ubt_IL/scripts/convert/common/hf_manager.py push walker_pick_sort --path meta/info.json  # 只上传单个文件
```

| 参数 | 说明 | 默认 |
|---|---|---|
| `names`(位置参数) | 本地目录名,可多个 | — |
| `--all` | 上传根目录下所有子目录 | 关 |
| `--path <rel>` | 只上传指定文件/子目录(定点添加) | — |
| `--sync` | 增量同步,只推送本地新增/变化的文件 | 关 |
| `--prune` | 配合 `--sync` 同时删除远程多余文件(危险,慎用) | 关 |
| `--delete <path...>` | 删除远程文件/子目录(可多个;可与 `--path` 同用:先删后增) | — |
| `--description` / `--description-file` | 卡片简介文本 / 从文件读取 | — |
| `--no-readme` | 不自动携带本地 README.md 作为卡片简介 | 关 |
| `--revision <ref>` | 上传到指定分支(不存在自动创建) | `main` |

`--path` / `--sync` / `--delete` / `--all` 互斥,`--prune` 仅配 `--sync`;上传时本地有 `README.md` 会自动作为卡片简介,也可上传后单独管理卡片(见 7.1)。

---

## 5. 下载(pull)

> 下载本人远程仓库或者收藏的数据集/模型。目标支持四种形式,优先级:**URL → 完整 repo_id → 收藏命中 → {账号}/短名**;仓库类型自动探测,无需 `--type`。

```bash
python ubt_IL/scripts/convert/common/hf_manager.py pull qingxiangliu/walker_pick_sort                    # 完整 repo_id
python ubt_IL/scripts/convert/common/hf_manager.py pull https://huggingface.co/datasets/qingxiangliu/walker_pick_sort   # HF 链接
python ubt_IL/scripts/convert/common/hf_manager.py pull walker_pick_sort --dry-run                       # 收藏名, 先预览
python ubt_IL/scripts/convert/common/hf_manager.py pull --all --jobs 4                                   # 并发下载全部远程仓库
```

| 参数 | 说明 | 默认 |
|---|---|---|
| `target`(位置参数) | 仓库名 / repo_id / URL / 收藏短名 | — |
| `--all` | 下载账号下所有远程仓库 | 关 |
| `--path <rel...>` | 只下载指定文件/子目录(可多个) | — |
| `--revision <ref>` | 指定分支/tag/commit(复现历史实验) | `main` |
| `--jobs N` | 并发下载仓库数 | 1 |
| `--show-card` | 下载前显示卡片简介 | 关 |

下载路径: dataset → `ubt_IL/dataset/<name>`, model → `ubt_IL/model/<name>`;本地已存在且非空时自动"合并下载"(同名文件复用,只补缺失/变更)。

---

## 6. 收藏(add / delete)

收藏别人的公开仓库的数据集/模型,便于快速查询/下载。收藏配置文件:`ubt_IL/dataset/bookmarks.json`(原子写入)。若想将其放到自己的仓库中,可参考7.5指令 duplicate永久克隆仓库。

```bash
python ubt_IL/scripts/convert/common/hf_manager.py add qingxiangliu/walker_pick_sort                                # 收藏仓库(URL / repo_id 均可)
python ubt_IL/scripts/convert/common/hf_manager.py add https://huggingface.co/qingxiangliu/datasets                 # 收藏整个命名空间
python ubt_IL/scripts/convert/common/hf_manager.py delete walker_pick_sort --dry-run     # 取消收藏(短名 / repo_id / URL 均可)
python ubt_IL/scripts/convert/common/hf_manager.py list --bookmarks                      # 查看收藏
```

| 参数 | 说明 | 默认 |
|---|---|---|
| `target`(位置参数) | HF 链接或完整 repo_id(`<账号>/<名字>`);delete 额外支持短名 | — |
| `--dry-run` | 只打印计划,不写入 | 关 |

`pull <短名>` 可直接命中收藏;收藏只存 `repo_id` 不绑定域名,切换 `HF_ENDPOINT` 镜像不影响;短名命中多个收藏时会提示改用完整 `repo_id`。

---

## 7. 其他扩展

### 7.1 卡片管理(card)

```bash
python ubt_IL/scripts/convert/common/hf_manager.py card init walker_pick_sort --dry-run    # 本地元数据生成 README.md
python ubt_IL/scripts/convert/common/hf_manager.py card get walker_pick_sort               # 查看远程卡片(需登录)
python ubt_IL/scripts/convert/common/hf_manager.py card set walker_pick_sort --text "新简介"   # 修改远程卡片(需登录)
```

| 子命令 | 说明 | 常用参数 |
|---|---|---|
| `init` | 根据本地元数据生成 README.md(本地操作) | `names...`, `--all`, `--dry-run` |
| `get` | 查看远程卡片完整内容(需登录或 `--remote-user`) | `names...`, `--all` |
| `set` | 修改远程卡片(需登录) | `--text` / `--file`, `--all`, `--dry-run` |

### 7.2 官方集合(collection)

```bash
python ubt_IL/scripts/convert/common/hf_manager.py collection create "IL 数据集" --description "说明" --private
python ubt_IL/scripts/convert/common/hf_manager.py collection add qingxiangliu/il-datasets qingxiangliu/walker_pick_sort --create
python ubt_IL/scripts/convert/common/hf_manager.py collection show qingxiangliu/il-datasets
python ubt_IL/scripts/convert/common/hf_manager.py collection list
```

| 子命令 | 说明 | 常用参数 |
|---|---|---|
| `create` | 创建集合 | `title`, `--description`, `--private` |
| `add` | 向集合添加仓库(可多个) | `slug`, `items...`, `--create` |
| `show` | 查看集合内容 | `slug` |
| `list` | 列出我的集合 | — |

### 7.3 删除远程仓库(rm, 危险)

> `delete-repo` 是 `rm` 的旧名别名。

```bash
python ubt_IL/scripts/convert/common/hf_manager.py rm qingxiangliu/old_dataset --yes   # 跳过确认
```

| 参数 | 说明 | 默认 |
|---|---|---|
| `repo_id`(位置参数) | 远程仓库 `<账号>/<名字>` | — |
| `--yes` | 跳过确认(否则需输入完整 repo_id 确认) | 关 |

### 7.4 可见性切换(visibility)

```bash
python ubt_IL/scripts/convert/common/hf_manager.py visibility qingxiangliu/walker_pick_sort --private
```

| 参数 | 说明 | 默认 |
|---|---|---|
| `repo_id`(位置参数) | 远程仓库 `<账号>/<名字>` | — |
| `--private` / `--public` | 设为私有 / 公开(二选一,必填) | — |

### 7.5 服务端复制仓库(duplicate)

> `copy` 是 `duplicate` 的旧名别名。无需本地中转,直接服务端复制。

```bash
python ubt_IL/scripts/convert/common/hf_manager.py duplicate qingxiangliu/walker_pick_sort                          # 同名复制到登录账号
python ubt_IL/scripts/convert/common/hf_manager.py duplicate qingxiangliu/walker_pick_sort --to qingxiangliu/copy --private
```

| 参数 | 说明 | 默认 |
|---|---|---|
| `target`(位置参数) | 收藏名 / repo_id / HF 链接 | — |
| `--to <repo_id>` | 目标 repo_id | 同名复制到登录账号 |
| `--private` | 新仓库设为私有 | 跟随源仓库 |

### 7.6 旧写法兼容

- `--list model` / `--add <url>` 等 `--<命令>` 旧写法自动转为子命令;
- 省略子命令时:URL 首参 → `add`,本地存在的目录名 → `push`;
- 命令别名汇总:`upload` → `push`,`download` → `pull`,`delete-repo` → `rm`,`copy` → `duplicate`。

---

## 8. 全局参数(任意命令后可追加)

| 参数 | 说明 | 默认 |
|---|---|---|
| `--type {dataset,model}` | 仓库类型 | `dataset` |
| `--dataset-dir <dir>` | 数据集根目录 | `ubt_IL/dataset` |
| `--model-dir <dir>` | 模型根目录 | `ubt_IL/model` |
| `--remote-user <账号>` | 远程命名空间(账号/组织) | 登录账号 |
| `--dry-run` | 只打印计划,不真正执行 | 关 |
| `--revision <ref>` | pull: 下载分支/tag/commit;push: 上传分支 | `main` |

```bash
python ubt_IL/scripts/convert/common/hf_manager.py push --type model --model-dir /data/models tienkung_pick_up_act
python ubt_IL/scripts/convert/common/hf_manager.py pull --remote-user qingxiangliu Walker_S2_sim_10_2RGB_act
```

---

## 9. 常见问题

**Q: `--list` 时远程显示"未登录或网络不可用"?**
先检查网络与镜像:

```bash
export HF_ENDPOINT=https://hf-mirror.com
export HF_TOKEN=hf_xxx          # 或 hf auth login
curl -I https://huggingface.co  # 应返回 HTTP 200
```

**Q: 为什么 `--type model` 也要带镜像才能用?**
本机直连 `huggingface.co` 会 5 秒超时,登录探测失败后降级匿名;设置 `HF_ENDPOINT=https://hf-mirror.com` 后即可正常查询/下载。

**Q: 上传报 `create_commit() got an unexpected keyword argument 'branch'`?**
`huggingface_hub` 1.0+ 中 `create_commit` 参数由 `branch` 改为 `revision`。请升级到已适配的版本:

```bash
pip install -U "huggingface_hub>=1.0"
```

**Q: `card set --type model` 报 `unrecognized arguments`?**
嵌套子命令(`card set` / `collection add` 等)已注册全局参数;请确认脚本为最新版本(该问题已在 `add_common_args` 统一注册后修复)。

**Q: 本地模型目录没有 README.md?**
`pull` 会连同远程 `README.md` 一起下载;也可用 `card init <name>` 根据本地文件自动生成。
