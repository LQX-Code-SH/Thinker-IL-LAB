#!/usr/bin/env python3
"""
管理 Hugging Face 数据集与模型: 查询 / 上传(push) / 下载(pull) / 收藏(add/delete) / 卡片。

设计模式:
    - 命令模式(Command): 每条子命令封装为独立对象(ListCommand/PushCommand/...),
      自包含参数定义、参数校验与执行逻辑; 新增命令只需实现 Command 接口并注册。
    - 策略模式(Strategy): dataset/model 的差异(list 接口/标签/收藏桶/本地目录)
      由 RepoTypeStrategy 子类封装, 消除散落各处的 if/else 分支。
    - 门面模式(Facade): Context 封装命令共享的运行环境(按需登录、命名空间解析、本地根目录)。
    - 工厂模式(Factory): build_parser() 由命令对象注册各自参数, 集中构建 argparse 解析器。

用法:
    # 1) 展示数据, 默认=本地 + HF 远程仓库 + 收藏夹(三者均按类型区分 dataset / model)
    python hf_manager.py list
    python hf_manager.py list model          # 简写: == list --type model(列出模型)
    python hf_manager.py --list model        # 旧写法同样支持
    python hf_manager.py list --local        # 只列出本地目录
    python hf_manager.py list --remote       # 只列出 HF 远程仓库
    python hf_manager.py list --bookmarks    # 只展示收藏夹(按当前类型过滤)

    # 2) 上传数据集(默认走镜像 huggingface.co, 国内可用 hf-mirror.com)
    python hf_manager.py push <name> [--description "说明文字"]
    python hf_manager.py push --all --dry-run     # 全量上传, 先 dry-run 预览
    python hf_manager.py push <name> --path <relative_path>   # 增量上传单文件/子目录
    python hf_manager.py push <name> --delete <remote_path>   # 删除远程单文件/子目录(可多个)
    python hf_manager.py push <name> --sync        # 增量同步: 仅推送本地新增/变化的文件
    python hf_manager.py push <name> --sync --prune   # 同步并删除远程多余文件(危险, 慎用)
    python hf_manager.py push <name> --revision dev    # 上传到指定分支(不存在自动创建, 默认 main)

    # 3) 下载数据集(支持 repo_id、URL、收藏名)
    python hf_manager.py pull <repo_id>            # 如 qingxiangliu/walker_pick_sort
    python hf_manager.py pull <hf 链接>            # 如 https://huggingface.co/datasets/xxx/yyy
    python hf_manager.py pull <收藏名> --dry-run
    python hf_manager.py pull <repo_id> --path data/chunk-000 meta/info.json  # 定向下载多个文件/子目录
    python hf_manager.py pull <repo_id> --revision <tag|commit> # 下载指定分支/tag/commit(复现实验)
    python hf_manager.py pull <repo_id> --jobs 4                # 并发下载多个仓库(--jobs N)

    # 4) 收藏别人的仓库/命名空间, 便于后续按名下载
    python hf_manager.py add https://huggingface.co/datasets/qingxiangliu/walker_pick_sort
    python hf_manager.py add https://huggingface.co/qingxiangliu/datasets   # 收藏命名空间
    python hf_manager.py delete <repo_id|收藏名|URL> [--dry-run]

    # 5) 查看/生成/修改数据集卡片
    python hf_manager.py card init <name> --dry-run    # 根据本地元数据生成 README.md
    python hf_manager.py card get  <name>              # 查看卡片内容
    python hf_manager.py card set  <name> --text "新内容"

    # 6) 官方集合 / 仓库管理
    python hf_manager.py collection create "IL 数据集" --private
    python hf_manager.py collection add  <slug> <repo_id> [--create]
    python hf_manager.py collection show <slug>
    python hf_manager.py collection list
    python hf_manager.py rm <repo_id> --yes          # 删除远程仓库(delete-repo 别名)
    python hf_manager.py visibility <repo_id> --private   # 公开/私有切换
    python hf_manager.py duplicate <收藏名|repo_id|URL> [--to <新repo_id>] [--private]
                                                    # 服务端复制仓库到自己账号(无需本地中转)

    # 7) 常用选项(任意命令后追加)
    --type model               # 仓库类型: dataset(默认) | model
    --remote-user <account>    # 远程命名空间, 默认=登录账号
    --dataset-dir <dir>        # 数据集根目录(默认 ubt_IL/dataset)
    --model-dir <dir>          # 模型根目录(默认 ubt_IL/model)
    --dry-run                  # 只打印计划, 不真正执行
    --revision <ref>           # pull: 下载指定分支/tag/commit; push: 上传到指定分支(默认 main)
"""

import argparse
import json
import os
import re
import socket
import sys
from abc import ABC, abstractmethod
from datetime import date
from pathlib import Path

try:
    from huggingface_hub import (
        CommitOperationAdd,
        CommitOperationDelete,
        HfApi,
        duplicate_repo,
        hf_hub_download,
        snapshot_download,
    )
except ImportError:
    print("[错误] 缺少依赖 huggingface_hub, 请先安装:", file=sys.stderr)
    print("    pip install huggingface_hub", file=sys.stderr)
    sys.exit(1)

# ── 常量 ──────────────────────────────────────────────────────────────────────
TYPE_DATASET = "dataset"
TYPE_MODEL = "model"
TYPES = (TYPE_DATASET, TYPE_MODEL)

DEFAULT_DATASET_DIR = Path(__file__).resolve().parents[3] / "dataset"
DEFAULT_MODEL_DIR = Path(__file__).resolve().parents[3] / "model"

DEFAULT_CARD_TEXT = """---
license: apache-2.0
task_categories:
- robotics
language:
- en
---

# 数据集卡片 (Auto-generated)

本卡片由 hf_manager 自动生成, 请补充数据说明后手动完善。
"""


# ── 策略模式: 仓库类型(dataset / model) ─────────────────────────────────────
class RepoTypeStrategy(ABC):
    """封装 dataset/model 的行为差异: 远程列表接口 / 中文标签 / 收藏桶名 / 本地目录。"""

    label = ""      # 中文标签: "数据集" / "模型"
    plural = ""     # 收藏桶名: "datasets" / "models"
    dir_attr = ""   # 本地根目录参数名: "dataset_dir" / "model_dir"

    @abstractmethod
    def list_remote(self, api: HfApi, user: str):
        """返回该类型下命名空间 user 的远程仓库列表(已排序, 可多次遍历/判空)。"""


class DatasetStrategy(RepoTypeStrategy):
    label = "数据集"
    plural = "datasets"
    dir_attr = "dataset_dir"

    def list_remote(self, api, user):
        return sorted(api.list_datasets(author=user), key=repo_name)


class ModelStrategy(RepoTypeStrategy):
    label = "模型"
    plural = "models"
    dir_attr = "model_dir"

    def list_remote(self, api, user):
        # full=True 才会填充 lastModified 等完整信息, 否则 UPDATED 列显示 ?
        return sorted(api.list_models(author=user, full=True), key=repo_name)


STRATEGIES: dict[str, RepoTypeStrategy] = {
    TYPE_DATASET: DatasetStrategy(),
    TYPE_MODEL: ModelStrategy(),
}


def strategy_of(repo_type: str) -> RepoTypeStrategy:
    """策略工厂: 仓库类型 -> 对应策略对象。"""
    return STRATEGIES[repo_type]


def detect_repo_type(api: HfApi, repo_id: str) -> str | None:
    """探测 repo_id 的仓库类型(dataset / model), 不存在或无权限时返回 None。"""
    for t in TYPES:
        try:
            api.repo_info(repo_id=repo_id, repo_type=t)
            return t
        except Exception:  # noqa: BLE001  (不存在/无权限/网络异常均视为未命中)
            continue
    return None


# ── 登录与共享环境 ──────────────────────────────────────────────────────────
def hf_endpoint() -> str:
    """HF 服务地址(支持镜像): 默认 huggingface.co, 可经 HF_ENDPOINT 覆盖。"""
    return os.environ.get("HF_ENDPOINT", "https://huggingface.co").rstrip("/")


def probe_endpoint(timeout: float = 5.0) -> bool:
    """对 HF 服务做快速 TCP 连通性探测(默认 5 秒), 避免后续请求长时间无输出。

    从 HF_ENDPOINT 解析主机与端口(http 用 80, https 用 443, 支持自定义端口如 localhost:8000)。
    """
    from urllib.parse import urlsplit
    u = urlsplit(hf_endpoint())
    host = u.hostname or "huggingface.co"
    port = u.port or (443 if u.scheme == "https" else 80)
    try:
        with socket.create_connection((host, port), timeout=timeout):
            return True
    except OSError:
        return False


def has_token() -> bool:
    """判断本地是否已配置 HF token(环境变量或缓存文件)。"""
    if os.environ.get("HF_TOKEN"):
        return True
    for p in (Path.home() / ".cache/huggingface/token",
              Path.home() / ".huggingface/token"):
        if p.is_file():
            return True
    return False


def ensure_login(required: bool = True) -> tuple[HfApi | None, str | None]:
    """确保已登录, 返回 (api, username)。

    required=True 且未登录/登录失败时直接退出; required=False 时降级为 (None, None)。
    """
    # 1) 快速连通性探测(5 秒): 不通立即提示镜像, 避免 whoami() 悬挂
    if not probe_endpoint():
        msg = (f"[登录] 无法连接 {hf_endpoint()} (5 秒探测超时)\n"
               "  网络可能不通, 国内请先设置镜像再重试:\n"
               "  export HF_ENDPOINT=https://hf-mirror.com")
        if required:
            sys.exit(msg)
        print(msg + "\n[登录] 以匿名模式跳过远程查询(仅本地操作)", flush=True)
        return None, None

    # 2) token 检查
    if not has_token():
        if required:
            sys.exit("[错误] 需要登录才能执行此操作(未检测到 HF_TOKEN)\n"
                     "  方式A(推荐): export HF_TOKEN=hf_xxx\n"
                     "  方式B: hf auth login\n"
                     "  国内镜像可加: export HF_ENDPOINT=https://hf-mirror.com")
        print("[登录] 未配置 token, 以匿名方式访问(仅可读公开仓库)", flush=True)
        return None, None

    # 3) whoami 校验(显式 10 秒超时, 失败按 required 决定退出或降级匿名)
    api = HfApi()
    print("[登录] 连接 HF 服务...", flush=True)
    try:
        who = api.whoami()
    except Exception as e:  # noqa: BLE001  (网络/超时/token 失效都可能抛出)
        if required:
            sys.exit(f"[错误] 连接 HF 失败: {type(e).__name__}: {e}\n"
                     "  国内网络建议先设置镜像: export HF_ENDPOINT=https://hf-mirror.com")
        print(f"[登录] 连接 HF 失败(降级为匿名): {type(e).__name__}", flush=True)
        return None, None
    print(f"[登录] 账号: {who['name']}", flush=True)
    return api, who["name"]


class Context:
    """门面(Facade): 封装命令共享的运行环境——按需登录、命名空间解析、本地根目录。"""

    def __init__(self, args):
        self.args = args
        self._api = None
        self._username = None
        self._resolved = False

    def login(self, required: bool = True):
        """确保已登录(幂等, 多次调用只登录一次)。返回 (api, username)。

        required=True 时未登录/登录失败直接退出; False 时降级为 (None, None), 由调用方决定匿名行为。
        """
        if not self._resolved:
            self._api, self._username = ensure_login(required=required)
            self._resolved = True
        return self._api, self._username

    @property
    def remote_user(self) -> str | None:
        """远程命名空间: 显式 --remote-user 优先, 否则用登录账号。"""
        return self.args.remote_user or self._username

    def local_root(self, repo_type: str | None = None) -> Path:
        """按仓库类型返回本地根目录(不存在则自动创建)。"""
        st = strategy_of(repo_type or self.args.repo_type)
        root = Path(getattr(self.args, st.dir_attr)).resolve()
        root.mkdir(parents=True, exist_ok=True)
        return root


# ── 通用工具 ─────────────────────────────────────────────────────────────────
def human_size(n: int | float) -> str:
    for unit in ("B", "KB", "MB", "GB", "TB"):
        if n < 1024:
            return f"{n:.1f} {unit}" if unit != "B" else f"{int(n)} B"
        n /= 1024
    return f"{n:.1f} PB"


def repo_name(repo) -> str:
    """从仓库信息对象提取仓库短名(author/name 中的 name), 命名空间由调用方拼接。"""
    return repo.id.split("/")[-1] if hasattr(repo, "id") else str(repo)


def looks_like_url(s: str) -> bool:
    return s.startswith(("http://", "https://"))


def parse_hf_url(url: str) -> dict:
    """解析 HF 链接, 返回 {kind, type?, namespace, repo_id?, name?}。

    kind: namespace(列表页, 如 .../qingxiangliu/datasets) | repo(仓库页)。
    type: 仅当 URL 带 /datasets/ 或 /models/ 前缀时确定; 裸 <ns>/<name>
          链接(如模型页 https://huggingface.co/ns/name)返回 None, 由调用方按 --type 兜底。
    """
    from urllib.parse import urlsplit
    u = urlsplit(url)
    if u.scheme not in ("http", "https"):
        raise ValueError(f"[错误] 不支持的协议: {u.scheme}, 仅支持 http/https")
    host = u.netloc.split(":")[0]
    if host not in ("huggingface.co", "hf-mirror.com"):
        raise ValueError(f"[错误] 非 Hugging Face 域名: {host}")
    parts = [p for p in u.path.split("/") if p]
    if not parts:
        raise ValueError(f"[错误] 无法解析 HF 链接: {url}")
    if parts[0] in ("datasets", "models"):
        t = TYPE_DATASET if parts[0] == "datasets" else TYPE_MODEL
        if len(parts) == 2:  # 命名空间列表页: .../datasets/<ns>
            return {"kind": "namespace", "type": t, "namespace": parts[1]}
        if len(parts) >= 3:  # 仓库页: .../datasets/<ns>/<name>
            return {"kind": "repo", "type": t, "namespace": parts[1],
                    "repo_id": "/".join(parts[1:3]), "name": parts[2]}
    if len(parts) == 2 and parts[1] in ("datasets", "models"):
        # 命名空间列表页: .../<ns>/datasets 或 .../<ns>/models
        t = TYPE_DATASET if parts[1] == "datasets" else TYPE_MODEL
        return {"kind": "namespace", "type": t, "namespace": parts[0]}
    if len(parts) >= 2:  # 裸仓库页: .../<ns>/<name>(类型由 --type 兜底)
        return {"kind": "repo", "type": None, "namespace": parts[0],
                "repo_id": "/".join(parts[:2]), "name": parts[1]}
    raise ValueError(f"[错误] 无法解析 HF 链接: {url}")


def _today() -> str:
    return date.today().isoformat()


def _usage_exit():
    sys.exit("""用法: python hf_manager.py <命令> [选项]
命令:
  list        查询本地 / 远程 / 收藏(附带显示简介)
  push        上传本地数据集/模型到 HF(upload 的旧名/别名; --revision/--delete/--path)
  pull        从 HF 下载数据集/模型到本地(download 的旧名/别名; --revision/--path/--jobs)
  add         收藏别人的 HF 仓库/命名空间(本地操作, 无需登录)
  delete      取消收藏(本地操作, 无需登录)
  card        查看/生成/修改数据集卡片(README.md)
  collection  管理 HF 官方集合(create / add / show / list)
  rm          删除 HF 远程仓库(delete-repo 的别名)
  visibility  切换 HF 仓库公开/私有可见性
  duplicate   服务端复制仓库到自己账号(copy 的别名)
详见 --help 或文件头部用法说明。""")


# ── 收藏夹元数据 ─────────────────────────────────────────────────────────────
def bookmarks_path(dataset_dir: Path) -> Path:
    return dataset_dir / "bookmarks.json"


def load_bookmarks(dataset_dir: Path) -> dict:
    p = bookmarks_path(dataset_dir)
    if not p.is_file():
        return {"datasets": [], "models": [], "namespaces": []}
    try:
        data = json.loads(p.read_text(encoding="utf-8"))
    except (json.JSONDecodeError, OSError) as e:
        print(f"[警告] 收藏文件损坏, 已重置: {p} ({e})")
        return {"datasets": [], "models": [], "namespaces": []}
    return {"datasets": data.get("datasets", []),
            "models": data.get("models", []),
            "namespaces": data.get("namespaces", [])}


def save_bookmarks(dataset_dir: Path, bookmarks: dict):
    """写入收藏(临时文件 + 原子替换, 避免写一半损坏收藏文件)。"""
    p = bookmarks_path(dataset_dir)
    tmp = p.with_suffix(".json.tmp")
    tmp.write_text(json.dumps(bookmarks, ensure_ascii=False, indent=2), encoding="utf-8")
    tmp.replace(p)


# ── 卡片简介 ─────────────────────────────────────────────────────────────────
def card_summary(text: str, max_len: int = 50) -> str:
    """从 README 文本中提取首行简介(去掉 # 标题 与 引用标记)。"""
    for line in text.splitlines():
        s = line.strip()
        if not s or s.startswith("#"):
            continue
        s = re.sub(r"^>+\s*", "", s)
        if s:
            return s[:max_len]
    return "(无内容)"


def strip_name(text: str, name: str) -> str:
    return text.replace(name, "").strip(" :#-") or text


def local_card_summary(d: Path) -> str:
    readme = d / "README.md"
    if readme.is_file():
        try:
            return card_summary(readme.read_text(encoding="utf-8"))
        except OSError:
            pass
    return "(无 README.md)"


def hf_token() -> str | None:
    """获取 HF 访问令牌: 优先 HF_TOKEN 环境变量, 否则读取 huggingface_hub 登录缓存。

    私有仓库的 raw 读取需要认证; 登录缓存(如 ~/.cache/huggingface/token)由
    huggingface-cli login 生成, 不设置环境变量也能取到。
    注: 不依赖 HfApi.token——新版 huggingface_hub 已移除该属性。
    """
    tok = os.environ.get("HF_TOKEN")
    if tok:
        return tok
    for p in (Path.home() / ".cache/huggingface/token",
              Path.home() / ".huggingface/token"):
        if p.is_file():
            return p.read_text(encoding="utf-8").strip()
    return None


def remote_card_text(repo_id: str, repo_type: str = TYPE_DATASET,
                     revision: str | None = None) -> str | None:
    """获取远程仓库 README.md 原文, 无权限/不存在时返回 None。

    使用官方 raw 端点 {endpoint}/{type}s/{repo_id}/resolve/{branch}/README.md,
    默认依次尝试 main / master 分支; 指定 revision 时只读取该分支/tag/commit。
    """
    import requests  # 延迟导入: 可选依赖, 仅在需要时加载
    endpoint = hf_endpoint()
    token = hf_token()
    headers = {"Authorization": f"Bearer {token}"} if token else {}
    branches = [revision] if revision else ("main", "master")
    # HF raw 端点: dataset 带 /datasets/ 前缀, model 不带 /models/ 前缀
    st = strategy_of(repo_type)
    raw_prefix = f"{st.plural}/" if st.plural == "datasets" else ""
    for branch in branches:
        url = f"{endpoint}/{raw_prefix}{repo_id}/resolve/{branch}/README.md"
        try:
            r = requests.get(url, timeout=10, headers=headers)
        except requests.RequestException:
            return None
        if r.status_code == 200:
            return r.text
    return None


def remote_card_summary(repo_id: str, repo_type: str = TYPE_DATASET) -> str:
    try:
        text = remote_card_text(repo_id, repo_type)
    except Exception:
        return "(无卡片)"
    if text is None:
        return "(无卡片)"
    return card_summary(text)


def infer_card(local_dir: Path) -> str:
    """从本地目录元数据自动生成卡片 markdown。"""
    name = local_dir.name
    files = [f for f in local_dir.rglob("*") if f.is_file()]
    total = sum(f.stat().st_size for f in files)
    lines = [DEFAULT_CARD_TEXT.strip(), "", f"## 数据说明", ""]
    lines.append(f"- 数据集名称: `{name}`")
    lines.append(f"- 文件数量: {len(files)}")
    lines.append(f"- 总大小: {human_size(total)}")
    from collections import Counter
    exts = Counter(f.suffix.lower() or "(无扩展名)" for f in files)
    if exts:
        top = exts.most_common(8)
        lines.append(f"- 常见类型: {', '.join(f'`{e}` x{n}' for e, n in top)}")
    lines.append("")
    lines.append("> 本卡片由 hf_manager `card init` 自动生成, 请人工补充:")
    lines.append("> - 数据采集方式 / 传感器说明")
    lines.append("> - 任务描述与演示场景")
    lines.append("> - 数据规模与统计口径")
    lines.append("> - 版权与授权信息")
    return "\n".join(lines)


# ── 查询 / 展示 ──────────────────────────────────────────────────────────────
def list_local(root: Path, repo_type: str = TYPE_DATASET):
    st = strategy_of(repo_type)
    print(f"[本地 {st.label}] {root}")
    items = sorted(d for d in root.iterdir() if d.is_dir() and not d.name.startswith("."))
    if not items:
        print(f"    (无{st.label})")
        return
    width = max(len(d.name) for d in items)
    col_w = max(width, 4)
    print(f"    {'NAME':<{col_w}}  {'SIZE':>10}  {'FILES':>6}")
    for d in items:
        files = [f for f in d.rglob("*") if f.is_file()]
        size = sum(f.stat().st_size for f in files)
        print(f"    {d.name:<{col_w}}  {human_size(size):>10}  {len(files):>6d}")
        print(f"        简介: {strip_name(local_card_summary(d), d.name)}")


def list_remote(api: HfApi, remote_user: str, repo_type: str = TYPE_DATASET):
    st = strategy_of(repo_type)
    print(f"[远程 {st.label}] 命名空间: {remote_user}")
    try:
        repos = st.list_remote(api, remote_user)
    except Exception as e:
        print(f"    (获取远程列表失败: {type(e).__name__}: {e})")
        print("    提示: 国内网络可先 export HF_ENDPOINT=https://hf-mirror.com 再重试")
        return
    if not repos:
        print(f"    (无{st.label})")
        return
    from concurrent.futures import ThreadPoolExecutor

    def _remote_stats(repo_id: str) -> tuple[str, str, int]:
        """并行抓取单仓库的 (简介, 文件总大小, 文件数); 失败时降级。"""
        try:
            card = remote_card_summary(repo_id, repo_type)
        except Exception:
            card = "(无卡片)"
        try:
            tree = api.list_repo_tree(repo_id, repo_type=repo_type, recursive=True)
            files = [f for f in tree if getattr(f, "size", None) is not None]
            return card, human_size(sum(f.size for f in files)), len(files)
        except Exception:
            return card, "?", 0

    metas: dict[str, tuple[str, str, int]] = {}
    with ThreadPoolExecutor(max_workers=8) as ex:
        futs = {
            (name := repo_name(r)): ex.submit(_remote_stats, f"{remote_user}/{name}")
            for r in repos
        }
        for name, f in futs.items():
            metas[name] = f.result()
    width = max(len(repo_name(r)) for r in repos)
    col_w = max(width, 4)
    print(f"    {'NAME':<{col_w}}  {'SIZE':>10}  {'FILES':>6}")
    for r in repos:
        name = repo_name(r)
        card, size_s, nfiles = metas[name]
        print(f"    {name:<{col_w}}  {size_s:>10}  {nfiles:>6d}")
        print(f"        简介: {strip_name(card, name)}")


def show_bookmarks(args, repo_type: str | None = None):
    """展示收藏夹(按类型过滤)。

    repo_type=None 显示全部类型; 指定时仅显示该类型的 datasets/models 桶
    与对应类型的 namespace 条目。namespace 条目实时枚举公开仓库(匿名只读),
    失败时降级仅显示收藏记录。
    """
    dataset_dir = Path(args.dataset_dir).resolve()
    bookmarks = load_bookmarks(dataset_dir)
    endpoint = hf_endpoint()
    print(f"[收藏] {bookmarks_path(dataset_dir)}")

    buckets = [strategy_of(t).plural for t in TYPES]
    if repo_type is not None:
        buckets = [strategy_of(repo_type).plural]
    repo_entries = [(e, b) for b in buckets for e in bookmarks.get(b, [])]

    ns_entries = bookmarks.get("namespaces", [])
    if repo_type is not None:
        ns_entries = [e for e in ns_entries
                      if e.get("type", TYPE_DATASET) == repo_type]

    if not repo_entries and not ns_entries:
        if repo_type is None:
            print("    (收藏夹为空, 用 add <URL|repo_id> 收藏感兴趣的仓库)")
        else:
            print(f"    (暂无{strategy_of(repo_type).label}收藏, 用 add <URL|repo_id> 收藏)")
        return

    if repo_entries:
        width = max(len(e.get("repo_id", "?")) for e, _ in repo_entries)
        col_w = max(width, 8)
        print(f"    {'TYPE':<8}  {'REPO_ID':<{col_w}}  {'ADDED':>10}")
        for e, bucket in repo_entries:
            print(f"    {bucket[:-1]:<8}  {e.get('repo_id', '?'):<{col_w}}  {e.get('added_at', '?'):>10}")

    for e in ns_entries:
        ns = e.get("namespace", "?")
        t = e.get("type", TYPE_DATASET)
        st = strategy_of(t)
        print(f"\n[命名空间] {ns} ({t} 列表页) 收藏于 {e.get('added_at', '?')}")
        print(f"    {endpoint}/{ns}/{st.plural}")
        try:
            repos = st.list_remote(HfApi(), ns, t)
        except Exception as ex:
            print(f"    (实时枚举公开仓库失败: {type(ex).__name__}, 仅显示收藏记录)")
            continue
        if not repos:
            print("    (该命名空间下暂无公开仓库)")
        else:
            print(f"    公开仓库 {len(repos)} 个: {', '.join(repo_name(r) for r in repos)}")


# ── 上传 / 下载核心 ──────────────────────────────────────────────────────────
def _skip_path(rel: Path) -> bool:
    """是否应跳过该相对路径(版本控制 .git* 与 HF 下载缓存 .cache, 与上传 ignore_patterns 保持一致)。"""
    return any(p.startswith((".git", ".cache")) for p in rel.parts)


def push_one(api: HfApi, repo_id: str, local_path: Path, dry_run: bool,
             repo_type: str = TYPE_DATASET, rel: str | None = None,
             description: str | None = None, no_readme: bool = False,
             sync: bool = False, prune: bool = False, revision: str | None = None,
             delete_paths: list[str] | None = None):
    """上传本地数据集/模型到 HF。

    rel 为 None 时整仓上传; rel 指定时增量上传单文件/子目录(自动创建仓库);
    sync=True 时按文件差异增量同步上传(prune=True 时删除远程多余文件);
    delete_paths 指定时只删除远程文件(可与 --path 同用: 先删后增);
    revision 指定目标分支(默认 main, 分支不存在时自动创建)。
    """
    if not dry_run:
        print(f"[仓库] 确保存在(不存在则创建): {repo_id}")
        api.create_repo(repo_id=repo_id, repo_type=repo_type, exist_ok=True)
    if sync:
        return _push_sync(api, repo_id, local_path, dry_run, repo_type, prune, revision)
    if delete_paths:
        _push_delete(api, repo_id, dry_run, repo_type, delete_paths, revision)
        if rel is not None:
            _push_incremental(api, repo_id, local_path, dry_run, repo_type, rel, revision)
        return
    if rel is not None:
        return _push_incremental(api, repo_id, local_path, dry_run, repo_type, rel, revision)
    _push_full(api, repo_id, local_path, dry_run, repo_type, description, no_readme, revision)


def _push_incremental(api, repo_id, local_path, dry_run, repo_type, rel,
                      revision: str | None = None):
    src = (local_path / rel).resolve()
    if not src.exists():
        print(f"[跳过] 本地不存在: {src}")
        return
    repo_rel = rel.replace("\\", "/").strip("/")
    branch = revision or "main"
    if dry_run:
        if src.is_dir():
            files = [f for f in src.rglob("*")
                     if f.is_file() and not _skip_path(f.relative_to(src))]
            total = sum(f.stat().st_size for f in files)
            print(f"[dry-run] 将添加目录: {repo_rel} ({len(files)} 个文件, {human_size(total)}) -> {repo_id}/{repo_rel} (分支: {branch})")
        else:
            print(f"[dry-run] 将添加文件: {repo_rel} ({human_size(src.stat().st_size)}) -> {repo_id}/{repo_rel} (分支: {branch})")
        return
    print(f"[添加] {repo_rel} -> {repo_id}/{repo_rel} (分支: {branch})")
    if src.is_dir():
        api.upload_folder(folder_path=str(src), path_in_repo=repo_rel,
                          repo_id=repo_id, repo_type=repo_type,
                          ignore_patterns=["*.git*", "*.cache*"],
                          revision=revision)
    else:
        api.upload_file(path_or_fileobj=str(src), path_in_repo=repo_rel,
                        repo_id=repo_id, repo_type=repo_type, revision=revision)
    print(f"[完成] {repo_id} 添加成功!")


def _push_full(api, repo_id, local_path, dry_run, repo_type, description, no_readme,
               revision: str | None = None):
    if description is None and not no_readme:
        readme = local_path / "README.md"
        if readme.is_file():
            description = readme.read_text(encoding="utf-8")
            print("[简介] 自动使用本地 README.md 作为卡片描述")
    branch = revision or "main"
    if dry_run:
        print(f"[dry-run] 将上传: {local_path} -> {repo_id} (分支: {branch})")
        for f in sorted(p for p in local_path.rglob("*")
                        if p.is_file() and not _skip_path(p.relative_to(local_path))):
            print(f"    {f.relative_to(local_path)}")
        if description:
            print(f"[dry-run] 将设置简介: {description[:60]}{'...' if len(description) > 60 else ''}")
        return
    print(f"[上传] {local_path} -> {repo_id} (分支: {branch}, 原样保留目录结构)")
    api.upload_folder(folder_path=str(local_path), repo_id=repo_id,
                      repo_type=repo_type, ignore_patterns=["*.git*", "*.cache*"],
                      revision=revision)
    if description:
        readme_local = local_path / "README.md"
        already = (readme_local.is_file()
                   and readme_local.read_text(encoding="utf-8") == description)
        if not already:
            print("[简介] 更新卡片描述(README.md)...")
            api.upload_file(path_or_fileobj=description.encode("utf-8"),
                            path_in_repo="README.md", repo_id=repo_id, repo_type=repo_type,
                            revision=revision)
    print(f"[完成] {repo_id} 上传成功!")


def _push_delete(api: HfApi, repo_id: str, dry_run: bool, repo_type: str,
                 paths: list[str], revision: str | None = None):
    """删除远程仓库中的指定文件/子目录(可多个)。"""
    branch = revision or "main"
    if dry_run:
        for rel in paths:
            rel = rel.replace("\\", "/").strip("/")
            print(f"[dry-run] 将删除远程文件: {repo_id}/{rel} (分支: {branch})")
        return
    for rel in paths:
        rel = rel.replace("\\", "/").strip("/")
        print(f"[删除] {repo_id}/{rel} (分支: {branch})")
        api.delete_file(path_in_repo=rel, repo_id=repo_id,
                        repo_type=repo_type, revision=revision)
    print(f"[完成] {repo_id} 删除成功!")


def _push_sync(api: HfApi, repo_id: str, local_path: Path, dry_run: bool,
               repo_type: str, prune: bool = False, revision: str | None = None):
    """增量同步上传: 只推送本地新增/变化的文件, prune=True 时删除远程多余文件。

    按文件大小比对(get_paths_info), 大改动按 200 个操作分片原子提交,
    避免单次操作数过多导致失败。README.md 作为普通文件自动纳入同步。
    revision 指定目标分支(默认 main); create_commit 使用 revision 参数。
    """
    branch = revision or "main"
    remote_files = set(api.list_repo_files(repo_id, repo_type=repo_type, revision=revision))
    local_files = {f.relative_to(local_path).as_posix(): f
                   for f in local_path.rglob("*")
                   if f.is_file() and not _skip_path(f.relative_to(local_path))}
    # 待删除的远程文件(仅 --prune; README.md 视为卡片, .gitattributes 等 HF 元文件不删)
    deletions = (remote_files - set(local_files)
                 - {".gitattributes", "README.md"}) if prune else set()
    # 待上传的本地文件(新增或远程大小不一致)
    size_of: dict[str, int] = {}
    common = remote_files & set(local_files)
    if common:
        try:
            infos = api.get_paths_info(repo_id=repo_id, repo_type=repo_type,
                                       paths=sorted(common), revision=revision)
            size_of = {info.path: getattr(info, "size", -1) for info in infos}
        except Exception:  # noqa: BLE001  (拿不到远程大小时退化为"全部覆盖")
            pass
    uploads = [(rel, f) for rel, f in local_files.items()
               if rel not in remote_files or size_of.get(rel) != f.stat().st_size]

    if not uploads and not deletions:
        print(f"[同步] {repo_id} 已是最新, 无需变更")
        return
    if dry_run:
        for rel, f in uploads:
            print(f"[dry-run] 将上传: {rel} ({human_size(f.stat().st_size)}) -> 分支 {branch}")
        for rel in sorted(deletions):
            print(f"[dry-run] 将删除: {rel}")
        print(f"[dry-run] 合计: 上传 {len(uploads)} 个, 删除 {len(deletions)} 个")
        return

    print(f"[同步] {repo_id} (分支: {branch}): 上传 {len(uploads)} 个, 删除 {len(deletions)} 个")
    ops: list = []

    def flush(ops: list):
        if ops:
            api.create_commit(repo_id=repo_id, repo_type=repo_type,
                              operations=ops, commit_message="hf_manager sync",
                              revision=revision)
            ops.clear()

    for rel, f in uploads:
        ops.append(CommitOperationAdd(path_or_fileobj=str(f), path_in_repo=rel))
        if len(ops) >= 200:
            flush(ops)
    for rel in sorted(deletions):
        ops.append(CommitOperationDelete(path_in_repo=rel))
        if len(ops) >= 200:
            flush(ops)
    flush(ops)
    print(f"[完成] {repo_id} 同步完成!")


def _pull_path(api: HfApi, repo_id: str, local_path: Path, dry_run: bool,
               repo_type: str, rels: list[str], show_card: bool = False,
               revision: str | None = None):
    """定向下载: 只拉取远程仓库中的指定文件/子目录(可多个), 保留相对目录结构。"""
    if show_card:
        text = remote_card_text(repo_id, repo_type, revision=revision)
        print(f"[简介] {repo_id}: {text if text is not None else '(无 README.md)'}")
    for rel in rels:
        _pull_one_path(api, repo_id, local_path, dry_run, repo_type, rel, revision)
    print(f"[完成] {local_path} 定向下载成功!")


def _pull_one_path(api: HfApi, repo_id: str, local_path: Path, dry_run: bool,
                   repo_type: str, rel: str, revision: str | None = None):
    """定向下载单个文件/子目录, 保留相对目录结构。"""
    rel = rel.replace("\\", "/").strip("/")
    branch = revision or "main"
    if dry_run:
        print(f"[dry-run] 将下载: {repo_id}/{rel} (分支: {branch}) -> {local_path / rel}")
        return
    # 先判断目标是文件还是目录(get_paths_info 的 type 字段)
    target_type = None
    try:
        infos = api.get_paths_info(repo_id=repo_id, repo_type=repo_type, paths=[rel],
                                   revision=revision)
        if infos:
            target_type = getattr(infos[0], "type", None)
    except Exception:  # noqa: BLE001  (探测失败时按文件处理, 下载报错会给出提示)
        pass
    if target_type == "directory":
        print(f"[下载] {repo_id}/{rel} (目录, 分支: {branch}) -> {local_path / rel}")
        snapshot_download(repo_id=repo_id, repo_type=repo_type,
                          local_dir=str(local_path), allow_patterns=[f"{rel}/**"],
                          ignore_patterns=[".git/*"], revision=revision)
    else:
        print(f"[下载] {repo_id}/{rel} (文件, 分支: {branch}) -> {local_path / rel}")
        (local_path / rel).parent.mkdir(parents=True, exist_ok=True)
        hf_hub_download(repo_id=repo_id, repo_type=repo_type, filename=rel,
                        local_dir=str(local_path), revision=revision)


def download_one(repo_id: str, local_path: Path, dry_run: bool,
                 show_card: bool = False, repo_type: str = TYPE_DATASET,
                 revision: str | None = None):
    """下载远程仓库到本地(目录结构原样保留)。"""
    branch = revision or "main"
    if show_card:
        text = remote_card_text(repo_id, repo_type, revision=revision)
        if text is None:
            print(f"[简介] {repo_id} 无 README.md")
        else:
            print(f"[简介] {repo_id}:\n{text}")
    if dry_run:
        print(f"[dry-run] 将下载: {repo_id} (分支: {branch}) -> {local_path}")
        return
    print(f"[下载] {repo_id} (分支: {branch}) -> {local_path}")
    snapshot_download(repo_id=repo_id, repo_type=repo_type,
                      local_dir=str(local_path), allow_patterns=["*"],
                      ignore_patterns=[".git/*"], revision=revision)
    print(f"[完成] {local_path} 下载成功!")


def resolve_pull_target(api: HfApi, remote_user: str | None, target: str, args) -> tuple[str, str, str]:
    """把 pull 目标解析为 (repo_id, 本地目录名, 仓库类型)。

    解析顺序: URL → 完整 repo_id(含 /, 任意公开仓库) → 收藏命中(短名/repo_id) → {账号}/{名};
    最后用 detect_repo_type 自动探测实际仓库类型, 避免 --type 默认 dataset
    导致模型仓库被按数据集解析而下载失败。
    """
    dataset_dir = Path(args.dataset_dir).resolve()
    if looks_like_url(target):
        parsed = parse_hf_url(target)
        if parsed["kind"] == "namespace":
            raise ValueError(
                f"不能直接下载命名空间页: {target}\n"
                "  可先 add 收藏该命名空间, 再用 pull 逐仓库下载"
            )
        repo_id, name = parsed["repo_id"], parsed["name"]
        rtype = parsed["type"] or args.repo_type
    elif "/" in target:
        repo_id, name = target, target.split("/")[-1]
        rtype = args.repo_type
    else:
        bookmarks = load_bookmarks(dataset_dir)
        hits = []
        for t in TYPES:
            bucket = strategy_of(t).plural
            for e in bookmarks.get(bucket, []):
                rid = e.get("repo_id", "")
                if rid == target or rid.split("/")[-1] == target:
                    hits.append((rid, rid.split("/")[-1], t))
        if hits:
            if len(hits) > 1:
                ids = ", ".join(h[0] for h in hits)
                raise ValueError(
                    f"收藏短名 {target} 命中多个仓库({ids}), 请改用完整 repo_id 指定")
            return hits[0]
        if not remote_user:
            raise ValueError(
                f"无法确定远程仓库: {target}\n"
                "  请用完整 repo_id(<账号>/<名字>) 或 URL, 或先登录/--remote-user"
            )
        repo_id, name = f"{remote_user}/{target}", target
        rtype = args.repo_type
    # 自动探测实际仓库类型(dataset/model), 探测失败则保留原类型
    detected = detect_repo_type(api, repo_id)
    if detected:
        rtype = detected
    return repo_id, name, rtype


# ── 命令模式: 每条子命令封装为对象 ──────────────────────────────────────────
class Command(ABC):
    """命令模式: 子命令对象, 自包含参数定义/校验/执行, 便于扩展与复用。"""

    name = ""      # 子命令名
    help = ""      # 帮助文案

    @abstractmethod
    def add_arguments(self, p: argparse.ArgumentParser) -> None:
        """向 argparse 注册本命令专属参数。"""

    def validate(self, _args) -> None:
        """参数校验(如互斥检查); 默认无。"""

    @abstractmethod
    def run(self, args, ctx: Context) -> None:
        """执行命令。"""


class TransferCommand(Command):
    """push/pull 公共基类: 共享转移类参数定义与 --all 互斥校验。"""

    def add_transfer_args(self, p, names_help, all_help):
        p.add_argument("datasets", nargs="*", help=names_help)
        p.add_argument("--all", action="store_true", help=all_help)
        p.add_argument("--dry-run", action="store_true", help="只打印计划, 不真正执行")

    def validate(self, args):
        if args.all and args.datasets:
            sys.exit("[错误] --all 与显式名称互斥, 请只指定一个: --all 或名称列表")


class ListCommand(Command):
    name = "list"
    help = "查询本地 / 远程 / 收藏(附带显示简介)"

    def add_arguments(self, p):
        p.add_argument("type_word", nargs="?", choices=TYPES, metavar="{model,dataset}",
                       help="仓库类型简写, 等价 --type(如 `list model`)")
        p.add_argument("--local", action="store_true", help="只列出本地目录")
        p.add_argument("--remote", action="store_true", help="只列出 HF 账号下的远程仓库")
        p.add_argument("--bookmarks", action="store_true", help="只展示收藏夹")

    def run(self, args, ctx):
        if args.type_word:
            args.repo_type = args.type_word  # `list model` 简写 -> --type model
        # 视图选择: 显式指定任一 flag 则只显示所选; 否则默认全部显示(本地+远程+收藏)
        if args.local or args.remote or args.bookmarks:
            show_local, show_remote, show_bm = args.local, args.remote, args.bookmarks
        else:
            show_local = show_remote = show_bm = True
        if show_local:
            list_local(ctx.local_root(), args.repo_type)
        if show_remote:
            if show_local:
                print()
            api, _ = ctx.login(required=False)
            remote_user = ctx.remote_user
            if api is None and remote_user:
                print("[远程] 未登录, 以匿名方式列出公开仓库...")
                list_remote(HfApi(), remote_user, args.repo_type)
            elif api is None:
                print("[远程] 未登录或网络不可用, 跳过远程查询, 请按以下指引处理后重试:")
                print("    # 1) 安装依赖(首次使用)")
                print("    pip install huggingface_hub")
                print("    # 2) 登录(初次使用必须登录, 即使已设置镜像也需账号)")
                print("    export HF_TOKEN=hf_xxx              # 方式A: 设置 token 环境变量")
                print("    hf auth login                       # 方式B: 交互式登录(token 存缓存目录)")
                print("    # 3) 国内网络建议走镜像")
                print("    export HF_ENDPOINT=https://hf-mirror.com")
                print("    # 4) 检查网络连通性")
                print("    curl -I https://huggingface.co      # 可访问则返回 HTTP 200")
            else:
                list_remote(api, remote_user, args.repo_type)
        if show_bm:
            if show_local or show_remote:
                print()
            show_bookmarks(args, args.repo_type)


class PushCommand(TransferCommand):
    name = "push"
    help = "上传本地数据集/模型到 HF(仅支持自己的命名空间, 不支持 org; upload 的旧名/别名)"

    def add_arguments(self, p):
        self.add_transfer_args(p,
                               "本地目录名(数据集/模型根目录下的子目录, 可多个)",
                               "上传根目录下所有子目录")
        p.add_argument("--path", help="只上传本地相对路径(单文件/子目录), 增量添加")
        p.add_argument("--sync", action="store_true",
                       help="增量同步上传(仅推送本地新增/变化的文件)")
        p.add_argument("--prune", action="store_true",
                       help="与 --sync 搭配: 同时删除远程多余的文件(危险, 慎用)")
        p.add_argument("--description", help="上传后设置的卡片简介文本")
        p.add_argument("--description-file", help="从文件读取卡片简介")
        p.add_argument("--no-readme", action="store_true",
                       help="上传时不自动携带本地 README.md 作为卡片简介")
        p.add_argument("--revision", help="上传到指定分支(默认 main, 分支不存在时自动创建)")
        p.add_argument("--delete", dest="delete_paths", nargs="+",
                       help="删除远程仓库中的文件/子目录(可多个; 可与 --path 同用: 先删后增)")

    def validate(self, args):
        super().validate(args)
        if args.path and args.all:
            sys.exit("[错误] --path 与 --all 互斥, 请只指定一个")
        if args.path and len(args.datasets) > 1:
            sys.exit("[错误] --path 只能指定单个本地目录, 请一次只给一个名称")
        if args.sync and args.all:
            sys.exit("[错误] --sync 与 --all 互斥, 请只指定一个")
        if args.sync and args.path:
            sys.exit("[错误] --sync 与 --path 互斥(同步是整仓差异, --path 是定点添加)")
        if args.prune and not args.sync:
            sys.exit("[错误] --prune 仅在与 --sync 搭配时使用")
        if args.delete_paths and args.sync:
            sys.exit("[错误] --delete 与 --sync 互斥(显式删除用 --delete, 自动差量删除用 --sync --prune)")
        if args.delete_paths and args.all:
            sys.exit("[错误] --delete 与 --all 互斥, 请指定本地目录名")

    def run(self, args, ctx):
        api, username = ctx.login(required=True)
        if args.remote_user and args.remote_user != username:
            sys.exit(f"[错误] push 只能上传到自己的命名空间, 不能指定他人账号 {args.remote_user}\n"
                     "  如需下载他人/公开仓库, 请用 pull <repo_id 或 URL>")
        if args.all:
            names = [d.name for d in ctx.local_root().iterdir()
                     if d.is_dir() and not d.name.startswith(".")]
        else:
            names = args.datasets
        if not names:
            sys.exit("[提示] 请指定本地目录名称, 或用 --all")

        description = None
        if args.description:
            description = args.description
        elif args.description_file:
            try:
                description = Path(args.description_file).read_text(encoding="utf-8")
            except Exception as e:
                sys.exit(f"[错误] 读取简介文件失败: {e}")

        root = ctx.local_root()
        fails = 0
        for name in names:
            local = root / name
            if not local.is_dir():
                print(f"[跳过] 本地不存在: {name} (根目录: {root})")
                fails += 1
                continue
            repo_id = f"{username}/{name}"
            try:
                push_one(api, repo_id, local, args.dry_run, repo_type=args.repo_type,
                         rel=args.path, description=description, no_readme=args.no_readme,
                         sync=args.sync, prune=args.prune, revision=args.revision,
                         delete_paths=args.delete_paths)
            except Exception as e:
                print(f"[跳过] 上传失败 {repo_id}: {e}")
                fails += 1
        if fails:
            sys.exit(fails)


class PullCommand(TransferCommand):
    name = "pull"
    help = "从 HF 下载数据集/模型到本地(download 的旧名/别名)"

    def add_arguments(self, p):
        self.add_transfer_args(p,
                               "仓库名 / repo_id(<账号>/<名字>) / URL / 收藏短名",
                               "下载账号下所有远程仓库")
        p.add_argument("--path", nargs="+",
                       help="只下载仓库中指定文件/子目录(可多个, 如 data/chunk-000 meta/info.json)")
        p.add_argument("--revision",
                       help="下载指定分支/tag/commit(默认 main; 用于复现历史版本实验)")
        p.add_argument("--jobs", type=int, default=1,
                       help="并发下载仓库数(默认 1=顺序下载; 如 --jobs 4 并行下载多个仓库)")
        p.add_argument("--show-card", action="store_true", help="下载前显示卡片简介")

    def validate(self, args):
        super().validate(args)
        if args.path and args.all:
            sys.exit("[错误] --path 与 --all 互斥, 请只指定一个")
        if args.path and len(args.datasets) > 1:
            sys.exit("[错误] --path 只能指定单个仓库, 请一次只给一个目标")

    def run(self, args, ctx):
        # 非 --all 也需要登录账号, 用于把短名解析为 {账号}/{名字}(未登录时降级匿名只读)
        api, username = ctx.login(required=False)
        if args.all:
            names = [repo_name(r) for r in strategy_of(args.repo_type).list_remote(api, username)]
        else:
            names = args.datasets
        if not names:
            sys.exit("[提示] 请指定仓库名(repo_id / URL / 收藏名), 或用 --all")
        if api is None:
            api = HfApi()  # 未登录: 匿名只读实例, 可下载公开仓库

        def _one(target: str) -> int:
            """下载单个仓库, 返回失败计数(0 或 1); 并发时各线程独立调用。"""
            try:
                repo_id, name, rtype = resolve_pull_target(api, username, target, args)
            except Exception as e:
                print(f"[跳过] 无法解析目标 {target}: {e}")
                return 1
            local = ctx.local_root(rtype) / name
            try:
                if args.path:
                    _pull_path(api, repo_id, local, args.dry_run, rtype,
                               args.path, show_card=args.show_card, revision=args.revision)
                else:
                    if local.exists() and any(local.iterdir()):
                        print(f"[提示] 本地目录已存在且非空, 将合并下载: {local}")
                    download_one(repo_id, local, args.dry_run,
                                 show_card=args.show_card, repo_type=rtype,
                                 revision=args.revision)
                return 0
            except Exception as e:
                print(f"[跳过] 下载失败 {repo_id}: {e}")
                return 1

        jobs = max(1, args.jobs)
        if jobs > 1 and len(names) > 1:
            from concurrent.futures import ThreadPoolExecutor, as_completed
            fails = 0
            with ThreadPoolExecutor(max_workers=jobs) as ex:
                for fut in as_completed([ex.submit(_one, t) for t in names]):
                    fails += fut.result()
        else:
            fails = sum(_one(t) for t in names)
        if fails:
            sys.exit(fails)


class AddCommand(Command):
    name = "add"
    help = "收藏别人的 HF 仓库/命名空间(本地操作, 无需登录)"

    def add_arguments(self, p):
        p.add_argument("target", nargs="?", help="HF 链接或完整 repo_id(<账号>/<名字>)")
        p.add_argument("--dry-run", action="store_true", help="只打印计划, 不写入收藏")

    def run(self, args, _ctx):
        dataset_dir = Path(args.dataset_dir).resolve()
        bookmarks = load_bookmarks(dataset_dir)
        target = args.target
        if not target:
            sys.exit("[提示] 用法: add <URL|repo_id> [--type dataset|model] [--dry-run]\n"
                     "  示例:\n"
                     "    add https://huggingface.co/qingxiangliu/datasets\n"
                     "    add https://huggingface.co/datasets/qingxiangliu/walker_pick_sort\n"
                     "    add qingxiangliu/walker_pick_sort")

        entry = None
        bucket = None
        if looks_like_url(target):
            try:
                parsed = parse_hf_url(target)
            except ValueError as e:
                sys.exit(f"[错误] {e}")
            ptype = parsed["type"] or args.repo_type  # 裸 <ns>/<name> 链接按 --type 兜底
            if parsed["kind"] == "namespace":
                entry = {"kind": "namespace", "type": ptype,
                         "namespace": parsed["namespace"], "added_at": _today()}
                bucket = "namespaces"
            else:
                entry = {"kind": "repo", "type": ptype,
                         "repo_id": parsed["repo_id"], "added_at": _today()}
                bucket = strategy_of(ptype).plural
        elif "/" in target:
            entry = {"kind": "repo", "type": args.repo_type,
                     "repo_id": target, "added_at": _today()}
            bucket = strategy_of(args.repo_type).plural
        else:
            ds = Path(args.dataset_dir).resolve() / target
            md = Path(args.model_dir).resolve() / target
            if ds.is_dir() or md.is_dir():
                sys.exit(f"[提示] add 现在是收藏命令, 上传本地目录请用 push {target}")
            sys.exit(f"[错误] 无法识别的收藏目标: {target}\n"
                     "  收藏需要 HF 链接或完整 repo_id(<账号>/<名字>)\n"
                     "  示例: add https://huggingface.co/datasets/qingxiangliu/walker_pick_sort")

        if bucket == "namespaces":
            dup = any(e.get("namespace") == entry["namespace"] and e.get("type") == entry["type"]
                      for e in bookmarks["namespaces"])
            desc = f"命名空间 {entry['namespace']} ({entry['type']} 列表页)"
            ref = f"https://huggingface.co/{entry['namespace']}/{entry['type']}s"
        else:
            dup = any(e.get("repo_id") == entry["repo_id"] for e in bookmarks[bucket])
            desc = f"{entry['type']} {entry['repo_id']}"
            ref = entry["repo_id"]
        if dup:
            print(f"[收藏] 已存在, 无需重复: {desc}")
            return
        if args.dry_run:
            print(f"[dry-run] 将收藏: {desc}")
            return
        bookmarks[bucket].append(entry)
        save_bookmarks(dataset_dir, bookmarks)
        print(f"[完成] 已收藏: {desc}")
        print(f"    存储: {bookmarks_path(dataset_dir)}")
        print(f"    查看: list --bookmarks    | 下载: pull {ref}")


class DeleteCommand(Command):
    name = "delete"
    help = "取消收藏(本地操作, 无需登录)"

    def add_arguments(self, p):
        p.add_argument("target", nargs="?", help="repo_id / 短名 / URL")
        p.add_argument("--dry-run", action="store_true", help="只打印计划, 不真正删除")

    def run(self, args, _ctx):
        dataset_dir = Path(args.dataset_dir).resolve()
        bookmarks = load_bookmarks(dataset_dir)
        target = args.target
        if not target:
            sys.exit("[提示] 用法: delete <repo_id|短名|URL> [--dry-run]")

        buckets = [strategy_of(t).plural for t in TYPES]
        hits = []
        if looks_like_url(target):
            try:
                parsed = parse_hf_url(target)
            except ValueError as e:
                sys.exit(f"[错误] {e}")
            ptype = parsed["type"] or args.repo_type  # 裸 <ns>/<name> 链接按 --type 兜底
            if parsed["kind"] == "namespace":
                hits = [(e, "namespaces") for e in bookmarks["namespaces"]
                        if e.get("namespace") == parsed["namespace"]
                        and e.get("type") == ptype]
            else:
                bucket = strategy_of(ptype).plural
                hits = [(e, bucket) for e in bookmarks.get(bucket, [])
                        if e.get("repo_id") == parsed["repo_id"]]
        elif "/" in target:
            hits = [(e, b) for b in buckets
                    for e in bookmarks.get(b, []) if e.get("repo_id") == target]
        else:
            hits = [(e, b) for b in buckets for e in bookmarks.get(b, [])
                    if e.get("repo_id") == target or e.get("repo_id", "").split("/")[-1] == target]
            if len(hits) > 1:
                print(f"[提示] 短名 {target} 命中 {len(hits)} 个收藏, 请用完整 repo_id 精确删除:")
                for e, b in hits:
                    print(f"    delete {e['repo_id']}   # {b[:-1]}")
                return

        if not hits:
            sys.exit(f"[提示] 收藏夹中不存在: {target}\n  可用 list --bookmarks 查看全部收藏")

        if args.dry_run:
            for e, b in hits:
                print(f"[dry-run] 将取消收藏: {b[:-1]} {e.get('repo_id') or e.get('namespace')}")
            return
        for e, b in hits:
            bookmarks[b].remove(e)
            print(f"[完成] 已取消收藏: {b[:-1]} {e.get('repo_id') or e.get('namespace')}")
        save_bookmarks(dataset_dir, bookmarks)


class CardCommand(Command):
    name = "card"
    help = "查看/生成/修改数据集卡片(README.md)"

    def add_arguments(self, p):
        sub = p.add_subparsers(dest="action", required=True, metavar="{init,get,set}")

        pci = sub.add_parser("init", help="从本地元数据自动生成简介(写 README.md, 本地操作)")
        add_common_args(pci)
        pci.add_argument("datasets", nargs="*", help="数据集/模型目录名")
        pci.add_argument("--all", action="store_true", help="为根目录下所有子目录生成简介")
        pci.add_argument("--dry-run", action="store_true", help="只打印计划, 不真正写文件")

        pcg = sub.add_parser("get", help="查看卡片完整内容")
        add_common_args(pcg)
        pcg.add_argument("datasets", nargs="*", help="数据集/模型名")
        pcg.add_argument("--all", action="store_true", help="查看账号下所有远程仓库的卡片")

        pcs = sub.add_parser("set", help="修改卡片内容")
        add_common_args(pcs)
        pcs.add_argument("datasets", nargs="*", help="数据集/模型名")
        pcs.add_argument("--text", help="卡片内容文本")
        pcs.add_argument("--file", help="从文件读取卡片内容")
        pcs.add_argument("--all", action="store_true", help="为账号下所有远程仓库设置相同卡片")
        pcs.add_argument("--dry-run", action="store_true", help="只打印计划, 不真正修改")

    def run(self, args, ctx):
        if args.action == "init":
            self._run_init(args, ctx)
        elif args.action == "set":
            self._run_set(args, ctx)
        else:
            self._run_get(args, ctx)

    def _resolve_names(self, args, ctx, *, remote: bool) -> list[str]:
        """解析 --all 或显式名称; remote=True 时 --all 需登录列出远程仓库。"""
        if args.all and args.datasets:
            sys.exit("[错误] --all 与显式名称互斥, 请只指定一个")
        if args.all:
            if remote:
                api, user = ctx.login(required=True)
                return [repo_name(r) for r in strategy_of(args.repo_type).list_remote(api, user)]
            return [d.name for d in ctx.local_root().iterdir()
                    if d.is_dir() and not d.name.startswith(".")]
        return args.datasets

    def _run_init(self, args, ctx):
        names = self._resolve_names(args, ctx, remote=False)
        if not names:
            sys.exit("[提示] card init 需指定数据集名称, 或用 --all")
        for name in names:
            local = ctx.local_root() / name
            if not local.is_dir():
                print(f"[跳过] 本地不存在: {name}")
                continue
            text = infer_card(local)
            if args.dry_run:
                print(f"[dry-run] 将生成 {local}/README.md:\n{text}\n")
                continue
            (local / "README.md").write_text(text, encoding="utf-8")
            print(f"[完成] 已生成简介: {local}/README.md")

    def _run_get(self, args, ctx):
        if args.all:
            names = self._resolve_names(args, ctx, remote=True)
        else:
            names = args.datasets
        if not names:
            sys.exit("[提示] 请指定数据集名称, 或用 --all")
        ctx.login(required=True)  # 先解析登录账号, 与 card set 一致
        user = ctx.remote_user
        if user is None:
            sys.exit("[错误] card get 需要 --remote-user 或登录, 以确定仓库命名空间")
        for name in names:
            repo_id = f"{user}/{name}"
            try:
                text = remote_card_text(repo_id, args.repo_type)
            except Exception as e:
                print(f"[跳过] 获取卡片失败 {repo_id}: {e}")
                continue
            if text is None:
                print(f"[跳过] 获取卡片失败(仓库可能无 README.md 或未授权): {repo_id}")
                continue
            print(f"===== {repo_id} 卡片 =====")
            print(text)

    def _run_set(self, args, ctx):
        names = self._resolve_names(args, ctx, remote=True)
        if not names:
            sys.exit("[提示] 请指定数据集名称, 或用 --all")
        api, user = ctx.login(required=True)
        if args.text is not None:
            desc = args.text
        elif args.file:
            try:
                desc = Path(args.file).read_text(encoding="utf-8")
            except Exception as e:
                sys.exit(f"[错误] 读取卡片文件失败: {e}")
        else:
            sys.exit("[提示] card set 需 --text 或 --file")
        if args.dry_run:
            print(f"[dry-run] 将更新卡片 {', '.join(names)}:\n{desc}")
            return
        for name in names:
            repo_id = f"{user}/{name}"
            try:
                api.upload_file(path_or_fileobj=desc.encode("utf-8"),
                                path_in_repo="README.md", repo_id=repo_id, repo_type=args.repo_type)
                print(f"[完成] 已更新卡片: {repo_id}")
            except Exception as e:
                print(f"[跳过] 更新卡片失败 {repo_id}: {e}")


class CollectionCommand(Command):
    name = "collection"
    help = "管理 HF 官方集合(create / add / show / list)"

    def add_arguments(self, p):
        sub = p.add_subparsers(dest="action", required=True,
                               metavar="{create,add,show,list}")

        pcc = sub.add_parser("create", help="创建集合")
        add_common_args(pcc)
        pcc.add_argument("title", help="集合标题")
        pcc.add_argument("--description", help="集合简介")
        pcc.add_argument("--private", action="store_true", help="创建为私有集合")
        pcc.add_argument("--dry-run", action="store_true", help="只打印计划, 不真正创建")

        pca = sub.add_parser("add", help="向集合添加仓库")
        add_common_args(pca)
        pca.add_argument("slug", help="集合 slug(如 qingxiangliu/il-datasets)")
        pca.add_argument("items", nargs="+", help="repo_id 或 HF 链接(可多个)")
        pca.add_argument("--create", action="store_true",
                         help="slug 不存在时自动创建(标题取 slug 末段)")
        pca.add_argument("--dry-run", action="store_true", help="只打印计划, 不真正添加")

        pcs = sub.add_parser("show", help="查看集合内容")
        add_common_args(pcs)
        pcs.add_argument("slug", help="集合 slug")

        pcl = sub.add_parser("list", help="列出我的集合")
        add_common_args(pcl)

    def _resolve_slug(self, username: str, slug: str) -> str:
        """把 slug 规范化为 <ns>/<name>; 允许裸短名(用登录账号补齐)。"""
        slug = slug.replace("https://huggingface.co/collections/", "").strip("/")
        if "/" not in slug:
            slug = f"{username}/{slug}"
        return slug

    def run(self, args, ctx):
        api, username = ctx.login(required=True)
        if args.action == "create":
            self._create(args, api)
        elif args.action == "add":
            self._add(args, api, username)
        elif args.action == "show":
            self._show(args, api)
        else:
            self._list(api, username)

    def _create(self, args, api):
        if args.dry_run:
            print(f"[dry-run] 将创建集合: {args.title} ({'私有' if args.private else '公开'})")
            return
        slug = api.create_collection(title=args.title, description=args.description or "",
                                     private=args.private)
        print(f"[完成] 集合已创建: {slug}")

    def _add(self, args, api, username):
        slug = self._resolve_slug(username, args.slug)
        if args.create:
            # --create: 先确保集合存在(幂等, 已存在则忽略)
            try:
                api.create_collection(title=slug.split("/")[-1],
                                      namespace=slug.split("/")[0])
            except Exception:  # noqa: BLE001  (已存在/无权限均继续)
                pass
        for item in args.items:
            repo_type = args.repo_type
            repo_id = item
            if looks_like_url(item):
                parsed = parse_hf_url(item)
                repo_type = parsed["type"] or repo_type
                repo_id = parsed["repo_id"]
            if args.dry_run:
                print(f"[dry-run] 将添加 {repo_type} {repo_id} -> {slug}")
                continue
            api.add_collection_item(slug, item_type=repo_type, item_id=repo_id)
            print(f"[完成] 已添加 {repo_type} {repo_id} -> {slug}")

    def _show(self, args, api):
        col = api.get_collection(args.slug)
        vis = "私有" if col.private else "公开"
        print(f"[集合] {col.title} (slug: {col.slug}, {vis}, {len(col.items)} 项)")
        for it in col.items:
            print(f"    - {it.item_type} {it.item_id}")

    def _list(self, api, username):
        cols = list(api.list_collections(owner=username))
        if not cols:
            print("(无集合, 可用 collection create <标题> 创建)")
            return
        for c in cols:
            print(f"    - {c.slug}: {c.title}")


class RmCommand(Command):
    name = "rm"
    help = "删除 HF 远程仓库(危险操作, 需确认; delete-repo 的别名)"

    def add_arguments(self, p):
        p.add_argument("repo_id", help="远程仓库 repo_id(<账号>/<名字>)")
        p.add_argument("--yes", action="store_true", help="跳过确认提示")
        p.add_argument("--dry-run", action="store_true", help="只打印计划, 不真正删除")

    def run(self, args, ctx):
        api, _ = ctx.login(required=True)
        repo_type = detect_repo_type(api, args.repo_id)
        if repo_type is None:
            sys.exit(f"[错误] 仓库不存在或无权限: {args.repo_id}")
        if args.dry_run:
            print(f"[dry-run] 将删除 {repo_type}: {args.repo_id}")
            return
        if not args.yes:
            try:
                ans = input(f"确认删除 {repo_type} {args.repo_id}? (输入完整 repo_id 确认): ")
            except EOFError:
                sys.exit("[取消] 无输入(EOF), 已取消删除")
            if ans.strip() != args.repo_id:
                sys.exit("[取消] 输入不一致, 已取消删除")
        api.delete_repo(repo_id=args.repo_id, repo_type=repo_type)
        print(f"[完成] 已删除 {repo_type}: {args.repo_id}")


class VisibilityCommand(Command):
    name = "visibility"
    help = "切换 HF 仓库公开/私有可见性"

    def add_arguments(self, p):
        p.add_argument("repo_id", help="远程仓库 repo_id(<账号>/<名字>)")
        g = p.add_mutually_exclusive_group(required=True)
        g.add_argument("--private", action="store_true", help="设为私有")
        g.add_argument("--public", action="store_true", help="设为公开")
        p.add_argument("--dry-run", action="store_true", help="只打印计划, 不真正切换")

    def run(self, args, ctx):
        api, _ = ctx.login(required=True)
        repo_type = detect_repo_type(api, args.repo_id)
        if repo_type is None:
            sys.exit(f"[错误] 仓库不存在或无权限: {args.repo_id}")
        private = bool(args.private)
        state = "私有" if private else "公开"
        if args.dry_run:
            print(f"[dry-run] 将把 {repo_type} {args.repo_id} 设为{state}")
            return
        api.update_repo_visibility(repo_id=args.repo_id, repo_type=repo_type, private=private)
        print(f"[完成] {repo_type} {args.repo_id} 已设为{state}")


class DuplicateCommand(Command):
    name = "duplicate"
    help = "服务端复制 HF 仓库到自己账号(官方 duplicate_repo, 无需本地中转; copy 的别名)"

    def add_arguments(self, p):
        p.add_argument("target", help="要复制的仓库: 收藏名 / repo_id(<账号>/<名字>) / HF 链接")
        p.add_argument("--to", help="目标 repo_id(<账号>/<名字>); 缺省 = 同名复制到登录账号")
        p.add_argument("--private", action="store_true",
                       help="新仓库设为私有(缺省跟随源仓库可见性)")
        p.add_argument("--dry-run", action="store_true", help="只打印计划, 不真正复制")

    def run(self, args, ctx):
        api, username = ctx.login(required=True)
        try:
            repo_id, name, repo_type = resolve_pull_target(api, username, args.target, args)
        except Exception as e:
            sys.exit(f"[错误] 无法解析复制目标 {args.target}: {e}")
        to_id = args.to
        if to_id and "/" not in to_id:
            to_id = f"{username}/{to_id}"
        target = to_id or f"{username}/{name}"
        vis = "私有" if args.private else "跟随源仓库"
        if args.dry_run:
            print(f"[dry-run] 将复制 {repo_type} {repo_id} -> {target} ({vis})")
            return
        print(f"[复制] {repo_type} {repo_id} -> {target} ...")
        url = duplicate_repo(from_id=repo_id, to_id=target,
                             repo_type=repo_type, private=args.private or None)
        print(f"[完成] 复制成功: {url} ({vis})")


COMMANDS: dict[str, Command] = {
    c.name: c for c in (
        ListCommand(), PushCommand(), PullCommand(),
        AddCommand(), DeleteCommand(), CardCommand(),
        CollectionCommand(), RmCommand(), VisibilityCommand(),
        DuplicateCommand(),
    )
}
ALIASES = {"upload": "push", "download": "pull", "delete-repo": "rm", "copy": "duplicate"}


# ── 入口 ─────────────────────────────────────────────────────────────────────
def _scan(argv: list[str], flag: str, default: str) -> str:
    if flag in argv:
        i = argv.index(flag)
        if i + 1 < len(argv):
            return argv[i + 1]
    return default


def _normalize_argv(argv: list[str]) -> list[str]:
    """兼容旧脚本参数风格: 省略子命令时自动推断。

    - 首参已显式给出子命令(含旧名别名)时直接透传, 不做旧式转换;
    - `--<命令>` 旧写法 -> 子命令(如 `--list model` -> `list model`, `--add <url>` -> `add <url>`);
    - `upload/download` 旧名别名; URL 首参自动收藏; 本地目录名自动 push。
    """
    argv = list(argv)
    first = next((a for a in argv if not a.startswith("-")), None)

    # 首参已是子命令(含旧名别名): 直接透传, 不做旧式 --<cmd> 转换
    # (避免 `push <name> --delete <path>` 的 --delete 被误判为 delete 收藏子命令)
    if first in ALIASES:
        argv[argv.index(first)] = ALIASES[first]
        return argv
    if first in COMMANDS:
        return argv

    # 旧写法 `--list ...` / `--add ...` 等转为对应子命令(其余参数原样保留)
    for cmd in COMMANDS:
        flag = f"--{cmd}"
        if flag in argv:
            return [cmd] + [a for a in argv if a != flag]

    # 省略子命令时的兜底推断
    if first is None:
        if "--help" in argv or "-h" in argv:
            pass
        elif not argv:
            _usage_exit()
        else:
            argv = ["push"] + argv  # 兼容旧写法: --all / --dry-run 等 -> push
    elif first.startswith(("http://", "https://")):
        argv = ["add"] + argv  # URL 首参自动映射到 add 收藏
    else:
            ds = Path(_scan(argv, "--dataset-dir", str(DEFAULT_DATASET_DIR))).resolve()
            md = Path(_scan(argv, "--model-dir", str(DEFAULT_MODEL_DIR))).resolve()
            if (ds / first).is_dir() or (md / first).is_dir():
                argv = ["push"] + argv
            else:
                sys.exit(f"[错误] 未知命令: {first}\n"
                         f"可用命令: {', '.join(COMMANDS)}\n"
                         "提示: 上传本地目录请用 push <name>; 收藏远程仓库请用 add <URL|repo_id>")
    return argv


def add_common_args(p: argparse.ArgumentParser) -> None:
    """注册全局公共参数(供顶层命令与嵌套子命令复用, 如 card set / collection add)。"""
    p.add_argument("--dataset-dir", default=str(DEFAULT_DATASET_DIR),
                   help="数据集根目录(本地路径, 默认 ubt_IL/dataset)")
    p.add_argument("--model-dir", default=str(DEFAULT_MODEL_DIR),
                   help="模型根目录(本地路径, 默认 ubt_IL/model)")
    p.add_argument("--remote-user", dest="remote_user",
                   help="远程仓库命名空间(账号/组织名, 默认=登录账号)")
    p.add_argument("--type", dest="repo_type", choices=TYPES, default=TYPE_DATASET,
                   help="仓库类型: dataset|model(默认 dataset; URL / 收藏命中时自动推断)")


def build_parser() -> argparse.ArgumentParser:
    """工厂: 构建根解析器 + 各子命令解析器(由命令对象注册自身参数)。"""
    p = argparse.ArgumentParser(
        description="管理 Hugging Face 数据集与模型: 查询 / 上传(push) / 下载(pull) / 收藏(add/delete) / 卡片")
    common = argparse.ArgumentParser(add_help=False)
    add_common_args(common)
    sub = p.add_subparsers(dest="command", required=True,
                           metavar="{list,push,pull,add,delete,card,collection,rm,visibility,duplicate}")
    for cmd in COMMANDS.values():
        cp = sub.add_parser(cmd.name, parents=[common], help=cmd.help)
        cmd.add_arguments(cp)
    return p


def main():
    args = build_parser().parse_args(_normalize_argv(sys.argv[1:]))
    cmd = COMMANDS[args.command]
    cmd.validate(args)
    cmd.run(args, Context(args))


if __name__ == "__main__":
    main()
