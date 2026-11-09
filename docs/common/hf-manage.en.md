# HF Dataset/Model Management (hf_manager.py)

Script: `ubt_IL/scripts/convert/common/hf_manager.py`

A single tool for Hugging Face **dataset** and **model** operations: list, push, pull, bookmark (add/delete), and model cards. Datasets and models share the same commands; use `--type` to switch (default `dataset`).

## Installation and directory conventions

Requires Python 3.10+ and `huggingface_hub>=1.0`:

```bash
pip install "huggingface_hub>=1.0"
python ubt_IL/scripts/convert/common/hf_manager.py list --local   # no network required; verifies the setup immediately
```

| Type | Local root (default) |
|---|---|
| dataset | `ubt_IL/dataset` |
| model | `ubt_IL/model` |

Override with the global `--dataset-dir` / `--model-dir` options.

## Login and mirror

Token priority: environment variable `HF_TOKEN` (recommended) -> huggingface_hub login cache (`hf auth login`).

```bash
export HF_TOKEN=hf_xxxxxxxx     # option A
hf auth login                   # option B (interactive)
```

In mainland China, set the mirror first (the script probes `huggingface.co` for 5 seconds and warns if unreachable):

```bash
export HF_ENDPOINT=https://hf-mirror.com
```

- Read-only commands (`list` / `pull`) allow anonymous access to public repos; write operations (`push` / `card set` / `rm`, etc.) require login.
- `--remote-user <account>` explicitly selects the remote namespace, useful for viewing or operating on another user's public repos.

## Listing (list)

```bash
python hf_manager.py list              # datasets by default (local + remote + bookmarks)
python hf_manager.py list model        # model list
python hf_manager.py list --bookmarks  # bookmarks only
python hf_manager.py list --type model --local   # local models
```

| Argument | Description | Default |
|---|---|---|
| `{model,dataset}` (positional) | Repo type shorthand, equivalent to `--type` | `dataset` |
| `--local` / `--remote` / `--bookmarks` | List only local / remote / bookmarked repos | Off |

## Upload (push)

Upload a local dataset/model to your namespace. The repo is created automatically if absent, and the directory structure is preserved.

```bash
python hf_manager.py push walker_pick_sort                        # upload a dataset
python hf_manager.py push --type model tienkung_pick_up_act       # upload a model
python hf_manager.py push walker_pick_sort --sync                 # incremental sync (only new/changed files)
python hf_manager.py push walker_pick_sort --path meta/info.json  # upload a single file
python hf_manager.py push --all                                   # upload everything
```

| Argument | Description | Default |
|---|---|---|
| `names` (positional) | Local directory names; multiple allowed | - |
| `--all` | Upload every subdirectory under the root | Off |
| `--path <rel>` | Upload only the specified file/subdirectory | - |
| `--sync` | Incremental sync: only upload new/changed local files | Off |
| `--prune` | With `--sync`, also delete extra remote files (dangerous) | Off |
| `--delete <path...>` | Delete remote files/subdirectories | - |
| `--description` / `--description-file` | Card description text / read it from a file | - |
| `--no-readme` | Do not automatically use the local `README.md` as the card description | Off |
| `--revision <ref>` | Upload to the specified branch | `main` |

## Download (pull)

Targets support four forms, in priority order: **URL -> full repo_id -> bookmark match -> {account}/short-name**; the repo type is detected automatically.

```bash
python hf_manager.py pull qingxiangliu/walker_pick_sort                    # full repo_id
python hf_manager.py pull https://huggingface.co/datasets/qingxiangliu/walker_pick_sort   # HF URL
python hf_manager.py pull walker_pick_sort --dry-run                       # bookmark name; preview first
python hf_manager.py pull --all --jobs 4                                   # download all remote repos concurrently
```

| Argument | Description | Default |
|---|---|---|
| `--all` | Download every remote repo under the account | Off |
| `--path <rel...>` | Download only the specified files/subdirectories | - |
| `--revision <ref>` | Branch/tag/commit (reproduce historical experiments) | `main` |
| `--jobs N` | Number of concurrent repo downloads | 1 |
| `--show-card` | Show the card description before downloading | Off |

Download paths: dataset -> `ubt_IL/dataset/<name>`, model -> `ubt_IL/model/<name>`. If a non-empty local directory already exists, the download is merged automatically (same-name files are reused; only missing/changed files are fetched).

## Bookmarks (add / delete)

Bookmark public repos from other users for quick lookup/download. Bookmark file: `ubt_IL/dataset/bookmarks.json`.

```bash
python hf_manager.py add qingxiangliu/walker_pick_sort                    # bookmark a repo (URL or repo_id)
python hf_manager.py add https://huggingface.co/qingxiangliu/datasets     # bookmark an entire namespace
python hf_manager.py delete walker_pick_sort --dry-run                    # remove a bookmark
python hf_manager.py list --bookmarks                                     # view bookmarks
```

> To copy a bookmarked repo into your own account, use the `duplicate` server-side clone (below).

## Other extensions

| Subcommand | Description | Example |
|---|---|---|
| `card init/get/set` | Card management (`init` generates a local README; `get` reads the remote card; `set` edits it, requires login) | `card init walker_pick_sort --dry-run` |
| `collection create/add/show/list` | Official collection management | `collection add <slug> <items...> --create` |
| `rm` (dangerous) | Delete a remote repo | `rm qingxiangliu/old_dataset --yes` |
| `visibility` | Toggle public/private | `visibility <repo_id> --private` |
| `duplicate` | Server-side repo copy (no local transfer) | `duplicate <repo_id> --to <new_repo_id> --private` |

**Legacy compatibility**: `upload` -> `push`, `download` -> `pull`, `delete-repo` -> `rm`, `copy` -> `duplicate`; legacy forms such as `--list model` / `--add <url>` are converted to subcommands automatically.

## Global options

| Argument | Description | Default |
|---|---|---|
| `--type {dataset,model}` | Repo type | `dataset` |
| `--dataset-dir <dir>` / `--model-dir <dir>` | Override the local root | `ubt_IL/dataset` / `ubt_IL/model` |
| `--remote-user <account>` | Remote namespace (user/organization) | Logged-in account |
| `--dry-run` | Print the plan without executing | Off |
| `--revision <ref>` | pull: download branch; push: upload branch | `main` |

## FAQ

**Q: `list` shows "not logged in or network unavailable" for remote repos?**
Check the network and mirror: `export HF_ENDPOINT=https://hf-mirror.com`, `export HF_TOKEN=hf_xxx`, `curl -I https://huggingface.co`.

**Q: Upload fails with `create_commit() got an unexpected keyword argument 'branch'`?**
`huggingface_hub` 1.0+ renamed `branch` to `revision`; upgrade with `pip install -U "huggingface_hub>=1.0"`.

**Q: The local model directory has no README.md?**
`pull` downloads the remote `README.md`; you can also run `card init <name>` to generate one from local files.

**Q: `card set --type model` reports `unrecognized arguments`?**
Nested subcommands already register global options; make sure `hf_manager.py` is the latest version from the repository.
