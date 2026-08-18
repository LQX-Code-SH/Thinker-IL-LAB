# UBTECH-IL-LAB 文档站点

本分支（`docs-source`）是文档站点的**源码分支**，仅包含 MkDocs 文档源码，与代码分支互不干扰。

线上站点：<https://lqx-code-sh.github.io/UBTECH-IL-LAB/>

## 分支结构

| 分支 | 内容 | 维护方式 |
|------|------|----------|
| `docs-source` | `mkdocs.yml` + `docs/`（本文档源码） | 手工编辑，提交到此分支 |
| `gh-pages` | 构建产物（HTML） | 由 `mkdocs gh-deploy` 自动生成，**不要手工修改** |
| `main` / `future` 等 | 项目代码 | 文档不进代码分支 |

## 目录说明

```
docs-source/
├── mkdocs.yml          # 站点配置：导航、主题、site_url / repo_url
├── .gitignore          # 忽略本地构建输出 site/
└── docs/
    ├── index.md            # 首页
    ├── getting-started.md  # 快速开始
    ├── architecture.md     # 架构总览
    ├── tienkung/           # 天工 Pro 文档线
    ├── walker-s2/          # Walker S2 文档线
    ├── common/             # 通用参考（数据转换 / 回放 / HF 管理）
    └── assets/             # 图片、演示视频等素材
```

## 环境要求

- Python 3.11+
- `mkdocs` 1.6.x 与 `mkdocs-material` 9.7.x：

```bash
pip install "mkdocs==1.6.1" "mkdocs-material==9.7.6"
```

## 日常编辑流程

```bash
git checkout docs-source

# 本地预览（http://127.0.0.1:8000，保存后实时刷新）
mkdocs serve

# 编辑 docs/ 下的 Markdown ...
git add docs/ && git commit -m "文档：..." && git push

# 构建并部署（自动替换 gh-pages 分支并推送，约 1 分钟后线上生效）
mkdocs gh-deploy
```

## 常见操作

**新增页面**：在 `docs/` 下新建 `xxx.md`，并在 `mkdocs.yml` 的 `nav` 中登记，否则页面可访问但不会出现在导航菜单。

**修改导航 / 站点名称 / 主题配色**：改 `mkdocs.yml`。

**添加图片或视频**：放入 `docs/assets/`，在 Markdown 中以 `assets/xxx.png` 相对路径引用。

**注意**：

- `mkdocs gh-deploy` 会整个替换 `gh-pages` 分支，手工改 `gh-pages` 的内容会在下次部署时丢失。
- 改完 `mkdocs.yml` 中的 `site_url` / `repo_url` 需与实际部署仓库一致，否则页面内链接与 canonical 地址会指错。
- 大体积视频建议压缩后再放入 `docs/assets/`（当前素材约 59MB，会整体进仓库与站点）。
