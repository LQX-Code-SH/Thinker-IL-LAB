# Thinker-IL-LAB 文档站点

本分支（`docs-source`）是文档站点的**源码分支**，仅包含 MkDocs 文档源码，与代码分支互不干扰。

线上站点：<https://lqx-code-sh.github.io/Thinker-IL-LAB/>

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
    ├── tienkung-dex/       # 天工行者 Dex 文档线
    ├── walker-s2/          # Walker S2 文档线
    ├── walker-c1/          # Walker C1 文档线
    ├── common/             # 通用参考（数据转换 / 回放 / HF 管理）
    └── assets/             # 图片、演示视频等素材
```

## 环境要求

- Python 3.11+
- `mkdocs` 1.6.x、`mkdocs-material` 9.7.x 与 `mkdocs-static-i18n` 1.3.x：

```bash
pip install "mkdocs==1.6.1" "mkdocs-material==9.7.6" "mkdocs-static-i18n==1.3.1"
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

## 中英文双站

站点使用 `mkdocs-static-i18n` 插件，采用**后缀结构**：

- 中文（默认）：`docs/xxx.md`，访问 `/`
- 英文：`docs/xxx.en.md`，访问 `/en/`
- 右上角语言选择器由插件自动生成，按当前页面跳转到对应语言版本
- 未提供 `.en.md` 的页面会**回退显示中文**（`fallback_to_default: true`），因此可以分批翻译，不影响构建
- 导航英文标题在 `mkdocs.yml` 的 `i18n.languages[nav_translations]` 中维护
- 英文页面的 `<video>` 相对路径由 `hooks/fix_i18n_assets.py` 自动补一级 `../`（英文页多一层 `/en/`）；Markdown 图片由 MkDocs 自动重写，无需处理

新增页面的英文版只需在中文文件同目录下创建同名 `.en.md`，无需改导航。

英文机型名称统一为：**Walker TienKung Pro**、**Walker TienKung Dex**、**Walker S2 EDU**、**Walker C1 EDU**。

当前已完成英文翻译的页面：

- 首页、快速开始、架构总览
- 仿真平台概览 / 容器构建与使用
- 模仿学习平台概览 / 容器构建与使用
- 天工行者 / 天工行者 Dex / Walker S2 / Walker C1 四篇机型概览
- Walker C1 完整文档线（8 篇）：概览 / 仿真平台 / 转换训练 / 仿真全流程 / 真机全流程 / 真机部署 / 推理服务器 / 技术架构
- Walker TienKung Dex 完整文档线（6 篇）：概览 / 仿真平台 / 转换训练 / 仿真全流程 / 真机全流程 / 机器人本体部署
- Walker TienKung Pro 完整文档线（6 篇）：概览 / 仿真平台 / 转换训练 / 仿真全流程 / 真机全流程 / ARM 板端部署
- Walker S2 EDU 完整文档线（9 篇）：概览 / 仿真平台 / 转换训练 / 仿真全流程 / 真机全流程 / 真机部署 / 推理服务器 / 腕部相机 / 应用案例
- 通用参考（5 篇）：数据转换 / 容器环境 / HDF5 布局 / HF 数据与模型 / 数据可视化与回放

**当前覆盖率 41 / 42 篇**。`walker-s2/cc-api.md`（4,596 行的优必选 SDK 二次开发手册）按决策**保留中文**：该文档主体为接口表、代码示例与现场操作步骤，代码本身为英文；英文站中该页回退显示中文，其余页面均为英文。

## 常见操作

**新增页面**：在 `docs/` 下新建 `xxx.md`，并在 `mkdocs.yml` 的 `nav` 中登记，否则页面可访问但不会出现在导航菜单。需要英文版时再补 `xxx.en.md`。

**修改导航 / 站点名称 / 主题配色**：改 `mkdocs.yml`。

**添加图片或视频**：放入 `docs/assets/`，在 Markdown 中以 `assets/xxx.png` 相对路径引用。

**注意**：

- `mkdocs gh-deploy` 会整个替换 `gh-pages` 分支，手工改 `gh-pages` 的内容会在下次部署时丢失。
- 改完 `mkdocs.yml` 中的 `site_url` / `repo_url` 需与实际部署仓库一致，否则页面内链接与 canonical 地址会指错。
- 大体积视频建议压缩后再放入 `docs/assets/`（当前素材约 59MB，会整体进仓库与站点）。
