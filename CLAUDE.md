# CodeGraph

本项目已建 CodeGraph 索引。

- 查符号定义/调用链/依赖/影响范围 → 先调 CodeGraph MCP 工具
- ❌ 禁止在未查图谱前对整个 repo 做 Grep/Glob 全量扫描
- ✅ 仅图谱定位到文件+行号后再 Read 具体文件

# 图片文件

- ❌ 禁止用 Read 读取仓库内图片文件（.png/.jpg/.jpeg 等）：当前模型不支持图像输入，读取会报错
- ✅ 涉及图片时只做路径引用（markdown 链接、`ls` 确认存在），不打开图片内容
