# 仿真平台：容器构建与使用

`ubt_sim` 仿真平台使用Docker容器管理运行环境，与宿主机环境隔离，便于迁移部署。容器内包含（ROS2 + Isaac Sim 镜像），兼任多款机器人的仿真环境。

命令分两段：构建/进入容器在**宿主机**执行；启动仿真与采集在**容器内**执行（容器内根路径为 `/ubt_sim`）。

## 构建并启动容器（宿主机）

```bash
cd ubt_sim/docker
bash run.sh build      # 构建 ROS2 + Isaac Sim 镜像
bash run.sh start      # 创建/启动容器 ubt-sim
bash run.sh init       # 容器安装相关依赖,只需首次构建容器时执行一次
bash run.sh check      # 校验环境（GPU / ROS2 / 消息包 / numpy<2 等）
bash run.sh bash       # 进入容器 shell（自动 source ROS2 环境）
```

如需区分真机/仿真的 ROS2 域，用 `ROS_DOMAIN_ID` 启动容器（默认 0）：

```bash
ROS_DOMAIN_ID=0 bash run.sh start
```

## 常用 run.sh 命令

```text
build         构建镜像
start         创建/启动容器
stop          停止容器（先停桥接）
restart       重启容器
bash          进入容器 shell（自动 source ROS2）
rm            删除容器
init          容器内安装依赖、构建 ROS2 消息与 C++ 图像桥接
check         校验环境
bridge-start  单独启动 ROS2-ZMQ 桥接
bridge-stop   单独停止桥接
```

## 常见问题

- **`bash run.sh check` 报错**：按提示重跑 `bash run.sh init`；确认 `numpy<2`、`bodyctrl_msgs` 与对应机型 SDK ROS2 消息包均已就绪。
- **X11 不可用 / 仿真窗口不显示**：宿主机执行 `xhost +`，确认 `DISPLAY` 已透传；无显示器时仅可用 `UBT_SIM_LOAD_ONLY=1` 预览。


## 下一步

- [平台概览](index.md) - 返回仿真平台介绍页
- [天工行者](../tienkung/sim-setup.md) - 天工机器人仿真使用说明 
- [Walker S2 EDU 探索者](../walker-s2/sim-setup.md) - Walker S2 EDU 探索者 机器人仿真使用说明
