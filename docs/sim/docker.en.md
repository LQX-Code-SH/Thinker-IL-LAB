# Simulation Platform: Container Setup

`ubt_sim` runs in a Docker container to isolate it from the host environment and simplify migration. The image contains ROS 2 and Isaac Sim and supports multiple robots.

Commands are split into two parts: build/enter the container on the **host**; launch simulation and collect data inside the **container** (project root is `/ubt_sim`).

## Build and start the container (host)

```bash
cd ubt_sim/docker
bash run.sh build      # build the ROS 2 + Isaac Sim image
bash run.sh start      # create/start the ubt-sim container
bash run.sh init       # install dependencies in the container (first build only)
bash run.sh check      # verify GPU / ROS 2 / message packages / numpy<2
bash run.sh bash       # enter the container (ROS 2 environment sourced automatically)
```

> ⚠️ **Robot-specific, must be changed before building**: edit `interfaceWhiteList` in `ubt_sim/docker/fastdds_no_shm.xml` and set `<address>` to the robot's direct-link subnet: Walker TienKung Pro `192.168.41.99`, Walker S2 EDU / Walker C1 EDU `192.168.11.99`, Walker TienKung Dex (Thor board `192.168.11.3`). The file is copied into the image at build time, so editing it afterwards has no effect.

To separate simulation and real-robot ROS 2 domains, set `ROS_DOMAIN_ID` when starting the container (default 0):

```bash
ROS_DOMAIN_ID=0 bash run.sh start
```

## Common run.sh commands

```text
build         Build the image
start         Create/start the container
stop          Stop the container (stops the bridge first)
restart       Restart the container
bash          Enter the container shell (ROS 2 sourced automatically)
rm            Remove the container
init          Install dependencies, build ROS 2 messages and the C++ image bridge
check         Verify the environment
bridge-start  Start the ROS2-ZMQ bridge separately
bridge-stop   Stop the bridge separately
```

## Troubleshooting

- **`bash run.sh check` fails**: follow the prompt and rerun `bash run.sh init`; verify `numpy<2` and that `bodyctrl_msgs` plus the robot-specific SDK ROS 2 message packages are available.
- **X11 unavailable / simulation window not shown**: run `xhost +` on the host and check that `DISPLAY` is forwarded; on a headless server use `UBT_SIM_LOAD_ONLY=1` for preview only.

## Next steps

- [Platform overview](index.md) - back to the simulation platform overview
- [Walker TienKung Pro](../tienkung/sim-setup.md) - Walker TienKung Pro simulation guide
- [Walker TienKung Dex](../tienkung-dex/sim-setup.md) - single-arm dexterous manipulation simulation
- [Walker S2 EDU](../walker-s2/sim-setup.md) - Walker S2 EDU simulation guide
- [Walker C1 EDU](../walker-c1/sim-setup.md) - Walker C1 EDU simulation guide
