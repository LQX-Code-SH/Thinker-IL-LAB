# Application Case: VLA Pick-and-Carry

Scenario: a tabletop VLA tidying task with the Walker S2 EDU humanoid robot. The robot picks parts from the table, places them into a storage box, and then carries the box to a distant target.

Approach:

- Train a tabletop-tidying VLA model with the Thinker-IL-LAB toolchain.
- Deploy with the inference server and hot-load the picking model; call the service with the inference client.
- Build a task manager for robot navigation, carrying, and VLA picking to switch between multiple tasks and complete the full flow. For robot navigation and carrying, see the [Walker S2 high-level motion interface](https://github.com/LQX-Code-SH/Walker-S2-High-Level-SDK-Demo.git).

## Demo video

<video src="../../assets/walker-s2应用案例.mp4" controls muted width="100%"></video>

## Further reading

- [Real-robot workflow](real-workflow.md) - End-to-end: real collection -> conversion -> training -> evaluation -> deployment
- [Inference server](inference-server.md) - Warm start, ZMQ commands, model hot-swap
