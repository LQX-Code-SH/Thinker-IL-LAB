"""灵巧手控制工具（tienkung_dex 插件侧）。

因时/强脑两种手在**线缆上**走的都不是策略侧量纲：因时是 13 个 0.1° 角码，
强脑是 6 个 1~1000 通道码。策略侧量纲（= 训练数据集口径）：
- 强脑：**闭合度 [0, 1]**（数据集 action/state 都是 0~1）；
- 因时：关节角 rad（暂时不变，后续改归一化 + 13 维展开）。

桥接（ros2_deploy_bridge.py）负责 策略量纲 → 角码，并顺带按通道上下限夹取；
插件这里做同一件事，目的只有一个——让 ``send_action()`` 返回的是**真机真正会
执行的值**（与 lerobot 其它机器人的语义一致），否则推理循环里记录的 action
会比硬件实际值"精细"，排查 sim2real 差异时会误导。

转换表本身在 constants.py，两边共用同一份数学（桥接端是运行期副本）。
"""

from __future__ import annotations

from .constants import hand_action_to_code, hand_code_to_value


def clip_hand_value(
    pos: float,
    variant: str = "inspire",
    channel: int = 0,
    joint_name: str = "",
) -> float:
    """单个手部控制量 → 真机通道分辨率下的实际下发值（策略量纲）。

    ``channel`` 是该关节在硬件分组里的下标（与真机通道号一一对应）；
    强脑变体还需要 ``joint_name``（当前换算不再用它，保留签名以兼容调用方）。
    返回值与入参同量纲：强脑是 0~1 闭合度，因时是 rad。
    """
    code = hand_action_to_code(variant, channel, pos, joint_name)
    return hand_code_to_value(variant, channel, code, joint_name)


def clip_hand_position(
    position: list,
    variant: str = "inspire",
    joints: list[str] | None = None,
) -> list:
    """整组手部关节角 → 真机通道分辨率下的实际下发值，按硬件通道序。"""
    if joints is None:
        return [clip_hand_value(pos, variant, i, "") for i, pos in enumerate(position)]
    return [
        clip_hand_value(pos, variant, i, name)
        for i, (pos, name) in enumerate(zip(position, joints))
    ]
