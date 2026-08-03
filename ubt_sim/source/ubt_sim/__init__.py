# Copyright (c) 2022-2025, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""
Python module serving as a project/extension template.
"""

# Register Gym environments.
from .task.tienkung_pro_parlor import *
from .task.walker_s2_parlor import *
from .task.walker_s2_part_sorting import *
from .task.walker_s2_pick_part import *
from .utils import monkey_patch
