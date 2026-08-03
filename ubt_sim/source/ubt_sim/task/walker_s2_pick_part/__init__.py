import gymnasium as gym

gym.register(
    id="UBTSim-WalkerS2-PickPart-v0",
    entry_point="ubt_sim.env:ManagerBasedRLDigitalTwinEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": f"{__name__}.walker_s2_pick_part_env_cfg:WalkerS2PickPartEnvCfg",
    },
)
