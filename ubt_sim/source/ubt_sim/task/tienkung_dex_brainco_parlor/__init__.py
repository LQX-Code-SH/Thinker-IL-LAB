import gymnasium as gym

gym.register(
    id="UBTSim-TienkungDex-Brainco-Parlor-v0",
    entry_point="ubt_sim.env:ManagerBasedRLDigitalTwinEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": f"{__name__}.tienkung_dex_brainco_parlor_env_cfg:TienkungDexBraincoParlorEnvCfg",
    },
)
