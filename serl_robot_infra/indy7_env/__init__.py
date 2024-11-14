from gym.envs.registration import register
import numpy as np

register(
    id="Indy7Env-Vision-v0",
    entry_point="indy7_env.envs:Indy7Env",
    max_episode_steps=100,
)

register(
    id="Indy7PegInsert-Vision-v0",
    entry_point="indy7_env.envs.peg_env:Indy7PegInsert",
    max_episode_steps=100,
)

register(
    id="Indy7PCBInsert-Vision-v0",
    entry_point="indy7_env.envs.pcb_env:Indy7PCBInsert",
    max_episode_steps=100,
)

register(
    id="Indy7CableRoute-Vision-v0",
    entry_point="indy7_env.envs.cable_env:Indy7CableRoute",
    max_episode_steps=100,
)

register(
    id="Indy7BinRelocation-Vision-v0",
    entry_point="indy7_env.envs.bin_relocation_env:Indy7BinRelocation",
    max_episode_steps=100,
)
