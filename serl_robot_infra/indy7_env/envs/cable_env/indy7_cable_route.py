import numpy as np
import time
import requests
import copy

from indy7_env.envs.indy7_env import Indy7Env
from indy7_env.utils.rotations import euler_2_quat
from indy7_env.envs.cable_env.config import CableEnvConfig

##############################################################################


class FrankaCableRoute(Indy7Env):
    def __init__(self, **kwargs):
        super().__init__(**kwargs, config=CableEnvConfig)

    def go_to_rest(self, joint_reset=False):
        """
        Move to the rest position defined in base class.
        Add a small z offset before going to rest to avoid collision with object.
        """
        self._send_gripper_command(-1)
        self._update_currpos()
        self._send_pos_command(self.currpos)
        time.sleep(0.5)

        # Move up to clear the slot
        self._update_currpos()
        reset_pose = copy.deepcopy(self.currpos)
        reset_pose[2] += 0.05
        self.interpolate_move(reset_pose, timeout=1)

        # execute the go_to_rest method from the parent class
        super().go_to_rest(joint_reset)
