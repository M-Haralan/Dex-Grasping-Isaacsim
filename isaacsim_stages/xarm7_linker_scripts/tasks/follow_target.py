# Martin Haralanov
# Modified from the Isaacsim standalone_examples
# In examples, it had a gripper class defined
# Currently no gripper class defined here

from typing import Optional
import isaacsim.core.api.tasks as tasks
import numpy as np
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.robot.manipulators.manipulators import SingleManipulator
from isaacsim.storage.native import get_assets_root_path
import os

# Inheriting from the base class Follow Target (provided by isaacsim)
class FollowTarget(tasks.FollowTarget):
    def __init__(
        self,
        name: str = "xarm7_follow_target",
        target_prim_path: Optional[str] = None,
        target_name: Optional[str] = None,
        target_position: Optional[np.ndarray] = None,
        target_orientation: Optional[np.ndarray] = None,
        offset: Optional[np.ndarray] = None,
    ) -> None:
        tasks.FollowTarget.__init__(
            self,
            name=name,
            target_prim_path=target_prim_path,
            target_name=target_name,
            target_position=target_position,
            target_orientation=target_orientation,
            offset=offset,
        )
        return

    def set_robot(self) -> SingleManipulator:
        asset_path = os.path.join(os.path.dirname(__file__), "../../xarm7_for_linker.usd")  # update if switching environments
        
        add_reference_to_stage(usd_path=asset_path, prim_path="/UF_ROBOT")
        
        manipulator = SingleManipulator(
            prim_path="/UF_ROBOT",  
            name="xarm7_robot",
            end_effector_prim_path="/UF_ROBOT/link7", # last link in the xarm7
        )
        return manipulator
