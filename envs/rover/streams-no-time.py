from typing import Dict, Optional, Any, List, Tuple, Union
from coast.action import Action
import random
from collections import defaultdict
from coast.object import Object
from pybullet_tools import pr2_primitives
from stream_utils import get_inv_vis_gen, get_motion_fn, get_cfree_ray_test, get_inv_com_gen, get_above_gen
from coast.formula import _and, _forall, _not, _P, _when
from utils import Register, Scan
from pybullet_tools.pr2_primitives import Conf, control_commands, Attach, Detach
import coast
KINECT_FRAME = "camera_rgb_optical_frame"
VIS_RANGE = 2

# TODO: Add certified fact to task state or remove preconditions from domain?
GEOMETRIC_PREDICATES = [
    _P("AtConf", "?v", "?q"),
    _P("Holding", "?v", "?r"),
]

class drop_rock(Action):
    # Move to rock and take sample
    parameters = [
        Object("?v", "rover"),
        Object("?s", "store"),
    ]
    inputs = [Object("?r", "rock")]
    outputs = []
    precondition = _and(_P("Holding", "?v", "?r"))
    effect = _and(_not(_P("Holding", "?v", "?r")))
    
    certified = _and(
    )

    def execute_state(
        self,
        state: pr2_primitives.State,
        problem: Any,
        robot: Any,
        arm: Any,
        inputs: Dict[str, Any],
        outputs: Dict[str, Any],
        objects,
        args_map,
    ) -> bool:
        v = objects[args_map[self.parameters[0].name]].value
        r = inputs["r"]

        pr2_primitives.apply_commands(
            state, [Detach(v, arm='base_link', body=r)]
        )
        return True

class sample_rock(Action):
    # Move to rock and take sample
    parameters = [
        Object("?v", "rover"),
        Object("?r", "rock"),
        Object("?s", "store"),
    ]
    inputs = [Object("?q1", "conf")]
    outputs = [Object("?q2", "conf"), Object("?t", "traj")]
    precondition = _and(_P("AtConf", "?v", "?q1"))
    effect = _and(_P("AtConf", "?v", "?q2"), _not(_P("AtConf", "?v", "?q1")), _P("Holding", "?v", "?r"))
    
    certified = _and(
        _P("SampleAbove", "?v", "?r", "?q2"), # Sample conf above rock
        _P("SampleMotion", "?v", "?q1", "?q2", "?t")  # Produce trajectory to rock
    )

    def execute_state(
        self,
        state: pr2_primitives.State,
        problem: Any,
        robot: Any,
        arm: Any,
        inputs: Dict[str, Any],
        outputs: Dict[str, Any],
        objects,
        args_map,
    ) -> bool:
        v = objects[args_map[self.parameters[0].name]].value
        r = objects[args_map[self.parameters[1].name]].value

        pr2_primitives.apply_commands(
            state, [outputs["t"], Attach(v, arm='base_link', grasp=None, body=r)]
        )
        return True

class send_analysis(Action):
    # Move to spot where com to lander is visible, then send information
    parameters = [
        Object("?v", "rover"),
        Object("?l", "lander"),
        Object("?r", "rock"),
    ]
    inputs = [Object("?q1", "conf")]
    outputs = [Object("?y", "ray"), Object("?q2", "conf"), Object("?t", "traj")]
    precondition = _and(_P("AtConf", "?v", "?q1"))
    effect = _and(_P("AtConf", "?v", "?q2"), _not(_P("AtConf", "?v", "?q1")))
    certified = _and(
        _P(
            "SampleCommConf", "?v", "?l", "?y", "?q2"
        ),  # Stream should produce cfree ray and conf
        _P("TestCfreeRayConf", "?y", "?v", "?q2"), # Test if ray intersects rover?
        _P("SampleMotion", "?v", "?q1", "?q2", "?t"),  # Produce trajectory
    )

    def execute_state(
        self,
        state: pr2_primitives.State,
        problem: Any,
        robot: Any,
        arm: Any,
        inputs: Dict[str, Any],
        outputs: Dict[str, Any],
        objects,
        args_map,
    ) -> bool:
        traj = outputs["t"]
        pr2_primitives.apply_commands(
            state, [traj, outputs["y"]]
        )
        return True
class send_image(Action):
    # Move to spot where com to lander is visible, then send image
    parameters = [
        Object("?v", "rover"),
        Object("?l", "lander"),
        Object("?r", "rock"),
    ]
    inputs = [Object("?q1", "conf")]
    outputs = [Object("?y", "ray"), Object("?q2", "conf"), Object("?t", "traj")]
    precondition = _and(_P("AtConf", "?v", "?q1"))
    effect = _and(_P("AtConf", "?v", "?q2"), _not(_P("AtConf", "?v", "?q1")))
    certified = _and(
        _P(
            "SampleCommConf", "?v", "?l", "?y", "?q2"
        ),  # Stream should produce cfree ray and conf
        _P("TestCfreeRayConf", "?y", "?v", "?q2"), # Test if ray intersects rover?
        _P("SampleMotion", "?v", "?q1", "?q2", "?t"),  # Produce trajectory
    )

    def execute_state(
        self,
        state: pr2_primitives.State,
        problem: Any,
        robot: Any,
        arm: Any,
        inputs: Dict[str, Any],
        outputs: Dict[str, Any],
        objects,
        args_map,
    ) -> bool:
        traj = outputs["t"]
        pr2_primitives.apply_commands(
            state, [traj, outputs["y"]]
        )
        return True
class take_image(Action):
    # Just take picture without moving
    parameters = [
        Object("?v", "rover"),
        Object("?o", "objective"),
        Object("?c", "camera"),
        Object("?m", "mode"),
    ]
    inputs = []
    outputs = []
    precondition = _and()
    effect = _and()
    certified = _and(
        
    )

    def execute_state(
        self,
        state: pr2_primitives.State,
        problem: Any,
        robot: Any,
        arm: Any,
        inputs: Dict[str, Any],
        outputs: Dict[str, Any],
        objects,
        args_map,
    ) -> bool:
        v = objects[args_map[self.parameters[0].name]].value
        o = objects[args_map[self.parameters[1].name]].value
        pr2_primitives.apply_commands(
            state, [Scan(v, o, detect=False, camera_frame=KINECT_FRAME)]
        )
        return True

class calibrate(Action):
    # Sample valid ray and conf and move there and send ray to objective
    parameters = [
        Object("?v", "rover"),
        Object("?o", "objective"),
        Object("?c", "camera"),
    ]
    inputs = [Object("?q1", "conf")]
    outputs = [Object("?y", "ray"), Object("?q2", "conf"), Object("?t", "traj")]
    precondition = _and(_P("AtConf", "?v", "?q1"))
    effect = _and(_P("AtConf", "?v", "?q2"), _not(_P("AtConf", "?v", "?q1")))
    certified = _and(
        _P(
            "SampleRayConf", "?v", "?o", "?y", "?q2"
        ),  # Stream should produce cfree ray and conf
        _P("TestCfreeRayConf", "?y", "?v", "?q2"), # Test if ray intersects rover?
        _P("SampleMotion", "?v", "?q1", "?q2", "?t"),  # Produce trajectory
    )

    def execute_state(
        self,
        state: pr2_primitives.State,
        problem: Any,
        robot: Any,
        arm: Any,
        inputs: Dict[str, Any],
        outputs: Dict[str, Any],
        objects,
        args_map,
    ) -> bool:
        v = objects[args_map[self.parameters[0].name]].value
        o = objects[args_map[self.parameters[1].name]].value
        traj = outputs["t"]
        pr2_primitives.apply_commands(
            state, [traj, Register(v, o, camera_frame=KINECT_FRAME, max_depth=VIS_RANGE)]
        )
        return True


class TestCfreeRayConf(coast.Stream):
    name = "TestCfreeRayConf"
    inputs = [
        Object("?y", "ray"),
        Object("?v", "rover"),
        Object("?q2", "conf")
    ]
    outputs: List[str] = []
    fluents: List[str] = []

    def __init__(self, world: Any):
        super().__init__(world)
        self._test = get_cfree_ray_test(
            problem=world.problem, collisions=True
        )

    def sample(
        self, inputs: Dict[str, Any], fluents: Dict[str, List[Dict[str, Any]]]
    ) -> Optional[Union[Dict[str, Any], coast.StreamConstraint]]:
        res = self._test(
            inputs["y"], inputs["v"], inputs["q2"]
        )
        return None if not res else {}
        
class SampleMotion(coast.Stream):
    name = "SampleMotion"
    inputs = [Object("?v", "rover"), Object("?q1", "conf"), Object("?q2", "conf")]
    outputs = [Object("?t", "traj")]
    fluents: List[str] = []

    def __init__(self, world: Any):
        super().__init__(world)
        self._sample_motion = get_motion_fn(
            world.problem, custom_limits=world.custom_limits, holonomic=False,
                reversible=True, teleport=False,
                use_aabb=True
        )

    def sample(
        self, inputs: Dict[str, Any], fluents: Dict[str, List[Dict[str, Any]]]
    ) -> Optional[Union[Dict[str, Any], coast.StreamConstraint]]:
        q1 = inputs["q1"]
        q2 = inputs["q2"]
        rover = inputs["v"]
        command = self._sample_motion(rover, q1, q2)
        if command is None:
            return None
        return {"t": command}

class SampleCommConf(coast.Stream):
    name = "SampleCommConf"
    inputs = [Object("?v", "rover"), Object("?l", "lander")]
    outputs = [Object("?y", "ray"), Object("?q2", "conf")]
    fluents: List[str] = []

    def __init__(self, world: Any):
        super().__init__(world, resample_p=0.2)  # 0.22 0.2

        self._get_inv_ray_conf = get_inv_com_gen(world.problem, custom_limits=world.custom_limits,holonomic=False, max_attempts=100,
                reversible=True, teleport=False,
                use_aabb=True)
        self.cache = defaultdict(list)
        self.use_cache_p = 0

    def sample(
        self, inputs: Dict[str, Any], fluents: Dict[str, List[Dict[str, Any]]]
    ) -> Optional[Union[Dict[str, Any], coast.StreamConstraint]]:
        rover = inputs["v"]
        lander = inputs["l"]
        if (rover, lander) in self.cache:
            print("In cache")
            if random.random() < self.use_cache_p:
                # Use cache
                print("Using cached")
                ray, q2 = random.choice(self.cache[(rover, lander)])
                return {"y": ray, "q2": q2}
        try:
            q2, ray = next(self._get_inv_ray_conf(rover, lander))
        except Exception:
            return None
        self.cache[(rover, lander)].append((ray, q2))
        return {"y": ray, "q2": q2}

class SampleAbove(coast.Stream):
    name = "SampleAbove"
    inputs = [Object("?v", "rover"), Object("?r", "rock")]
    outputs = [Object("?q2", "conf")]
    fluents: List[str] = []

    def __init__(self, world: Any):
        super().__init__(world, resample_p=0.2)  # 0.22 0.2

        self._get_above = get_above_gen(world.problem, custom_limits=world.custom_limits, holonomic=False,
                reversible=True, teleport=False, iterations=30,
                use_aabb=True)
        self.cache = defaultdict(list)
        self.use_cache_p = 0

    def sample(
        self, inputs: Dict[str, Any], fluents: Dict[str, List[Dict[str, Any]]]
    ) -> Optional[Union[Dict[str, Any], coast.StreamConstraint]]:
        rover = inputs["v"]
        rock = inputs["r"]
        # try:
        # return None
        if (rover, rock) in self.cache:
            print("In cache")
            if random.random() < self.use_cache_p:
                # Use cache
                print("Using cached")
                q2 = random.choice(self.cache[(rover, rock)])
                return {"q2": q2}
        q2 = next(self._get_above(rover, rock))
        if q2 is None:
            return None
        self.cache[(rover, rock)].append(q2)
        return {"q2": q2}
class SampleRayConf(coast.Stream):
    name = "SampleRayConf"
    inputs = [Object("?v", "rover"), Object("?o", "objective")]
    outputs = [Object("?y", "ray"), Object("?q2", "conf")]
    fluents: List[str] = []

    def __init__(self, world: Any):
        super().__init__(world, resample_p=0.2)  # 0.22 0.2

        self._get_inv_ray_conf = get_inv_vis_gen(world.problem, custom_limits=world.custom_limits, holonomic=False, max_attempts=25,
                reversible=True,
                use_aabb=True, teleport=False)
        self.cache = defaultdict(list)
        self.use_cache_p = 0
    def sample(
        self, inputs: Dict[str, Any], fluents: Dict[str, List[Dict[str, Any]]]
    ) -> Optional[Union[Dict[str, Any], coast.StreamConstraint]]:
        rover = inputs["v"]
        objective = inputs["o"]
        if (rover, objective) in self.cache:
            print("In cache")
            if random.random() < self.use_cache_p:
                # Use cache
                print("Using cached")
                q2, ray = random.choice(self.cache[(rover, objective)])
                return {"y": ray, "q2": q2}
        try:
            q2, ray = next(self._get_inv_ray_conf(rover, objective))
        except Exception:
            return None
        self.cache[(rover, objective)].append((q2, ray))
        return {"y": ray, "q2": q2}
