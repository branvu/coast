from typing import Dict, Optional, Any, List, Tuple, Union

import time
import numpy as np
from ctrlutils import eigen
import spatialdyn as dyn

import coast
from coast import Object
from constraints import CfreeConstraint
from coast.action import Action
from coast.formula import _and, _forall, _not, _P, _when, Formula

from pddlstream_utils import (
    get_cfree_traj_pose_test,
    get_cfree_pose_pose_test,
    get_cfree_approach_pose_test,
    get_cfree_grasp_pose_test,
)
from pybullet_tools import pr2_primitives, utils as pb_utils
from pybullet_tools.pr2_primitives import (
    Pose,
    Conf,
    get_ik_ir_gen,
    get_motion_gen,
    get_stable_gen,
    get_grasp_gen,
    control_commands,
    GripperCommand,
    Attach,
    Detach,
    get_gripper_joints,
)
from pybullet_tools.pr2_utils import (
    get_arm_joints,
    ARM_NAMES,
    get_group_joints,
    get_group_conf,
)
from pybullet_tools.utils import (
    connect,
    get_pose,
    is_placement,
    disconnect,
    get_joint_positions,
    HideOutput,
    LockRenderer,
    wait_for_user,
    get_max_limit,
    set_renderer,
)


def compute_opspace_ik(
    ab: dyn.ArticulatedBody,
    pose: Tuple[Tuple[float, float, float], Tuple[float, float, float, float]],
    ee_offset: np.ndarray = np.array([0.0, 0.0, 0.046]),  # Default for Kuka IIWA.
) -> List[float]:
    pos = np.array(pose[0])
    quat = eigen.Quaterniond(pose[1])

    # Set nullspace.
    if dyn.opspace.is_singular(ab, dyn.jacobian(ab)):
        q = np.array([np.pi / 2, np.pi / 6, 0, -np.pi / 3, 0, np.pi / 2, 0])
    else:
        q = np.array(ab.q)
    q[0] = np.arctan2(pos[1], pos[0])
    ab.q = q

    q = dyn.inverse_kinematics(ab, pos, quat, offset=ee_offset)

    return q.tolist()


"""
class Move(Action):
    parameters = [Object("?t1", "timestep"), Object("?t2", "timestep")]
    inputs = [Object("?q1", "conf"), Object("?q2", "conf")]
    outputs = [Object("?bt", "traj")]
    precondition = _P("AtConf", "?q1")
    effect = _and(_not(_P("AtConf", "?q1")), _P("AtConf", "q2"))
    certified = _and(_P("SampleBMotion", "?q1", "?q2", "?bt"))

    def execute_state(
        self,
        state: pr2_primitives.State,
        robot: Any,
        arm: Any,
        inputs: Dict[str, Any],
        outputs: Dict[str, Any],
    ) -> bool:
        traj: pr2_primitives.Command = outputs["bt"]
        traj.apply(state)
        return True
"""


GEOMETRIC_PREDICATES = [
    _P("AtConf", "?q"),
    _P("Holding", "?b", "?g"),
    _P("AtPose", "?b", "?p"),
]


class Pick(Action):
    parameters = [
        Object("?b", "block"),
        Object("?s", "table"),
        Object("?g", "grasp"),
        Object("?t1", "timestep"),
        Object("?t2", "timestep"),
    ]
    inputs = [Object("?q1", "conf"), Object("?p", "pose")]
    outputs = [
        # Object("?g", "grasp"),
        Object("?q2", "conf"),
        Object("?t", "traj"),
        Object("?bt", "traj"),
    ]
    precondition = _and(_P("AtConf", "?q1"), _P("AtPose", "?b", "?p"))
    effect = _and(
        _not(_P("AtPose", "?b", "?p")),
        _P("Holding", "?b", "?g"),
        _not(_P("AtConf", "?q1")),
        _P("AtConf", "?q2"),
    )
    certified = _and(
        # _P("SampleGrasp", "?b", "?g"),
        _forall(
            "bb",
            "block",
            _forall(
                "pp",
                "pose",
                _when(
                    _P("AtPose", "?bb", "?pp"),
                    # _and(
                    # _P("TestGrasp", "?bb", "?pp", "?g"),
                    _P("CFreeApproachPose", "?g", "?b", "?p", "?bb", "?pp")
                    # ),
                ),
            ),
        ),
        _forall(
            "bb",
            "block",
            _forall(
                "pp",
                "pose",
                _when(
                    _P("AtPose", "?bb", "?pp"),
                    _P("CFreeTrajPose", "?t", "?bb", "?pp", "?g"),
                ),
            ),
        ),
        _P("IK", "?b", "?p", "?g", "?q2", "?t"),
        _and(_P("SampleBMotion", "?q1", "?q2", "?bt")),
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
        object = objects[args_map[self.parameters[0].name]]
        object = object.value
        self._sample_grasp = pr2_primitives.get_grasp_gen(problem, front_only=False)
        pickg = objects[args_map[self.parameters[2].name]]

        grasp: pr2_primitives.Grasp = self._sample_grasp(
            object, fixed_grasp=pickg.value
        )[0][0]
        traj: pr2_primitives.Command = outputs["t"]
        close_gripper = GripperCommand(robot, arm, grasp.grasp_width, teleport=False)
        attach = Attach(robot, arm, grasp, object)
        [t] = traj.commands
        [base_traj] = outputs["bt"]
        combine = [base_traj, t, close_gripper, attach, t.reverse()]
        pr2_primitives.apply_commands(state, combine)
        return True


class Place(Action):
    parameters = [
        Object("?b", "block"),
        Object("?s", "table"),
        Object("?g", "grasp"),
        Object("?t1", "timestep"),
        Object("?t2", "timestep"),
    ]
    inputs = [
        Object("?q1", "conf"),
        # Object("?g", "grasp")
    ]
    outputs = [
        Object("?p", "pose"),
        Object("?q2", "conf"),
        Object("?t", "traj"),
        Object("?bt", "traj"),
    ]
    precondition = _and(_P("AtConf", "?q1"), _P("Holding", "?b", "?g"))
    effect = _and(
        _not(_P("Holding", "?b", "?g")),
        _P("AtPose", "?b", "?p"),
        _not(_P("AtConf", "?q1")),
        _P("AtConf", "?q2"),
    )
    certified = _and(
        _P("SamplePose", "?b", "?s", "?p"),
        _forall(
            "bb",
            "block",
            _forall(
                "pp",
                "pose",
                _when(
                    _P("AtPose", "?bb", "?pp"),
                    _P("TestPose", "?b", "?p", "?bb", "?pp"),
                ),
            ),
        ),
        _forall(
            "bb",
            "block",
            _forall(
                "pp",
                "pose",
                _when(
                    _P("AtPose", "?bb", "?pp"),
                    _P("CFreeApproachPose", "?g", "?b", "?p", "?bb", "?pp"),
                ),
            ),
        ),
        _P("IK", "?b", "?p", "?g", "?q2", "?t"),
        _forall(
            "bb",
            "block",
            _forall(
                "pp",
                "pose",
                _when(
                    _P("AtPose", "?bb", "?pp"),
                    _P("CFreeTrajPose", "?t", "?bb", "?pp", "?g"),
                ),
            ),
        ),
        _and(_P("SampleBMotion", "?q1", "?q2", "?bt")),
    )

    def execute_state(
        self,
        state: pr2_primitives.State,
        problem: Any,
        robot,
        arm,
        inputs: Dict[str, Any],
        outputs: Dict[str, Any],
        objects,
        args_map,
    ) -> bool:
        object = objects[args_map[self.parameters[0].name]]
        object = object.value
        gripper_joint = get_gripper_joints(robot, arm)[0]
        position = get_max_limit(robot, gripper_joint)
        open_gripper = GripperCommand(robot, arm, position, teleport=False)
        detach = Detach(robot, arm, object)
        traj: pr2_primitives.Command = outputs["t"]
        [t] = traj.commands
        [base_traj] = outputs["bt"]
        new_commands = [base_traj, t, detach, open_gripper, t.reverse()]
        pr2_primitives.apply_commands(state, new_commands)
        return True


class SamplePose(coast.Stream):
    name = "SamplePose"
    inputs = [Object("?b", "block"), Object("?s", "table")]
    outputs = [Object("?p", "pose")]
    fluents: List[str] = []

    def __init__(self, world: Any):
        super().__init__(world, resample_p=0.02)  #0.02 0.22 0.2

        self._sample_pose = pr2_primitives.get_stable_gen(world.problem)

    def sample(
        self, inputs: Dict[str, Any], fluents: Dict[str, List[Dict[str, Any]]]
    ) -> Optional[Union[Dict[str, Any], coast.StreamConstraint]]:
        surface = inputs["s"]
        body = inputs["b"]
        try:
            pose = next(self._sample_pose(body=body, surface=surface))[
                0
            ]  # Could also add fluents here
        except Exception:
            return None
        return {"p": pose}


class TestPlacePose(coast.Stream):
    name = "TestPose"
    inputs = [
        Object("?b1", "block"),
        Object("?p1", "pose"),
        Object("?b2", "block"),
        Object("?p2", "pose"),
    ]
    outputs: List[Object] = []
    fluents: List[str] = []

    def __init__(self, world: Any):
        super().__init__(world)
        self._test = get_cfree_pose_pose_test(collisions=True)

    def sample(
        self, inputs: Dict[str, Any], fluents: Dict[str, List[Dict[str, Any]]]
    ) -> Optional[Union[Dict[str, Any], coast.StreamConstraint]]:
        b1 = inputs["b1"]
        b2 = inputs["b2"]
        p1: pr2_primitives.Pose = inputs["p1"]
        p2: pr2_primitives.Pose = inputs["p2"]
        does_not_collide = self._test(b1=b1, p1=p1, b2=b2, p2=p2)

        return None if not does_not_collide else {}


class SampleGrasp(coast.Stream):
    name = "SampleGrasp"
    inputs = [Object("?b", "block")]
    outputs = [Object("?g", "grasp")]
    fluents: List[str] = []

    def __init__(self, world: Any):
        super().__init__(world, resample_p=0.8)  # 0.2
        self._sample_grasp = pr2_primitives.get_grasp_gen(
            world.problem, front_only=False
        )

    def sample(
        self, inputs: Dict[str, Any], fluents: Dict[str, List[Dict[str, Any]]]
    ) -> Optional[Union[Dict[str, Any], coast.StreamConstraint]]:
        body: int = inputs["b"]
        grasp: pr2_primitives.Grasp = self._sample_grasp(body=body)[0][0]
        return None if grasp is None else {"g": grasp}


class TestGrasp(coast.Stream):
    name = "TestGrasp"
    inputs = [
        Object("?b1", "block"),
        Object("?p1", "pose"),
        Object("?g", "grasp"),
    ]
    outputs: List[Object] = []

    def __init__(self, world: Any):
        super().__init__(world)
        self._test = get_cfree_grasp_pose_test(world.problem)

    def sample(
        self, inputs: Dict[str, Any], fluents: Dict[str, List[Dict[str, Any]]]
    ) -> Optional[Union[Dict[str, Any], coast.StreamConstraint]]:
        b1 = inputs["b1"]
        p1: pr2_primitives.Pose = inputs["p1"]
        g1: pr2_primitives.Grasp = inputs["g"]
        does_not_collide = self._test(g1=g1, b1=b1, p1=p1)

        return None if not does_not_collide else {}


class InverseKinematics(coast.Stream):
    name = "IK"
    inputs = [Object("?b", "block"), Object("?p", "pose"), Object("?g", "grasp")]
    outputs = [Object("?q", "conf"), Object("?t", "traj")]
    fluents: List[str] = ["AtPose"]

    def __init__(self, world: Any):
        super().__init__(world, resample_p_fail=0.01)  # 0.1 0.2
        self._robot = world.robot
        # self._ab: dyn.ArticulatedBody = world.ab
        self._q_home: np.ndarray = world.q_home
        self._problem = world.problem
        self._custom_limits = world.custom_limits
        self._ik_sampler = pr2_primitives.get_ik_ir_gen(
            problem=self._problem,
            custom_limits=self._custom_limits,
            learned=True,
            constraint=True,
        )
        self.names = world.body_names
        self._sample_grasp = pr2_primitives.get_grasp_gen(
            world.problem, front_only=False
        )

    def sample(
        self, inputs: Dict[str, Any], fluents: Dict[str, List[Dict[str, Any]]]
    ) -> Optional[Union[Dict[str, Any], coast.StreamConstraint]]:
        obj = inputs["b"]
        body_pose: pr2_primitives.Pose = inputs["p"]
        # grasp: pr2_primitives.Grasp = inputs["g"]
        atpose_fluents = [
            ("atpose", args["b"], args["p"])
            for args in fluents["AtPose"]
            if obj != args["b"]
        ]
        poses = [tup[2] for tup in atpose_fluents]
        obstacles = [tup[1] for tup in atpose_fluents]
        # set_renderer(True)
        # wait_for_user()
        for p in poses:
            p.assign()
        # set_renderer(False)
        # wait_for_user(obstacles)
        grasp: pr2_primitives.Grasp = self._sample_grasp(obj, fixed_grasp=inputs["g"])[
            0
        ][0]
        try:
            maybeConfStatusTup = next(
                self._ik_sampler(
                    self._problem.arms[0],
                    obj,
                    body_pose,
                    grasp,
                    self._problem.fixed,
                    obstacles,
                )
            )  # arm, obj, pose, grasp
        except StopIteration:
            print("failed")
            # Constrain the grasp for all bodies
            constraints = ""
            for block in self._problem.movable:
                name = self.names[block]
                if name != self.names[inputs["b"]]:
                    # add the constraint
                    p = f"(collisionblock {self.names[inputs['b']]} {name} grasp{inputs['g']})"
                    constraints += p + " "
            total = f"(and {constraints})"
            return coast.StreamConstraint(total, CfreeConstraint)
            return None
        conf, status, tup = maybeConfStatusTup
        if status:
            return {"q": conf, "t": tup[0]}
        if tup is None:  # Indicates that there's no poses to constrain

            return None
        my_block, obstructing_block = tup
        # # set_renderer(True)
        # # wait_for_user("IK Failed Collision")
        # # set_renderer(False)
        constraint = f"(collisionblock {self.names[inputs['b']]} {self.names[obstructing_block]} grasp{inputs['g']})"
        return coast.StreamConstraint(constraint, CfreeConstraint)


class SampleBMotion(coast.Stream):
    name = "SampleBMotion"
    inputs = [Object("?q1", "conf"), Object("?q2", "conf")]
    outputs = [Object("?bt", "traj")]
    fluents: List[str] = []

    def __init__(self, world: Any):
        super().__init__(world)
        self._sample_bmotion = pr2_primitives.get_motion_gen(
            world.problem, custom_limits=world.custom_limits
        )

    def sample(
        self, inputs: Dict[str, Any], fluents: Dict[str, List[Dict[str, Any]]]
    ) -> Optional[Union[Dict[str, Any], coast.StreamConstraint]]:
        q1 = inputs["q1"]
        q2 = inputs["q2"]
        command = self._sample_bmotion(q1, q2)
        if command is None:
            return None
        return {"bt": command}


class TestCFreeApproachPose(coast.Stream):
    name = "CFreeApproachPose"
    inputs = [
        Object("?g", "grasp"),
        Object("?b1", "block"),
        Object("?p1", "pose"),
        Object("?b2", "block"),
        Object("?p2", "pose"),
    ]
    outputs: List[str] = []
    fluents: List[str] = []

    def __init__(self, world: Any):
        super().__init__(world)
        self._test = get_cfree_approach_pose_test(
            problem=world.problem, collisions=True
        )
        self.names = world.body_names
        self._sample_grasp = pr2_primitives.get_grasp_gen(
            world.problem, front_only=False
        )

    def sample(
        self, inputs: Dict[str, Any], fluents: Dict[str, List[Dict[str, Any]]]
    ) -> Optional[Union[Dict[str, Any], coast.StreamConstraint]]:
        # g1 = inputs["g"]
        g1 = self._sample_grasp(inputs["b1"], fixed_grasp=inputs["g"])[0][0]
        does_not_collide, maybe_obstruction = self._test(
            g1=g1,
            b1=inputs["b1"],
            b2=inputs["b2"],
            p2=inputs["p2"],
            p1=inputs["p1"],
        )
        if does_not_collide:
            return {}
        block1, obstructing_block = maybe_obstruction
        constraint = f"(collisionblock {self.names[inputs['b1']]} {self.names[obstructing_block]} grasp{inputs['g']})"
        return coast.StreamConstraint(constraint, CfreeConstraint)


class TestCFreeTrajPose(coast.Stream):
    name = "CFreeTrajPose"
    inputs = [
        Object("?t", "traj"),
        Object("?b", "block"),
        Object("?p", "pose"),
        Object("?g", "grasp"),
    ]
    outputs: List[str] = []
    fluents: List[str] = []

    def __init__(self, world: Any):
        super().__init__(world)
        self._test = get_cfree_traj_pose_test(robot=world.robot)
        # self._robot = world.robot
        self.names = world.body_names

    def sample(
        self, inputs: Dict[str, Any], fluents: Dict[str, List[Dict[str, Any]]]
    ) -> Optional[Union[Dict[str, Any], coast.StreamConstraint]]:
        command: pr2_primitives.Command = inputs["t"]
        body: int = inputs["b"]
        pose: pr2_primitives.Pose = inputs["p"]
        # config = kuka_primitives.BodyConf(
        #     body=command.body_paths[0].body,
        #     configuration=command.body_paths[0].path[0],
        #     joints=command.body_paths[0].joints,
        # )
        assert pose.body == body

        does_not_collide, maybe_obstruction = self._test(c=command, b2=body, p2=pose)
        if does_not_collide or maybe_obstruction is None:
            return {}

        block, obs = maybe_obstruction
        constraint = (
            f"(collisionblock {self.names[block]} {self.names[obs]} grasp{inputs['g']})"
        )
        return coast.StreamConstraint(constraint, CfreeConstraint)
