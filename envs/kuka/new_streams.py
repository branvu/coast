from typing import Dict, Optional, Any, List, Tuple

import numpy as np
from ctrlutils import eigen
import spatialdyn as dyn

import coast
from coast import Object
from coast.action import Action
from coast.formula import _and, _forall, _not, _P, _when
from pybullet_tools import kuka_primitives, utils as pb_utils


GEOMETRIC_PREDICATES = [
    _P("AtConf", "?q"),
    _P("Holding", "?b", "?g"),
    _P("AtPose", "?b", "?p"),
]


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


class Clean(Action):
    parameters = [Object("?b", "object"), Object("?s", "phys"), Object("?t1",
        "timestep"), Object("?t2", "timestep")]
    inputs = [Object("?q1", "conf"), Object("?p", "pose")]
    outputs: List[Object] = []
    precondition = _and(_P("AtConf", "?q1"), _P("AtPose", "?b", "?p"))
    effect = _and()
    certified = _and()

    def execute(self, inputs: Dict[str, Any], outputs: Dict[str, Any]) -> bool:
        return True


class Cook(Action):
    parameters = [Object("?b", "object"), Object("?s", "phys"), Object("?t1",
        "timestep"), Object("?t2", "timestep")]
    inputs = [Object("?q1", "conf"), Object("?p", "pose")]
    outputs: List[Object] = []
    precondition = _and(_P("AtConf", "?q1"), _P("AtPose", "?b", "?p"))
    effect = _and()
    certified = _and()

    def execute(self, inputs: Dict[str, Any], outputs: Dict[str, Any]) -> bool:
        return True


class Pick(Action):
    parameters = [Object("?b", "object"), Object("?s", "phys"), Object("?t1",
        "timestep"), Object("?t2", "timestep")]
    inputs = [Object("?q1", "conf"), Object("?p", "pose")]
    outputs = [Object("?g", "grasp"), Object("?q2", "conf"), Object("?traj1",
        "traj"), Object("?traj2", "traj")]
    precondition = _and(_P("AtConf", "?q1"), _P("AtPose", "?b", "?p"))
    effect = _and(
        _not(_P("AtPose", "?b", "?p")),
        _P("Holding", "?b", "?g"),
        _not(_P("AtConf", "?q1")),
        _P("AtConf", "?q2"),
    )
    certified = _and(
        _P("SampleGrasp", "?b", "?g"),
        _P("IK", "?b", "?p", "?g", "?q2", "?traj2"),
        _forall(
            "bb",
            "block",
            _forall(
                "pp",
                "pose",
                _when(
                    _P("AtPose", "?bb", "?pp"),
                    _P("CFreeTrajPose", "?traj2", "?bb", "?pp"),
                ),
            ),
        ),
        _P("FreeMotion", "?q1", "?q2", "?traj1")
    )

    def execute(self, inputs: Dict[str, Any], outputs: Dict[str, Any]) -> bool:
        traj1: kuka_primitives.Command = outputs["traj1"]
        traj1.refine(num_steps=60).execute(time_step=0.0001)
        traj2: List[kuka_primitives.Command] = outputs["traj2"]
        traj2.refine(num_steps=60).execute(time_step=0.0001)
        return True


class Place(Action):
    parameters = [Object("?b", "object"), Object("?s", "phys"), Object("?t1",
        "timestep"), Object("?t2", "timestep")]
    inputs = [Object("?q1", "conf"), Object("?g", "grasp")]
    outputs = [Object("?p", "pose"), Object("?q2", "conf"), Object("?traj1",
        "traj"), Object("?traj2", "traj")]
    precondition = _and(_P("AtConf", "?q1"), _P("Holding", "?b", "?g"))
    effect = _and(
        _not(_P("Holding", "?b", "?g")),
        _P("AtPose", "?b", "?p"),
        _not(_P("AtConf", "?q1")),
        _P("AtConf", "?q2"),
    )
    certified = _and(
        _P("SamplePose", "?b", "?s", "?p"),
        _P("IK", "?b", "?p", "?g", "?q2", "?traj2"),
        _forall(
            "bb",
            "block",
            _forall(
                "pp",
                "pose",
                _when(
                    _P("AtPose", "?bb", "?pp"),
                    _P("CFreeTrajPose", "?traj2", "?bb", "?pp"),
                ),
            ),
        ),
        _P("HoldingMotion", "?b", "?g", "?q1", "?q2", "?traj1")
    )

    def execute(self, inputs: Dict[str, Any], outputs: Dict[str, Any]) -> bool:
        traj1: kuka_primitives.Command = outputs["traj1"]
        traj1.refine(num_steps=60).execute(time_step=0.0001)
        traj2: List[kuka_primitives.Command] = outputs["traj2"]
        traj2.reverse().refine(num_steps=60).execute(time_step=0.0001)
        return True


class SamplePose(coast.Stream):
    name = "SamplePose"
    inputs = [Object("?b", "object"), Object("?s", "phys")]
    outputs = [Object("?p", "pose")]
    fluents: List[str] = []

    def __init__(self, world: Any):
        super().__init__(world, resample_p=0.6) # 0.7
        self._sample_pose = kuka_primitives.get_stable_gen(fixed=world.fixed_bodies)

    def sample(
        self, inputs: Dict[str, Any], fluents: Dict[str, List[Dict[str, Any]]]
    ) -> Optional[Dict[str, Any]]:
        # pos, quat = inputs["loc"]
        # pose = kuka_primitives.BodyPose(
        #     body=inputs["b"], pose=pb_utils.Pose(pb_utils.Point(*pos))
        # )
        #
        # return {"p": pose}

        pose: kuka_primitives.BodyPose = next(
            self._sample_pose(body=inputs["b"], surface=inputs["s"])
        )
        return None if pose is None else {"p": pose[0]}


class SampleGrasp(coast.Stream):
    name = "SampleGrasp"
    inputs = [Object("?b", "object")]
    outputs = [Object("?g", "grasp")]
    fluents: List[str] = []

    def __init__(self, world: Any):
        super().__init__(world)
        self._sample_grasp = kuka_primitives.get_grasp_gen(
            robot=world.robot, grasp_name="top"
        )

    def sample(
        self, inputs: Dict[str, Any], fluents: Dict[str, List[Dict[str, Any]]]
    ) -> Optional[Dict[str, Any]]:
        body: int = inputs["b"]
        # TODO: save state to avoid grasp generator reset?
        grasp: kuka_primitives.BodyGrasp = next(self._sample_grasp(body=body))
        return None if grasp is None else {"g": grasp[0]}


class FreeMotion(coast.Stream):
    name = "FreeMotion"
    inputs = [Object("?q1", "conf"), Object("?q2", "conf")]
    outputs = [Object("?traj1", "traj")]
    fluents: List[str] = ["AtPose"]

    def __init__(self, world: Any):
        super().__init__(world)
        self._robot = world.robot
        self._ab: dyn.ArticulatedBody = world.ab
        self._q_home: np.ndarray = world.q_home
        self._sampler = kuka_primitives.get_free_motion_gen(self._robot, world.fixed_bodies) 

    def sample(
        self, inputs: Dict[str, Any], fluents: Dict[str, List[Dict[str, Any]]]
    ) -> Optional[Dict[str, Any]]:
        q1 = inputs["q1"]
        q2 = inputs["q2"]
        atpose_fluents = [("atpose", args["b"], args["p"]) for args in fluents["AtPose"]]
        traj = self._sampler(q1, q2, fluents=atpose_fluents)
        if traj is not None:
            return {"traj1": traj[0]}
        return None


class HoldingMotion(coast.Stream):
    name = "HoldingMotion"
    inputs = [Object("?b", "object"), Object("?g", "grasp"), Object("?q1", "conf"), Object("?q2", "conf")]
    outputs = [Object("?traj1", "traj")]
    fluents: List[str] = ["AtPose"]

    def __init__(self, world: Any):
        super().__init__(world)
        self._robot = world.robot
        self._ab: dyn.ArticulatedBody = world.ab
        self._q_home: np.ndarray = world.q_home
        self._sampler = kuka_primitives.get_holding_motion_gen(self._robot, world.fixed_bodies) 

    def sample(
        self, inputs: Dict[str, Any], fluents: Dict[str, List[Dict[str, Any]]]
    ) -> Optional[Dict[str, Any]]:
        q1 = inputs["q1"]
        q2 = inputs["q2"]
        b = inputs["b"]
        g = inputs["g"]
        atpose_fluents = [("atpose", args["b"], args["p"]) for args in fluents["AtPose"]]
        traj = self._sampler(q1, q2, b, g, fluents=atpose_fluents)
        if traj is not None:
            return {"traj1": traj[0]}
        return None


class InverseKinematics(coast.Stream):
    name = "IK"
    inputs = [Object("?b", "object"), Object("?p", "pose"), Object("?g", "grasp")]
    outputs = [Object("?q", "conf"), Object("?traj2", "traj")]
    fluents: List[str] = []

    def __init__(self, world: Any):
        super().__init__(world)
        self._robot = world.robot
        self._ab: dyn.ArticulatedBody = world.ab
        self._q_home: np.ndarray = world.q_home
        self.world = world

    def sample(
        self, inputs: Dict[str, Any], fluents: Dict[str, List[Dict[str, Any]]]
    ) -> Optional[Dict[str, Any]]:
        body_pose: kuka_primitives.BodyPose = inputs["p"]
        grasp: kuka_primitives.BodyGrasp = inputs["g"]

        grasp_pose = pb_utils.end_effector_from_body(body_pose.pose, grasp.grasp_pose)
        approach_pose = pb_utils.approach_from_grasp(grasp.approach_pose, grasp_pose)

        self._ab.q = self._q_home
        q_approach = compute_opspace_ik(self._ab, approach_pose)
        if np.isnan(q_approach).any():
            return None

        self._ab.q = q_approach
        q_grasp = compute_opspace_ik(self._ab, grasp_pose)
        if np.isnan(q_grasp).any():
            return None

        conf = kuka_primitives.BodyConf(self._robot, q_approach)
        conf.assign()
        path = pb_utils.plan_direct_joint_motion(
            self._robot, conf.joints, q_grasp, obstacles=[]
        )
        if path is None:
            return None

        traj = kuka_primitives.Command(
            [
                kuka_primitives.BodyPath(self._robot, path),
                kuka_primitives.Attach(object, self._robot, grasp.link),
                kuka_primitives.BodyPath(self._robot, path[::-1], attachments=[grasp]),
            ]
        )

        return {"q": conf, "traj2": traj}


# class TestCFreePosePose(coast.Stream):
#     name = "CFreePosePose"
#     inputs = [
#         Object("?b1", "block"),
#         Object("?p1", "pose"),
#         Object("?b2", "block"),
#         Object("?p2", "pose"),
#     ]
#     outputs: List[str] = []
#
#     def __init__(self, world: Any):
#         super().__init__(world)
#         self._test = pddlstream_utils.get_cfree_pose_pose_test()
#
#     def sample(self, inputs: Dict[str, Any]) -> Optional[Dict[str, Any]]:
#         return {} if not self._test(**inputs) else None
#
#
# class TestCFreeApproachPose(coast.Stream):
#     name = "CFreeApproachPose"
#     inputs = [
#         Object("?b1", "block"),
#         Object("?p1", "pose"),
#         Object("?g1", "grasp"),
#         Object("?b2", "block"),
#         Object("?p2", "pose"),
#     ]
#     outputs: List[str] = []
#
#     def __init__(self, world: Any):
#         super().__init__(world)
#         self._test = pddlstream_utils.get_cfree_obj_approach_pose_test()
#
#     def sample(self, inputs: Dict[str, Any]) -> Optional[Dict[str, Any]]:
#         return {} if not self._test(**inputs) else None


class TestCFreeTrajPose(coast.Stream):
    name = "CFreeTrajPose"
    inputs = [Object("?t", "traj"), Object("?b", "object"), Object("?p", "pose")]
    outputs: List[str] = []
    fluents: List[str] = []

    def __init__(self, world: Any):
        super().__init__(world)
        self._test = kuka_primitives.get_movable_collision_test()
        # self._robot = world.robot

    def sample(
        self, inputs: Dict[str, Any], fluents: Dict[str, List[Dict[str, Any]]]
    ) -> Optional[Dict[str, Any]]:
        command: kuka_primitives.Command = inputs["t"]
        body: int = inputs["b"]
        pose: kuka_primitives.BodyPose = inputs["p"]
        # config = kuka_primitives.BodyConf(
        #     body=command.body_paths[0].body,
        #     configuration=command.body_paths[0].path[0],
        #     joints=command.body_paths[0].joints,
        # )
        assert pose.body == body

        does_collide = self._test(command=command, body=body, pose=pose)

        return None if does_collide else {}
