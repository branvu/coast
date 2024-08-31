#!/usr/bin/env python3

import argparse
import csv
import dataclasses
import pathlib
import pickle
import random
import time
from typing import Dict, List, Optional, Set, Tuple, Any
import os
import time
from collections import OrderedDict
import numpy as np
import pybullet_tools as pbt
from pybullet_tools.pr2_primitives import Conf
from pybullet_tools.utils import (
    draw_base_limits,
    add_data_path,
    load_pybullet,
    set_point,
    Point,
    create_box,
    stable_z,
    HUSKY_URDF,
    load_model,
    TURTLEBOT_URDF,
    joints_from_names,
    set_joint_positions,
    get_joint_positions,
    joint_from_name,
    link_from_name,
    get_link_pose,
    draw_pose,
    wait_for_user,
    get_center_extent,
    draw_aabb,
    get_aabb,
    HideOutput,
    GREY,
    BLACK,
    RED,
    BLUE,
    BROWN,
    TAN,
    GREEN,
    get_bodies,
    set_pose
)
# from pybullet_tools.pr2_problems import (
#     sample_placements,
# )
import spatialdyn as dyn
import symbolic

import coast

from utils import BeliefState
BASE_JOINTS = ["x", "y", "theta"]


def get_base_joints(robot):
    return joints_from_names(robot, BASE_JOINTS)


def get_base_conf(robot):
    return get_joint_positions(robot, get_base_joints(robot))


def set_base_conf(robot, conf):
    set_joint_positions(robot, get_base_joints(robot), conf)


def get_custom_limits(robot, base_limits, yaw_limit=None):
    x_limits, y_limits = zip(*base_limits)
    custom_limits = {
        joint_from_name(robot, "x"): x_limits,
        joint_from_name(robot, "y"): y_limits,
    }
    if yaw_limit is not None:
        custom_limits.update(
            {
                joint_from_name(robot, "theta"): yaw_limit,
            }
        )
    return custom_limits


class RoversProblem(object):
    def __init__(
        self,
        rovers=[],
        rover_names=[],
        landers=[],
        objectives=[],
        rocks=[],
        soils=[],
        stores=[],
        limits=[],
        body_types=[],
        rock_names=[],
        objective_names=[],
        init_confs=[]
    ):
        self.rovers = rovers
        self.rover_names = rover_names
        self.rock_names = rock_names
        self.landers = landers
        self.objectives = objectives
        self.rocks = rocks
        self.soils = soils
        self.stores = stores
        self.limits = limits
        no_collisions = self.rovers + self.rocks
        self.fixed = set(get_bodies()) - set(no_collisions)
        self.body_types = body_types
        self.costs = False
        self.objective_names = objective_names
        self.init_confs = init_confs
@dataclasses.dataclass
class World:
    init_rover_confs: List
    problem: RoversProblem
    custom_limits: Any
    new_state: Set[str]
    goal_state: Set[str]


def load_world(
    n_objectives: int = 1, n_rocks: int = 0, n_obstacles: int = 8, saved_poses: Dict[str, List[Any]] = None
) -> World:
    # TODO: store internal world info here to be reloaded
    pbt.utils.set_default_camera()
    pbt.utils.draw_global_system()

    n_rovers = 2
    n_soil = 0
    n_stores = 1

    base_extent = 5.0
    base_limits = (-base_extent / 2.0 * np.ones(2), base_extent / 2.0 * np.ones(2))
    mount_width = 0.5
    mound_height = 0.1

    floor = create_box(base_extent, base_extent, 0.001, color=TAN)  # TODO: two rooms
    set_point(floor, Point(z=-0.001 / 2.0))

    wall1 = create_box(
        base_extent + mound_height, mound_height, mound_height, color=GREY
    )
    set_point(wall1, Point(y=base_extent / 2.0, z=mound_height / 2.0))
    wall2 = create_box(
        base_extent + mound_height, mound_height, mound_height, color=GREY
    )
    set_point(wall2, Point(y=-base_extent / 2.0, z=mound_height / 2.0))
    wall3 = create_box(
        mound_height, base_extent + mound_height, mound_height, color=GREY
    )
    set_point(wall3, Point(x=base_extent / 2.0, z=mound_height / 2.0))
    wall4 = create_box(
        mound_height, base_extent + mound_height, mound_height, color=GREY
    )
    set_point(wall4, Point(x=-base_extent / 2.0, z=mound_height / 2.0))
    # TODO: can add obstacles along the wall

    wall = create_box(
        mound_height, base_extent, mound_height, color=GREY
    )  # TODO: two rooms
    set_point(wall, Point(z=mound_height / 2.0))

    add_data_path()
    with HideOutput():
        lander = load_pybullet(HUSKY_URDF, scale=1)
    lander_z = stable_z(lander, floor)
    set_point(lander, Point(-1.9, -2, lander_z))

    mound1 = create_box(mount_width, mount_width, mound_height, color=GREY)
    set_point(mound1, [+2, 2, mound_height / 2.0])
    mound2 = create_box(mount_width, mount_width, mound_height, color=GREY)
    set_point(mound2, [-2, 2, mound_height / 2.0])
    mound3 = create_box(mount_width, mount_width, mound_height, color=GREY)
    set_point(mound3, [+0.5, 2, mound_height / 2.0])
    mound4 = create_box(mount_width, mount_width, mound_height, color=GREY)
    set_point(mound4, [-0.5, 2, mound_height / 2.0])
    mounds = [mound1, mound2, mound3, mound4]
    # random.shuffle(mounds)

    body_types = []
    initial_surfaces = OrderedDict()
    min_distances = {}
    obstacles = []
    for _ in range(n_obstacles):
        body = create_box(mound_height, mound_height, 4 * mound_height, color=GREY)
        initial_surfaces[body] = floor
        obstacles.append(body)

    rover_confs = [(+1, -1.75, np.pi), (-1, -1.75, 0)]
    assert n_rovers <= len(rover_confs)

    landers = [lander]
    stores = ["store{}".format(i) for i in range(n_stores)]

    rovers = []
    rover_names = []
    for i in range(n_rovers):
        # camera_rgb_optical_frame
        with HideOutput():
            rover = load_model(TURTLEBOT_URDF)
        robot_z = stable_z(rover, floor)
        set_point(rover, Point(z=robot_z))
        # handles = draw_aabb(get_aabb(rover)) # Includes the origin
        # print(get_center_extent(rover))
        # wait_for_user()
        # TODO Save base conf
        set_base_conf(rover, rover_confs[i])
        rovers.append(rover)
        rover_names.append(f"rv{i + 1}")
        # dump_body(rover)
        # draw_pose(get_link_pose(rover, link_from_name(rover, KINECT_FRAME)))

    obj_width = 0.07
    obj_height = 0.2

    objectives = []
    objective_names = []
    for i in range(n_objectives):
        body = create_box(obj_width, obj_width, obj_height, color=BLUE)
        objectives.append(body)
        # initial_surfaces[body] = random.choice(mounds)
        initial_surfaces[body] = mounds[i % len(mounds)]
        objective_names.append(f"o{i + 1}")
    min_distances.update({r: 0.05 for r in objectives})

    # TODO: it is moving to intermediate locations attempting to reach the rocks

    rocks = []
    rock_names = []
    for i in range(n_rocks):
        body = create_box(0.075, 0.075, 0.01, color=BLACK)
        rocks.append(body)
        rock_names.append(f"r{i + 1}")
        body_types.append((body, "stone"))
    for _ in range(n_soil):
        body = create_box(0.1, 0.1, 0.005, color=BROWN)
        rocks.append(body)
        body_types.append((body, "soil"))
    soils = []  # Treating soil as rocks for simplicity

    initial_surfaces.update({r: floor for r in rocks})
    min_distances.update({r: 0.2 for r in rocks})
    
    if saved_poses is None:
        sample_placements(
            initial_surfaces, min_distances=min_distances
        ) 
    else:
        for i in range(n_rocks):
            set_pose(rocks[i], saved_poses["rocks"][i])
        for i in range(n_objectives):
            set_pose(objectives[i], saved_poses["objectives"][i])
        for i in range(n_obstacles):
            set_pose(obstacles[i], saved_poses["obstacles"][i])
        for i in range(n_soil):
            set_pose(soils[i], saved_poses["soils"])

    for i in range(n_rocks):
        pbt.utils.add_text(
                f"r{i + 1}", position=tuple(pbt.utils.get_pose(rocks[i])[0])
        )
    for i in range(n_objectives):
        pbt.utils.add_text(
                f"o{i + 1}", position=tuple(pbt.utils.get_pose(objectives[i])[0])
        )

    # New intial symbolic state
    new_state = set(
        [
            "(Supports c1 rgbd)",
            "(Supports c2 rgbd)",
            "(OnBoard c1 rv1)",
            "(OnBoard c2 rv2)",
            "(Free rv1 s1)",
            "(Free rv2 s2)",
        ]
    )
    # Init where the blocks are on what tables
    # for t in range(29):
    #     new_state.add(f"(Next t{t + 1} t{t + 2})")
    # new_state.add("(AtTimestep t1)")

    problem = RoversProblem(
        rovers,
        rover_names,
        landers,
        objectives,
        rocks,
        soils,
        stores,
        base_limits,
        body_types,
        rock_names,
        objective_names,
        rover_confs
    )

    # wait_for_user()
    custom_limits = {}
    for rover in problem.rovers:
        custom_limits.update(get_custom_limits(rover, problem.limits))
    draw_base_limits(problem.limits, color=RED, width=10)
    goal_state = set()
    goal_state.add("(Free rv1 s1)")
    goal_state.add("(Free rv2 s2)")
    for i in range(1, n_objectives + 1):
        goal_state.add(f"(ReceivedImage o{i} rgbd)")
    for i in range(1, len(rocks) + 1):
        goal_state.add(f"(ReceivedAnalysis r{i})")
    # wait_for_user(f"Setup World with limits: {custom_limits}")
    return World(rover_confs, problem, custom_limits, new_state, goal_state)


def init_geometric_state(
    n_objectives: int, n_rocks: int = 0, saved_poses: Dict[str, List[Any]] = None
) -> Tuple[World, str, coast.GeometricState, List[coast.Object]]:
    pbt.utils.connect(use_gui=True)
    world = load_world(n_objectives, n_rocks, saved_poses=saved_poses)
    state = coast.GeometricState()
    objects: List[coast.Object] = []
    objects.append(coast.Object(name="lander1", object_type="lander",
        value=world.problem.landers[0]))
    for i in range(len(world.problem.objective_names)):
        objects.append(
            coast.Object(
                name=world.problem.objective_names[i],
                object_type="objective",
                value=world.problem.objectives[i],
            )
        )

    for i in range(len(world.problem.rover_names)):
        objects.append(
            coast.Object(
                name=world.problem.rover_names[i],
                object_type="rover",
                value=world.problem.rovers[i],
            )
        )
    for i in range(len(world.problem.rock_names)):
        objects.append(
            coast.Object(
                name=world.problem.rock_names[i],
                object_type="rock",
                value=world.problem.rocks[i],
            )
        )

    stream_state: Set[str] = set()
    # state.set(
    #     "config",
    #     0,
    #     (
    #         world.robot,
    #         pbt.pr2_primitives.Conf(
    #             world.robot, pbt.utils.get_movable_joints(world.robot)
    #         ),
    #     ),
    # )
    # state.set("commands", 0, [])
    stream_state.add("AtConf(rv1, v1q0)")
    stream_state.add("AtConf(rv2, v2q0)")
    rq0 = Conf(
        world.problem.rovers[0],
        get_base_joints(world.problem.rovers[0]),
        world.init_rover_confs[0],
    )
    rq0.assign()
    rq1 = Conf(
        world.problem.rovers[1],
        get_base_joints(world.problem.rovers[1]),
        world.init_rover_confs[1],
    )
    rq1.assign()
    objects.append(
        coast.Object(
            "v1q0",
            "conf",
            rq0,
        )
    )
    pbt.utils.add_text("rv1", position=(*rq0.values[:2], 0))
    pbt.utils.add_text("rv2", position=(*rq1.values[:2], 0))
    objects.append(
        coast.Object(
            "v2q0",
            "conf",
            rq1,
        )
    )

    problem = create_problem(world)
    return world, problem, stream_state, objects

def create_problem(world: World) -> str:
    from symbolic import _and

    problem = symbolic.Problem("rovers_problem", domain="rovers")
    for prop in world.new_state:
        problem.add_initial_prop(prop)
    goal = _and(list(world.goal_state))
    problem.set_goal(goal)
    print(problem)

    return repr(problem)

def num_to_constants(n: int):
    res = "constants"
    res += "\nrv1 - rover\n"
    res += "rv2 - rover\n"
    res += "c1 - camera\n"
    res += "c2 - camera\n"
    res += "s1 - store\n"
    res += "s2 - store\n"
    res += "lander1 - lander\n"
    res += "rgbd - mode\n"
    for i in range(1, n + 1):
        res += f"r{i} - rock\n"
        res += f"o{i} - objective\n"
    res += ")"
    return res
def main(
    domain_pddl: str,
    stream_pddl, 
    streams_py: str,
    algorithm: str,
    max_level: int,
    search_sample_ratio: float,
    timeout: int,
    experiment: int,
    write_experiment: str,
    random_seed: int,
):
    saved_poses: Dict[Tuple[int, int], Dict[str, List[Any]]] = {}
    pickle_dir = "pickles/0727_rover_pickles_incr_obj_rock"# rover_pickles_incr_obj_rock
    for pkl_name in os.listdir(pickle_dir):
        f = os.path.join(pickle_dir, pkl_name)
        with open(f, "rb") as fp:
            aPickle = pickle.load(fp)
            for key in aPickle:
                if key not in saved_poses:
                    saved_poses[key] = aPickle[key]
    for n in range(4, 5):
        for e in range(experiment):
            edit_domain_pddl = coast.utils.replace_domain_constants(domain_pddl, num_to_constants(n))
            n_objectives = n
            poses = saved_poses[(n_objectives, e)]
            if random_seed is not None:
                np.random.seed(int(random_seed))
                random.seed(int(random_seed))
                print("random_seed args:", random_seed)
            else:
                print("random_seed:", e)
                np.random.seed(e)
                random.seed(e)

            (   
                world,
                problem_pddl,
                stream_state,
                objects,
            ) = init_geometric_state(n_objectives, n, poses)
            # wait_for_user()
            saver = pbt.utils.WorldSaver()
            pbt.utils.set_renderer(enable=False) 
            plan = coast.plan(
                streams_pddl=stream_pddl,
                domain_pddl=edit_domain_pddl,
                problem_pddl=problem_pddl,
                streams_py=streams_py,
                algorithm=algorithm,
                max_level=max_level,
                search_sample_ratio=search_sample_ratio,
                timeout=timeout,
                experiment=experiment,
                stream_state=stream_state,
                objects=objects,
                random_seed=random_seed,
                world=world,
                constraint_cls=coast.constraint.ParameterConstraint,
                use_cache=False,
                sort_streams=False
            )
            # if plan.action_plan is None or plan.objects is None:
            #     pbt.utils.disconnect()
            #     return
            pbt.utils.disconnect()
            (   
                world,
                problem_pddl,
                stream_state,
                objects,
            ) = init_geometric_state(n_objectives, n, poses)
            pbt.utils.set_renderer(enable=True)
            saver.restore()
            if experiment > 1:
                with open(write_experiment, "a", newline="") as f_object:
                    res = plan.log.get_timing()
                    writer_csv = csv.DictWriter(
                        f_object,
                        fieldnames=list(res.keys()) + ["total", "increment", "solved", "n_success_plans",
                            "n_fail_plans"],
                    )
                    res["increment"] = n_objectives
                    res["solved"] = plan.action_plan is not None
                    res["n_success_plans"] = plan.log.num_success_task_plans
                    res["n_fail_plans"] = plan.log.num_fail_task_plans
                    writer_csv.writerow(res)
                    f_object.close()
                    # writer_csv = csv.DictWriter(
                    #     f_object, fieldnames=list(plan.log.keys())
                    # )
                    # # writer_csv.writerow(total_loggings.keys())
                    # writer_csv.writerow(plan.log.collect_profiles())
                    # f_object.close()
            if experiment == 1:
                pbt.utils.wait_for_user("Execute?")
                py_state = BeliefState(world.problem)
                time.sleep(3) 
                for action in plan.action_plan:
                    # wait_for_user(f"Executing {action}")
                    action.execute_state(
                        py_state,
                        world.problem,
                        None,
                        None,
                        plan.objects,
                    )
            pbt.utils.disconnect()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(add_help=False)
    parser.add_argument(
        "--stream-pddl",
        "--stream",
        nargs="?",
        default="envs/rover/domain.pddl",
        help="Pddl stream file.",
    )
    parser.add_argument(
        "--domain-pddl",
        "--domain",
        nargs="?",
        default="envs/rover/domain-no-time.pddl",
        help="Pddl domain file.",
    )
    parser.add_argument(
        "--streams-py",
        nargs="?",
        default="envs/rover/streams-no-time.py",
        help="Pddl streams file.",
    )
    parser.add_argument(
        "--algorithm",
        nargs="?",
        default="improved",
        help="Max planning level.",
    )
    parser.add_argument(
        "--max-level",
        nargs="?",
        type=int,
        default=6,
        help="Max planning level.",
    )
    parser.add_argument(
        "--search-sample-ratio",
        type=float,
        default=1,#0.1,  # 2 35 1.0 = don't try to resample.
        help="The ratio of search versus sample",
    )
    parser.add_argument(
        "--timeout", type=int, default=1200, help="Timeout for algorithm"
    )
    parser.add_argument(
        "--experiment",
        type=int,
        default=1,
        help="The number of times you want to run the experiment",
    )
    parser.add_argument(
        "--write-experiment",
        type=str,
        default="experiments/0820_rover_ours_ff_sort_w_cache.csv",
        help="The file to write the csv of experiment",
    )
    parser.add_argument("--random-seed", type=int, default=None, help="Random seed")
    args = parser.parse_args()
    print(args)
    try:
        main(**vars(args))
    except Exception as e:
        pbt.utils.disconnect()
        raise e
