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
import numpy as np
import pybullet_tools as pbt
import pybullet_tools.pr2_problems
import pybullet_tools.pr2_primitives
from pybullet_tools.pr2_problems import (
    create_pr2,
    create_table,
    sample_placements,
    Problem,
)
from pybullet_tools.pr2_utils import (
    get_other_arm,
    get_carry_conf,
    set_arm_conf,
    open_arm,
    arm_conf,
    REST_LEFT_ARM,
    close_arm,
    set_group_conf,
)
from pybullet_tools.utils import (
    get_bodies,
    add_data_path,
    load_pybullet,
    set_point,
    Point,
    create_box,
    stable_z,
    joint_from_name,
    get_point,
    wait_for_user,
    set_pose,
    RED,
    GREEN,
    BLUE,
    BLACK,
    WHITE,
    BROWN,
    TAN,
    GREY,
)
import spatialdyn as dyn
import symbolic

import coast
from constraints import Constraint


@dataclasses.dataclass
class World:
    robot: int
    problem: Problem
    custom_limits: Any
    q_home: np.ndarray
    body_names: Dict[int, str]
    movable_bodies: List[int]
    fixed_bodies: List[int]
    new_state: Set[str]
    goal_state: Set[str]


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


def load_world(num_blocks: int = 5, saved_poses: Dict[str, List[Any]] = None) -> World:
    # TODO: store internal world info here to be reloaded
    pbt.utils.set_default_camera()
    pbt.utils.draw_global_system()

    with pbt.utils.HideOutput():
        arm = "left"
        grasp_type = "side"
        block_width = 0.04
        block_height = 0.2
        base_limits = (-2.5 * np.ones(2), 2.5 * np.ones(2))
        other_arm = get_other_arm(arm)
        initial_conf = get_carry_conf(arm, grasp_type)
        pbt.utils.draw_base_limits(base_limits, color=(1, 0.5, 0))
        add_data_path()
        floor = load_pybullet("plane.urdf")
        robot = create_pr2()
        set_arm_conf(robot, arm, initial_conf)
        open_arm(robot, arm)
        set_arm_conf(robot, other_arm, arm_conf(other_arm, REST_LEFT_ARM))
        close_arm(robot, other_arm)
        set_group_conf(
            robot, "base", [-1.0, 0, 0]
        )  # Be careful to not set the pr2's pose
        custom_limits = get_custom_limits(robot, base_limits)
        table1 = create_table(dx=2.5, theta=0, length=1.5, height=0.7)
        table2 = create_table(dx=-2.5, theta=0, length=1.5, height=0.7)
        table3 = create_table(dy=2.5, theta=np.pi / 2, length=1.5, height=0.7)
        table4 = create_table(dy=-2.5, theta=np.pi / 2, length=1.5, height=0.7)

        body_names = {
            table1: "btable",
            table2: "ftable",
            table3: "rtable",
            table4: "ltable",
        }
        print("Table IDs", body_names)
        surfaces = [table1, table2, table3, table4]

        blue_blocks = [
            create_box(block_width, block_width, block_height, color=BLUE)
            for _ in range(num_blocks)
        ]
        green_blocks = [
            create_box(block_width, block_width, block_height, color=GREEN)
            for _ in range(num_blocks)
        ]
        red_blocks = [
            create_box(block_width, block_width, block_height, color=RED)
            for _ in range(2 * num_blocks)
        ]
        blocks = blue_blocks + green_blocks + red_blocks
        half_blocks = random.sample(blocks, len(blocks) // 2)
        initial_surfaces = {block: table1 for block in half_blocks}
        for b in blocks:
            if b not in half_blocks:
                initial_surfaces[b] = table2

        min_distances = {block: 0.05 for block in blocks}
        if saved_poses is None:
            sample_placements(initial_surfaces, min_distances=min_distances)
        else:
            for i in range(len(blue_blocks)):
                set_pose(blue_blocks[i], saved_poses["blue"][i])
            for i in range(len(red_blocks)):
                set_pose(red_blocks[i], saved_poses["red"][i])
            for i in range(len(green_blocks)):
                set_pose(green_blocks[i], saved_poses["green"][i])

        goal_state = set()
        for i in range(1, len(blue_blocks) + 1):
            body_names[blue_blocks[i - 1]] = f"b{i}"
            p = pbt.utils.get_pose(blue_blocks[i - 1])[0]
            pbt.utils.add_text(f"b{i}", position=tuple(p))
            goal_state.add(
                f"(ontable {body_names[blue_blocks[i - 1]]} {body_names[table3]})"
            )
        for i in range(1, 1 + len(green_blocks)):
            body_names[green_blocks[i - 1]] = f"b{i + len(blue_blocks)}"
            p = pbt.utils.get_pose(green_blocks[i - 1])[0]
            pbt.utils.add_text(f"b{i + len(blue_blocks)}", position=tuple(p))
            goal_state.add(
                f"(ontable {body_names[green_blocks[i - 1]]} {body_names[table4]})"
            )
        for i in range(1, 1 + len(red_blocks)):
            p = pbt.utils.get_pose(red_blocks[i - 1])[0]
            pbt.utils.add_text(
                f"b{i + len(blue_blocks) + len(green_blocks)}", position=tuple(p)
            )
            body_names[
                red_blocks[i - 1]
            ] = f"b{i + len(blue_blocks) + len(green_blocks)}"
        print(
            str(
                pathlib.Path("external/pybullet-planning/pybullet-planning.git")
                / pbt.pr2_utils.DRAKE_PR2_URDF
            )
        )
        """ ab = dyn.urdf.load_model(
            str(
                pathlib.Path("external/pybullet-planning/pybullet-planning.git")
                / pbt.pr2_utils.DRAKE_PR2_URDF
            )
        )
        """
        ab = None
    # New intial symbolic state
    new_state = set(["(handempty)"])
    # Init where the blocks are on what tables
    for b in blocks:
        block_name = body_names[b]
        table_name = body_names[initial_surfaces[b]]
        new_state.add(f"(ontable {block_name} {table_name})")
    for t in range(21):
        new_state.add(f"(Next t{t + 1} t{t + 2})")
    new_state.add("(AtTimestep t1)")

    fixed_bodies = surfaces
    movable_bodies = blocks
    # wait_for_user()
    q_home = np.array([0.0, np.pi / 6, 0.0, -np.pi / 3, 0.0, np.pi / 2, 0.0])

    problem = Problem(
        robot=robot,
        movable=blocks,
        arms=[arm],
        grasp_types=[grasp_type],
        surfaces=surfaces,
        costs=True,
    )
    return World(
        robot,
        problem,
        custom_limits,
        q_home,
        body_names,
        movable_bodies,
        fixed_bodies,
        new_state,
        goal_state,
    )


def init_geometric_state(
    num_blocks: int, saved_poses: Dict[str, List[Any]] = None
) -> Tuple[World, str, coast.GeometricState, Set[str], List[coast.Object],]:
    pbt.utils.connect(use_gui=True)
    world = load_world(num_blocks, saved_poses=saved_poses)
    state = coast.GeometricState()
    objects: List[coast.Object] = [
        coast.Object(name=body_name, object_type="block", value=body_id)
        for body_id, body_name in world.body_names.items()
        if body_name[0] == "b"
    ]
    objects.extend(
        [
            coast.Object(name=body_name, object_type="table", value=body_id)
            for body_id, body_name in world.body_names.items()
            if body_name[1] == "t"
        ]
    )
    stream_state: Set[str] = set()
    state.set(
        "config",
        0,
        (
            world.robot,
            pbt.pr2_primitives.Conf(
                world.robot, pbt.utils.get_movable_joints(world.robot)
            ),
        ),
    )
    state.set("cur_grasp", 0, None)
    state.set("commands", 0, [])
    stream_state.add("AtConf(q0)")
    objects.append(
        coast.Object(
            "q0",
            "conf",
            pbt.pr2_primitives.Conf(
                world.robot, pbt.utils.get_movable_joints(world.robot)
            ),
        )
    )

    for body_id in world.movable_bodies:
        body_pose = pbt.pr2_primitives.Pose(body_id, pbt.utils.get_pose(body_id))
        state.set(world.body_names[body_id], 0, (body_id, body_pose))

        stream_state.add(f"AtPose({world.body_names[body_id]}, p0_{body_id})")
        objects.append(coast.Object(f"p0_{body_id}", "pose", body_pose))
    for i in range(1, 5):
        objects.append(coast.Object(f"grasp{i}", "grasp", i))
    problem = create_problem(world, num_blocks)
    return world, problem, state, stream_state, objects


def update_goal(problem_pddl: str, num_blocks: int) -> None:
    # Change the goal to match the number of blocks
    with open(problem_pddl, "r") as f:
        problem_str = f.read()

    with open(problem_pddl, "w") as f:
        f.write(f"{problem_str[:problem_str.find('(:goal')]}")
        f.write(f"(:goal (ontable b{num_blocks} loc5))\n")
        f.write(")")


def create_problem(world: World, num_blocks: int) -> str:
    from symbolic import _and

    problem = symbolic.Problem(f"blocks-{num_blocks}", domain="sorting")
    for prop in world.new_state:
        problem.add_initial_prop(prop)
    goal = _and(list(world.goal_state))
    problem.set_goal(goal)
    print(problem)

    return repr(problem)


def main(
    domain_pddl: str,
    problem_pddl: str,
    streams_pddl: str,
    streams_py: str,
    algorithm: str,
    max_level: int,
    pybullet: bool,
    search_sample_ratio: float,
    timeout: int,
    experiment: int,
    write_experiment: str,
    random_seed: int,
):
    saved_poses: Dict[Tuple[int, int], Dict[str, List[Any]]] = {}
    for pkl_name in os.listdir("pickles/sorting"):
        f = os.path.join("pickles/sorting", pkl_name)
        with open(f, "rb") as fp:
            aPickle = pickle.load(fp)
            for key in aPickle:
                if key not in saved_poses:
                    saved_poses[key] = aPickle[key]
    for num_blocks in [2]:
        for e in range(experiment):
            e = 1
            poses = saved_poses[(num_blocks, e)]
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
                state,
                stream_state,
                objects,
            ) = init_geometric_state(num_blocks, poses)
            # update_goal(problem_pddl, num_blocks)
            # wait_for_user()
            saver = pbt.utils.WorldSaver()
            pbt.utils.set_renderer(enable=False)

            plan = coast.plan(
                domain_pddl=domain_pddl,
                problem_pddl=problem_pddl,
                streams_pddl=streams_pddl,
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
                constraint_cls=Constraint,
                use_cache=False
            )
            if plan.action_plan is None or plan.objects is None:
                pbt.utils.disconnect()
                return

            pbt.utils.set_renderer(enable=True)
            saver.restore()
            if experiment > 1:
                with open(write_experiment, "a", newline="") as f_object:
                    res = plan.log.get_timing()
                    writer_csv = csv.DictWriter(
                        f_object,
                        fieldnames=list(res.keys()) + ["total", "num_blocks", "solved"],
                    )
                    res["num_blocks"] = num_blocks
                    res["solved"] = plan.action_plan is not None
                    writer_csv.writerow(res)
                    f_object.close()
                    # writer_csv = csv.DictWriter(
                    #     f_object, fieldnames=list(plan.log.keys())
                    # )
                    # # writer_csv.writerow(total_loggings.keys())
                    # writer_csv.writerow(plan.log.collect_profiles())
                    # f_object.close()
            if args.pybullet and experiment == 1:
                pbt.utils.wait_for_user("Execute?")
                py_state = pybullet_tools.pr2_primitives.State()
                a = 2
                time.sleep(1)
                for action in plan.action_plan:
                    # wait_for_user(f"Executing {action}")
                    text = pbt.utils.add_text(action, (1.5, 0, a))
                    action.execute_state(
                        py_state,
                        world.problem,
                        world.robot,
                        world.problem.arms[0],
                        plan.objects,
                    )
                    pbt.utils.remove_debug(text)
                time.sleep(1)
              

            pbt.utils.disconnect()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(add_help=False)
    parser.add_argument(
        "--domain-pddl",
        "--domain",
        nargs="?",
        default="envs/sorting/domain.pddl",
        help="Pddl domain file.",
    )
    parser.add_argument(
        "--problem-pddl",
        "--problem",
        nargs="?",
        default="envs/sorting/problem.pddl",
        help="Pddl problem file.",
    )
    parser.add_argument(
        "--streams-pddl",
        "--streams",
        nargs="?",
        default="envs/sorting/streams.pddl",
        help="Pddl streams file.",
    )
    parser.add_argument(
        "--streams-py",
        nargs="?",
        default="envs/sorting/streams.py",
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
        "--pybullet",
        type=bool,
        default=True,
        help="Running a domain that runs pybullet",
    )
    parser.add_argument(
        "--search-sample-ratio",
        type=float,
        default=0.35,  # 0.1 2 35 1.0 = don't try to resample.
        help="The ratio of search versus sample",
    )
    parser.add_argument(
        "--timeout", type=int, default=1500, help="Timeout for algorithm"
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
        default="experiments/sorting_ours.csv",
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
