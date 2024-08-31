#!/usr/bin/env python3

import argparse
import csv
import dataclasses
import pathlib
import random
import time
from typing import Dict, List, Optional, Set, Tuple

import numpy as np
import pybullet_tools as pbt
import pybullet_tools.kuka_primitives
import spatialdyn as dyn
import symbolic

import coast
from constraints import LocationConstraint
from coast.constraint import Constraint


@dataclasses.dataclass
class World:
    robot: int
    ab: dyn.ArticulatedBody
    q_home: np.ndarray
    body_names: Dict[int, str]
    movable_bodies: List[int]
    fixed_bodies: List[int]
    table: int
    block_height: float
    new_state: Set[str]
    locations: List[Optional[pbt.utils.Pose]]
    q0: int


def load_world(num_blocks: int = 5, timestep: bool = False) -> World:
    # TODO: store internal world info here to be reloaded
    pbt.utils.set_default_camera()
    pbt.utils.draw_global_system()

    with pbt.utils.HideOutput():
        # add_data_path()
        robot = pbt.utils.load_model(
            pbt.utils.DRAKE_IIWA_URDF, fixed_base=True
        )  # DRAKE_IIWA_URDF | KUKA_IIWA_URDF
        table = pbt.utils.load_model("models/long_floor.urdf")
        ab = dyn.urdf.load_model(
            str(
                pathlib.Path("external/pybullet-planning")
                / pbt.utils.DRAKE_IIWA_URDF
            )
        )
        block_width = 0.07
        block_height = 0.2

        red_id = num_blocks - 1
        blocks = []
        for i in range(num_blocks):
            if i != red_id:
                blocks.append(
                    pbt.utils.create_box(
                        block_width, block_width, block_height, color=pbt.utils.BLUE
                    )
                )
            else:
                blocks.append(
                    pbt.utils.create_box(
                        block_width, block_width, block_height, color=pbt.utils.RED
                    )
                )
         
    pbt.utils.draw_pose(
        pbt.utils.Pose(),
        parent=robot,
        parent_link=pbt.kuka_primitives.get_tool_link(robot),
    )  # TODO: not working
    # dump_body(robot)
    # wait_for_user()
    locations = [None] * 25
    plocations = [None] * 9
    z_block = pbt.utils.stable_z(blocks[0], table)
   
    # LOCS = [
    #     (-0.25, 0.9),
    #     (-0.25, 0.78),
    #     (-0.25, 0.66),
    #     (-0.13, 0.9),
    #     (-0.13, 0.78),
    #     (-0.13, 0.66),
    #     (-0.01, 0.9),
    #     (-0.01, 0.78),
    #     (-0.01, 0.66),
    # ]
    locations[0] = pbt.utils.Pose(pbt.utils.Point(x=-0.25, y=0.9, z=z_block))
    locations[1] = pbt.utils.Pose(pbt.utils.Point(x=-0.25, y=0.78, z=z_block))
    locations[2] = pbt.utils.Pose(pbt.utils.Point(x=-0.25, y=0.66, z=z_block))
    locations[3] = pbt.utils.Pose(pbt.utils.Point(x=-0.13, y=0.9, z=z_block))
    locations[4] = pbt.utils.Pose(pbt.utils.Point(x=-0.13, y=0.78, z=z_block))
    locations[5] = pbt.utils.Pose(pbt.utils.Point(x=-0.13, y=0.66, z=z_block))
    locations[6] = pbt.utils.Pose(pbt.utils.Point(x=-0.01, y=0.9, z=z_block))
    locations[7] = pbt.utils.Pose(pbt.utils.Point(x=-0.01, y=0.78, z=z_block))
    locations[8] = pbt.utils.Pose(pbt.utils.Point(x=-0.01, y=0.66, z=z_block))
    
    new_box = pbt.utils.create_box(block_width + 0.01, block_width +
            0.01, 0.001, color=pbt.utils.GREEN)
    z_block = pbt.utils.stable_z(new_box, table)
    plocations[0] = pbt.utils.Pose(pbt.utils.Point(x=-0.25, y=0.9, z=z_block))
    plocations[1] = pbt.utils.Pose(pbt.utils.Point(x=-0.25, y=0.78, z=z_block))
    plocations[2] = pbt.utils.Pose(pbt.utils.Point(x=-0.25, y=0.66, z=z_block))
    plocations[3] = pbt.utils.Pose(pbt.utils.Point(x=-0.13, y=0.9, z=z_block))
    plocations[4] = pbt.utils.Pose(pbt.utils.Point(x=-0.13, y=0.78, z=z_block))
    plocations[5] = pbt.utils.Pose(pbt.utils.Point(x=-0.13, y=0.66, z=z_block))
    plocations[6] = pbt.utils.Pose(pbt.utils.Point(x=-0.01, y=0.9, z=z_block))
    plocations[7] = pbt.utils.Pose(pbt.utils.Point(x=-0.01, y=0.78, z=z_block))
    plocations[8] = pbt.utils.Pose(pbt.utils.Point(x=-0.01, y=0.66, z=z_block))

   
    body_names = {
        table: "mytable",
    }
    possible_locs = [i for i in range(9) if i != 4]
    new_state = set(["(handempty)"])
    loc_idxs = random.sample(possible_locs, num_blocks)
    line = ""
    for idx in range(9):
        new_box = pbt.utils.create_box(block_width + 0.01, block_width +
                0.01, 0.001, color=pbt.utils.GREEN)
        pbt.utils.add_body_name(new_box, str(idx + 1), text_size=1)
        pbt.utils.set_pose(new_box, plocations[idx])
    if num_blocks != 1:
        for idx in range(9):
            if idx not in loc_idxs[1:] and idx != 4:
                new_state.add(f"(clear loc{idx + 1})")
        pbt.utils.set_pose(blocks[0], locations[4])  # Always set 0 block to middle
        new_state.add("(ontable b1 loc5)")
        line += "loc5 "
    else:
        for idx in range(9):
            if idx != loc_idxs[0]:
                new_state.add(f"(clear loc{idx + 1})")
    for i in range(1, num_blocks + 1):
        body_names[blocks[i - 1]] = f"b{i}"
        # new_state.add(f"(exist b{i})")
        # new_state.add(f"(clear b{i})")
        if i > 1 or num_blocks == 1:
            pbt.utils.set_pose(blocks[i - 1], locations[loc_idxs[i - 1]])
            # wait_for_user(f"blocks {i} {blocks[i - 1]} {loc_idxs[i - 1] + 1}")
            new_state.add(f"(ontable b{i} loc{loc_idxs[i - 1] + 1})")
            line += f"loc{loc_idxs[i - 1] + 1} "
            print(f"(ontable b{i} loc{loc_idxs[i - 1] + 1})", locations[loc_idxs[i - 1]])
    if timestep: 
        for t in range(19):
            new_state.add(f"(Next t{t + 1} t{t + 2})")
        new_state.add("(AtTimestep t1)")
    line = line.strip() + "\n"
    f = open("load_poses/loc_configs.txt", "a")
    f.write(line)
    f.close()
    movable_bodies = blocks

    fixed_bodies = list(set(body_names.keys()) - set(movable_bodies))

    q_home = np.array([0.0, np.pi / 6, 0.0, -np.pi / 3, 0.0, np.pi / 2, 0.0])
    q0 = pbt.kuka_primitives.BodyConf(
                    robot, pbt.utils.get_configuration(robot)
        )
    return World(
        robot,
        ab,
        q_home,
        body_names,
        movable_bodies,
        fixed_bodies,
        table,
        block_height,
        new_state,
        locations,
        q0
    )


def init_geometric_state_kuka(
    num_blocks: int,
    timestep: bool
) -> Tuple[World, str, Set[str]]:
    pbt.utils.connect(use_gui=True)
    world = load_world(num_blocks, timestep)
    # prev_state_start = {}
    # prev_state_start["config"] = {
    #    "t1": (robot, BodyConf(robot, get_configuration(robot)))
    # }
    state = coast.GeometricState()
    objects: List[coast.Object] = [
        coast.Object(name=body_name, object_type="block", value=body_id)
        for body_id, body_name in world.body_names.items()
    ]
    stream_state: Set[coast.CertifiedFact] = set()
    state.set(
        "config",
        0,
        (
            world.robot,
            pbt.kuka_primitives.BodyConf(
                world.robot, pbt.utils.get_configuration(world.robot)
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
           world.q0,
        )
    )

  
    for body_id in world.movable_bodies:
        body_pose = pbt.kuka_primitives.BodyPose(body_id, pbt.utils.get_pose(body_id))
        state.set(world.body_names[body_id], 0, (body_id, body_pose))

        stream_state.add(f"AtPose({world.body_names[body_id]}, p0_{body_id})")
        objects.append(coast.Object(f"p0_{body_id}", "pose", body_pose))
        # prev_state_start[world.body_names[obj]] = {
        #     "t1": (obj, pbt.kuka_primitives.BodyPose(obj, pbt.utils.get_pose(obj)))
        # }
        # prev_state_start[names[obj]] = {"t1": (obj, BodyPose(obj, get_pose(obj)))}
    for i, loc in enumerate(world.locations):
        objects.append(coast.Object(f"loc{i+1}", "location", loc))
        state.set(f"loc{i + 1}", 0, loc)
    #     prev_state_start[f"loc{i + 1}"] = world.locations[i]
    #     prev_state_start[str(world.locations[i]).replace(" ", "")] = f"loc{i+1}"
    state.set("stacks", 0, {m: 1 for m in world.movable_bodies})
  
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
    from symbolic import _P

    problem = symbolic.Problem(f"blocks-{num_blocks}", domain="blocks")
    for prop in world.new_state:
        problem.add_initial_prop(prop)
    problem.set_goal(_P("ontable", f"b{num_blocks}", "loc5"))
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
    timestep: bool
):
    valid_6 = [0, 1, 5, 6, 7]
    valid_7 = [5, 6, 9, 15, 18]
    for num_blocks in [1, 2, 3, 4, 5 ,6 , 7]:#range(1):
        # experiment = 5
        """
        num_blocks = 6, random_seed = 2, 3, 4 impossible (1, 5, 6, 7)
        """
        for e in range(experiment):
            if num_blocks == 6:
                e = valid_6[0]
                valid_6.pop(0)
            if num_blocks == 7:
                e = valid_7[0]
                valid_7.pop(0)
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
            ) = init_geometric_state_kuka(
                num_blocks, timestep
            ) 
            # update_goal(problem_pddl, num_blocks)
            # pbt.utils.wait_for_user()
            saver = pbt.utils.WorldSaver()
            pbt.utils.set_renderer(enable=False)
            if timestep:
                constraint_to_use = Constraint
                # use_cache = True
            else:
                constraint_to_use = LocationConstraint
                use_cache = False
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
                constraint_cls=constraint_to_use,
                use_cache=False,
                sort_streams=True
            )
            if experiment > 1:
                with open(write_experiment, "a", newline="") as f_object:
                    res = plan.log.get_timing()
                    writer_csv = csv.DictWriter(
                        f_object, fieldnames=list(res.keys()) + ["total",
                            "num_blocks", "solved", "n_success_plans", "n_fail_plans"]
                    )
                    res = plan.log.get_timing()
                    res["num_blocks"] = num_blocks
                    res["solved"] = plan.action_plan is not None
                    res["n_success_plans"] = plan.log.num_success_task_plans
                    res["n_fail_plans"] = plan.log.num_fail_task_plans

                    writer_csv.writerow(res)
                    f_object.close()
            if (plan.action_plan is None or plan.objects is None) and experiment == 1:
                pbt.utils.disconnect()
                return

            pbt.utils.set_renderer(enable=True)
            saver.restore()
            plan.log.print()
            if args.pybullet and experiment == 1:
                pbt.utils.wait_for_user("Execute?")
                a = 1.2
                time.sleep(1)
                for action in plan.action_plan:
                    text = pbt.utils.add_text(action, (0.45, -0.2, a))
                    action.execute(plan.objects)
                    pbt.utils.remove_debug(text)
                pbt.utils.wait_for_user("Finish?")

            pbt.utils.disconnect()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(add_help=False)
    parser.add_argument(
            "--timestep",
            type=bool,
            default=False)
    parser.add_argument(
        "--domain-pddl",
        "--domain",
        nargs="?",
        default="envs/block/domain-no-stack.pddl",
        help="Pddl domain file.",
    )
    parser.add_argument(
        "--problem-pddl",
        "--problem",
        nargs="?",
        default="envs/block/problem.pddl",
        help="Pddl problem file.",
    )
    parser.add_argument(
        "--streams-pddl",
        "--streams",
        nargs="?",
        default="envs/kuka/streams.pddl",
        help="Pddl streams file.",
    )
    parser.add_argument(
        "--streams-py",
        nargs="?",
        default="envs/block/streams-motion.py",
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
        default=1.0,  # 1.0 = don't try to resample.
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
        default="experiments/0824_block_ff_timestep_no_cfree.csv",
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
