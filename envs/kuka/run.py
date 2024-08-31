#!/usr/bin/env python3

import argparse
import csv
import dataclasses
import pathlib
import random
import time
from typing import Dict, List, Optional, Set, Tuple, Any

import numpy as np
import pybullet_tools as pbt
from pybullet_tools import kuka_primitives, utils as pb_utils
import spatialdyn as dyn
import symbolic

import coast
from coast.constraint import Constraint


@dataclasses.dataclass
class World:
    robot: int
    ab: dyn.ArticulatedBody
    body_names: Dict[int, str]
    movable_bodies: List[int]
    fixed_bodies: List[Any]
    new_state: Set[str]
    q_home: np.ndarray


def load_world() -> World:
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
                pathlib.Path("external/pybullet-planning/")
                / pbt.utils.DRAKE_IIWA_URDF
            )
        )

        sink = pb_utils.load_model(
            pb_utils.SINK_URDF, pose=pb_utils.Pose(pb_utils.Point(x=-0.5))
        )
        stove = pb_utils.load_model(
            pb_utils.STOVE_URDF, pose=pb_utils.Pose(pb_utils.Point(x=+0.5))
        )
        celery = pb_utils.load_model(pb_utils.BLOCK_URDF, fixed_base=False)
        radish = pb_utils.load_model(pb_utils.SMALL_BLOCK_URDF, fixed_base=False)
        egg = pb_utils.load_model(pb_utils.BLOCK_URDF, fixed_base=False)
        bacon = pb_utils.load_model(pb_utils.BLOCK_URDF, fixed_base=False)
        pb_utils.add_body_name(stove, "mystove", text_size=1, color=(1,1,0))
        pb_utils.add_body_name(sink, "mysink", text_size=1, color=(1,1,0))
        pb_utils.add_body_name(celery, "celery", text_size=1)
        pb_utils.add_body_name(radish, "radish", text_size=1)
        pb_utils.add_body_name(egg, "egg", text_size=1)
        pb_utils.add_body_name(bacon, "bacon", text_size=1)
        # cup = pb_utils.load_model('models/dinnerware/cup/cup_small.urdf',
        # pb_utils.Pose(pb_utils.Point(x=+0.5, y=+0.5, z=0.5)), fixed_base=False)
    pb_utils.draw_pose(
        pb_utils.Pose(), parent=robot, parent_link=kuka_primitives.get_tool_link(robot)
    )  # TODO: not working
    # dump_body(robot)

    body_names = {
        sink: "mysink",
        stove: "mystove",
        celery: "celery",
        radish: "radish",
        bacon: "bacon",
        egg: "egg",
        table: "mytable",
    }
    movable_bodies = [celery, radish, bacon, egg]
    pb_utils.set_pose(
        celery, pb_utils.Pose(pb_utils.Point(y=0.5, z=pb_utils.stable_z(celery, table)))
    )
    pb_utils.set_pose(
        radish,
        pb_utils.Pose(pb_utils.Point(y=-0.5, z=pb_utils.stable_z(radish, table))),
    )
    pb_utils.set_pose(
        bacon,
        pb_utils.Pose(pb_utils.Point(x=0.3, y=-0.5, z=pb_utils.stable_z(bacon, table))),
    )
    pb_utils.set_pose(
        egg,
        pb_utils.Pose(pb_utils.Point(x=-0.3, y=0.5, z=pb_utils.stable_z(egg, table))),
    )
    new_state = set()
    for m in movable_bodies:
        name = body_names[m]
        new_state.add(f"(On {name} mytable)")
    
    for t in range(29):
        new_state.add(f"(Next t{t + 1} t{t + 2})")
    new_state.add("(AtTimestep t1)")
    fixed_bodies = [sink, stove, table]
    q_home = np.array([0.0, np.pi / 6, 0.0, -np.pi / 3, 0.0, np.pi / 2, 0.0])
    # pb_utils.wait_for_user()

    return World(
        robot,
        ab,
        body_names,
        movable_bodies,
        fixed_bodies,
        new_state,
        q_home
    )


def init_geometric_state_kuka(
    num_blocks: int,
) -> Tuple[World, str, Set[str], Set[str], List[Any]]:
    pbt.utils.connect(use_gui=True)
    world = load_world()
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
            pbt.kuka_primitives.BodyConf(
                world.robot, pbt.utils.get_configuration(world.robot)
            ),
        )
    )

    for body_id in world.movable_bodies:
        body_pose = pbt.kuka_primitives.BodyPose(body_id, pbt.utils.get_pose(body_id))
        state.set(world.body_names[body_id], 0, (body_id, body_pose))

        stream_state.add(f"AtPose({world.body_names[body_id]}, p0_{body_id})")
        objects.append(coast.Object(f"p0_{body_id}", "pose", body_pose))

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
    from symbolic import _P, _and

    problem = symbolic.Problem(f"blocks-{num_blocks}", domain="pick-and-place")
    
    for prop in world.new_state:
        problem.add_initial_prop(prop)
    res = {}
    count = 1
    for m in world.movable_bodies:
        name = world.body_names[m]
        res[count] = _P("Cooked", name)
        res[count + 1] = _P("Cleaned", name)
        count += 2
    total = []
    for i in range(1, num_blocks + 1):
        total.append(res[i])
    problem.set_goal(_and(total))
    # problem.set_goal(_P("ontable", f"b{num_blocks}", "loc5"))
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
    for num_blocks in range(1, 9):
        for e in range(experiment):
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
                num_blocks
            )  # TODO: This going to fail for other benchmarks
            # update_goal(problem_pddl, num_blocks)

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
                sort_streams=False,
                use_cache=False
            )
            if experiment > 1:
                with open(write_experiment, "a", newline="") as f_object:
                    res = plan.log.get_timing()
                    writer_csv = csv.DictWriter(
                        f_object, fieldnames=list(res.keys()) + ["total",
                            "num_blocks", "solved", "n_success_plans",
                            "n_fail_plans"]
                    )
                    res["num_blocks"] = num_blocks
                    res["solved"] = plan.action_plan is not None
                    res["n_success_plans"] = plan.log.num_success_task_plans
                    res["n_fail_plans"] = plan.log.num_fail_task_plans
                    writer_csv.writerow(res)
                    f_object.close()
            if plan.action_plan is None or plan.objects is None:
                pbt.utils.disconnect()
                return

            pbt.utils.set_renderer(enable=True)
            saver.restore()
            plan.log.print()
            if args.pybullet and experiment == 1:
                pbt.utils.wait_for_user("Execute?")
                time.sleep(1)
                a = 1
                for action in plan.action_plan:
                    # pbt.utils.wait_for_user("next")
                    text = pb_utils.add_text(action, (0.6, -0.2, a),
                            text_size=1.7)
                    action.execute(plan.objects)
                    # a -= 0.05
                    if str(action)[0].lower() == 'c':
                        time.sleep(0.3)
                    pb_utils.remove_debug(text)
                # pbt.utils.wait_for_user("Finish?")
            elif not args.pybullet and args.experiment == 1:
                for action_state in plan.action_skeleton:
                    # TODO: fix
                    # action_args: List[str] = symbolic.parse_args(action_state)
                    # action = symbolic.parse_head(action_state)
                    # coast.policy.execute_action(
                    #     action, action_args, plan.geometric_state, ab, redis
                    # )
                    time.sleep(1)
            time.sleep(1)
            pbt.utils.disconnect()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(add_help=False)
    parser.add_argument(
        "--domain-pddl",
        "--domain",
        nargs="?",
        default="envs/kuka/domain.pddl",
        help="Pddl domain file.",
    )
    parser.add_argument(
        "--problem-pddl",
        "--problem",
        nargs="?",
        default="envs/kuka/problem.pddl",
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
        default="envs/kuka/new_streams.py",
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
        default=10,  # 0.02 1.0 = don't try to resample.
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
        default="experiments/0815_kuka_our_no_sort_ff.csv",
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
