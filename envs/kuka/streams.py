from typing import Dict, Optional, Sequence, Any, List, Tuple

import coast
from coast import Object
import symbolic  # type: ignore
from ncollide import ncollide3d
from ctrlutils import eigen
import ctrlutils
import json
from pprint import pprint
from envs.pybullet.pddlstream_utils import (
    get_cfree_pose_pose_test,
    get_cfree_obj_approach_pose_test,
)
from pybullet_tools.utils import (
    WorldSaver,
    connect,
    dump_body,
    get_pose,
    set_pose,
    Pose,
    Point,
    set_default_camera,
    stable_z,
    BLOCK_URDF,
    SMALL_BLOCK_URDF,
    get_configuration,
    SINK_URDF,
    STOVE_URDF,
    load_model,
    is_placement,
    get_body_name,
    disconnect,
    DRAKE_IIWA_URDF,
    get_bodies,
    HideOutput,
    wait_for_user,
    KUKA_IIWA_URDF,
    add_data_path,
    load_pybullet,
    LockRenderer,
    has_gui,
    draw_pose,
    draw_global_system,
)
from pybullet_tools.kuka_primitives import (
    BodyPose,
    BodyConf,
    Command,
    get_grasp_gen,
    get_stable_gen,
    get_ik_fn,
    get_free_motion_gen,
    get_holding_motion_gen,
    get_movable_collision_test,
    get_tool_link,
)


def placepose(object, surface, prev_state):
    # Generates a gripper pose on the object
    func = from_gen_fn(
        get_stable_gen(
            prev_state["fixed"][
                0
            ]  # + update_obstacles(prev_state, exclude=str(object))
        )
    )
    return next(func(object, surface))[0]


def pickgrasp(object, prev_state, t):
    # Generate a pick body pose for the object
    return next(from_gen_fn(get_grasp_gen(prev_state["robot"][t][0]))(object))[0]


def get_free_motion_traj(conf1, conf2, prev_state, t):
    # update_position(prev_state)
    # Generates a trajectory between two gripper poses
    func = get_free_motion_gen(
        prev_state["robot"][t][0],
        prev_state["fixed"][0] + update_obstacles(prev_state, t),
        False,
    )
    res = func(conf1, conf2, fluents=[])
    if res is None:
        return None
    return res[0]


def ik_stream(object, object_pose, grasp_pose, prev_state, t):
    func = get_ik_fn(
        prev_state["robot"][t][0],
        prev_state["fixed"][0],  # + update_obstacles(prev_state, exclude=object),
        num_attempts=10,
    )
    return func(object, object_pose, grasp_pose)


def testCfreeApproachPose(b1, p1, g1, b2, p2):
    test = get_cfree_obj_approach_pose_test()
    return test(b1, p1, g1, b2, p2)


def testCfreeTrajPose(command, object, object_pose):
    test = get_movable_collision_test()
    res = test(command, object, object_pose)
    return not res


def testCfreePosePose(b1, p1, b2, p2):
    test = get_cfree_pose_pose_test()
    res = test(b1, p1, b2, p2)
    return res


def get_holding_motion_traj(conf1, conf2, object, grasp, prev_state, t):
    func = get_holding_motion_gen(
        prev_state["robot"][t][0],
        prev_state["fixed"][0] + update_obstacles(prev_state, t, exclude=str(object)),
        False,
    )
    res = func(conf1, conf2, object, grasp, fluents=[])
    if res is None:
        return None
    else:
        return res[0]


def update_position(prev_state, t):
    for m in prev_state["movable_names"]:
        obj, obj_pose = prev_state[m][t]
        obj_pose.assign()


def update_obstacles(prev_state, t, exclude=None):
    obstacles = []
    for m in prev_state["movable_names"]:
        obj, obj_pose = prev_state[m][t]
        if str(exclude) != str(obj):
            obstacles.append(obj)
    return obstacles


def from_list_gen_fn(list_gen_fn):
    # Purposefully redundant for now
    return list_gen_fn


def from_gen_fn(gen_fn):
    return from_list_gen_fn(
        lambda *args, **kwargs: (
            [] if ov is None else [ov] for ov in gen_fn(*args, **kwargs)
        )
    )


def update_logging(name: Any, logging: Dict[str, Tuple[int, int]], success=True):
    if name in logging:
        if success:
            logging[name] = (logging[name][0] + 1, logging[name][1] + 1)
        else:
            logging[name] = (logging[name][0], logging[name][1] + 1)
    else:
        if success:
            logging[name] = (1, 1)
        else:
            logging[name] = (0, 1)


def get_ik_cache(bindings: Dict[Any, Any], object, object_pose, grasp):
    # Checking all of the keys in binding
    for key in bindings:
        if (
            key[0] == "ikstream"
            and key[1] == str(object)
            and key[2] == str(object_pose)
        ):
            test_grasp = key[3]
            if test_grasp == grasp.without_id():
                return key
    return None

def propagate_value(prev_state, key1, key2, copied_value):
    # Fill in next timesteps to be at this pose
    # v1 v1 v2 v1 v1
    # None v2
    prev = f"t{int(key2[1:])}"
    for idx in range(int(key2[1:]), prev_state["action_len"] - 1):
        timestep2 = f"t{idx + 2}"
        if prev_state[key1][prev] == prev_state[key1][timestep2]:
            prev_state[key1][timestep2] = copied_value
        else:
            break
    return prev_state
class FailPick(coast.Stream):
    name = "Failpick"
    certified_fact = "Failpick"
    substreams = 5
    substream_names = [
        "pickgrasp",
        "ikstream_pick",
        "test_ik",
        "test_approach",
        "freemotion",
    ]

    def sample(
        self,
        inputs: Sequence[symbolic.Object],
        outputs: Sequence[symbolic.Object],
        stream_state: coast.StreamState,
    ) -> Optional[Dict[str, Object]]:
        return None

    def get_logging_key(
        self,
        inputs: Sequence[Object],
        substream_id: int,
        prev_state: Dict[Any, Any],
        bindings: Dict[Any, Any],
    ) -> Optional[Any]:
        t1 = inputs[-2].name
        t2 = inputs[-1].name
        object = prev_state[inputs[0].name][t1][0]

        # Pick Grasp
        logging_key = ("pickgrasp", str(object), t1, t2)

        if substream_id == 0:
            return logging_key
        object_pose = prev_state[inputs[0].name][t1][1]
        if logging_key in bindings:
            pickg = bindings[logging_key]
        else:
            return None

        # PICK IK
        logging_key = (
            "ikstream",
            str(object),
            str(object_pose),
            pickg.without_id(),
            t1,
            t2,
        )
        if substream_id == 1:
            return logging_key
        if logging_key not in bindings:
            return None
        new_robot_config, command = bindings[logging_key]

        logging_key = ("test_ik", str(command), t1, t2) # TODO: Flatten
        if substream_id == 2:
            return logging_key
        if logging_key not in bindings:
            print("new test_ik")
            return None

        # Test Approach
        logging_key = (
            "test_approach",
            str(object),
            str(object_pose),
            str(pickg),
            t1,
            t2,
        )
        if substream_id == 3:
            return logging_key
        if logging_key not in bindings:
            return None
        # Free motion
        logging_key = (
            "freemotion",
            str(prev_state["robot"][t1][1]),
            str(new_robot_config),
            t1,
            t2,
        )
        if substream_id == 4:
            return logging_key
        if logging_key not in bindings:
            print("returning none free", logging_key)
            return None
        return logging_key

    def sample_from_prev(
        self,
        inputs: Sequence[Object],
        outputs: Sequence[Object],
        stream_state: coast.StreamState,
        prev_state: Dict[str, Any],
        bindings: Dict[Any, Any],
        logging: Dict[str, Tuple[int, int]],
        substream: int,
    ) -> Optional[Tuple[Dict[str, Any], Dict[Any, Any]]]:
        t2 = inputs[-1].name
        t1 = inputs[-2].name
        object = prev_state[inputs[0].name][t1][0]  # The object we want to pick up
        object_pose = prev_state[inputs[0].name][t1][1]
        update_position(prev_state, t1)
        
        # Sample a grasp of the object defaults top grasp
        key = ("pickgrasp", str(object), t1, t2)
        if key not in bindings:
            pickg = pickgrasp(object, prev_state, t1)[0]
            bindings[key] = pickg
            if pickg is None:
                print("Pick grasp not found")
                update_logging(key, logging, success=False)
                return None
            update_logging(key, logging, success=True)
        else:
            pickg = bindings[key]
        # ================== OUTPUT = PICKGRASP ============== #

        if substream == 0:
            prev_state["cur_grasp"][t2] = pickg
            prev_state = propagate_value(prev_state, "cur_grasp", t1, prev_state["cur_grasp"][t2])
            print("Pick grasp success")
            return prev_state, bindings

        # Obtains a configuration for the robot to grip the object
        key = ("ikstream", str(object), str(object_pose), pickg.without_id(), t1, t2)
        cached_key = get_ik_cache(bindings, object, object_pose, pickg)
        if cached_key is None:
            res = ik_stream(object, object_pose, pickg, prev_state, t1)
            if res is None:
                print("ikstream failed for pick", key)
                update_logging(key, logging, False)
                return None
            bindings[key] = res
            update_logging(key, logging, True)
        else:
            if substream == 1:
                update_logging(key, logging, True)
                bindings[key] = bindings[cached_key]
            res = bindings[cached_key]
        new_robot_config, command = res
        if substream == 1:
            old_state = prev_state["robot"][t1]
            prev_state["robot"][t2] = (old_state[0], new_robot_config)
            prev_state = propagate_value(prev_state, "robot", t1, prev_state["robot"][t2])
            prev_state["commands"].append(("pick", command, t1, t2))
            print("ikstream pick success")
            return prev_state, bindings

        key = ("test_ik", str(command), t1, t2)
        if key not in bindings:
            for name in prev_state["movable_names"]:
                test_object, test_object_pose = prev_state[name][t1]
                if object == test_object:
                    continue
                
                res = testCfreeTrajPose(
                    command=command, object=test_object, object_pose=test_object_pose
                )
                bindings[key] = res
                if not res:
                    print("Collision when picking object", object)
                    update_logging(key, logging, False)
                    return None
            bindings[key] = res
            update_logging(key, logging, True)
        else:
            res = bindings[key]

        if substream == 2:
            print("Testing pick ik success")
            return prev_state, bindings
        
        key = ("test_approach", str(object), str(object_pose), str(pickg), t1, t2)
        if key not in bindings:
            for name in prev_state["movable_names"]:
                test_object, test_object_pose = prev_state[name][t1]

                if test_object == object:
                    continue
                res = testCfreeApproachPose(
                    object, object_pose, pickg, test_object, test_object_pose
                )
                if res is False:
                    print(
                        "Collision between",
                        object,
                        object_pose,
                        test_object,
                        test_object_pose,
                    )
                    update_logging(key, logging, False)
                    return None
                
                bindings[key] = res
                update_logging(key, logging, True)
        else:
            res = bindings[key]
        if substream == 3:
            print("Testing pick approach success")
            return prev_state, bindings
        # Generate free motion trajectory from previous bodypose
        key = ("freemotion", str(prev_state["robot"][t1][1]), str(new_robot_config), t1, t2)
        if key not in bindings:
            new_command = get_free_motion_traj(
                prev_state["robot"][t1][1], new_robot_config, prev_state, t1
            )
            bindings[key] = new_command
            if not new_command:
                print("Free motion not possible")
                update_logging(key, logging, False)
                return None
            update_logging(key, logging, True)
        else:
            new_command = bindings[key]
        prev_state["commands"].append(("move", new_command, t1, t2))
        return prev_state, bindings


class FailPlace(coast.Stream):
    name = "Failplace"
    certified_fact = name
    substreams = 6
    substream_names = [
        "placepose",
        "ikstream_place",
        "test_ik",
        "test_approach",
        "test_placepose",
        "holdingmotion",
    ]

    def sample(
        self,
        inputs: Sequence[symbolic.Object],
        outputs: Sequence[symbolic.Object],
        stream_state: coast.StreamState,
    ) -> Optional[Dict[str, Object]]:
        return None

    def get_logging_key(
        self,
        inputs: Sequence[Object],
        substream_id: int,
        prev_state: Dict[Any, Any],
        bindings: Dict[Any, Any],
    ) -> Optional[Any]:
        t1 = inputs[-2].name
        t2 = inputs[-1].name
        object, object_pose = prev_state[inputs[0].name][t1]
        surface = prev_state[inputs[1].name][0]

        logging_key: Tuple = ("placepose", str(object), str(surface), t1, t2)
        if substream_id == 0:
            return logging_key
        placep = bindings[logging_key]

        logging_key = (
            "ikstream",
            str(object),
            str(placep),
            prev_state["cur_grasp"][t1].without_id(),
            t1,
            t2,
        )
        if substream_id == 1:
            return logging_key
        new_robot_config, command = bindings[logging_key]

        logging_key = ("test_ik", str(command), t1, t2)
        if substream_id == 2:
            return logging_key

        logging_key = (
            "test_approach",
            str(object),
            str(object_pose),
            str(prev_state["cur_grasp"][t1]),
            t1,
            t2,
        )
        if substream_id == 3:
            return logging_key

        logging_key = ("test_placepose", str(object), str(placep), t1, t2)
        if substream_id == 4:
            return logging_key
        # Holding motion
        logging_key = (
            "holdingmotion",
            str(prev_state["robot"][t1][1]),
            str(new_robot_config),
            str(object),
            str(prev_state["cur_grasp"][t1]),
            t1,
            t2,
        )

        return logging_key

    def sample_from_prev(
        self,
        inputs: Sequence[Object],
        outputs: Sequence[Object],
        stream_state: coast.StreamState,
        prev_state: Dict[str, Any],
        bindings: Dict[Any, Any],
        logging: Dict[str, Tuple[int, int]],
        substream: int,
    ) -> Optional[Tuple[Dict[str, Any], Dict[Any, Any]]]:
        t1 = inputs[-2].name
        t2 = inputs[-1].name
        object, object_pose = prev_state[inputs[0].name][t1]
        surface = prev_state[inputs[1].name][0]
        update_position(prev_state, t1)
        # Sample a stable pose on the surface
        key = ("placepose", str(object), str(surface), t1, t2)
        if key not in bindings:
            placep = placepose(object, surface, prev_state)[0]
            if placep is None:
                print("Could not find place pose")
                update_logging(key, logging, False)
                return None
            print("Found place pose for", str(object), "on", str(surface), "=", placep)
            bindings[key] = placep
            update_logging(key, logging, True)
        else:
            placep = bindings[key]

        if substream == 0:
            prev_state[inputs[0].name][t2] = (prev_state[inputs[0].name][t1][0], placep)
            prev_state = propagate_value(prev_state, inputs[0].name, t1, prev_state[inputs[0].name][t2])
            print("place pose success")
            return prev_state, bindings

        # Find a robot configuration to place the object
        key = (
            "ikstream",
            str(object),
            str(placep),
            prev_state["cur_grasp"][t1].without_id(),
            t1,
            t2,
        )
        cached_key = get_ik_cache(bindings, object, placep,
                prev_state["cur_grasp"][t1])
        if cached_key is None:
            res = ik_stream(object, placep, prev_state["cur_grasp"][t1], prev_state, t1)
            if res is None:
                print("ikstream place failed", key)
                update_logging(key, logging, False)
                return None
            bindings[key] = res
            update_logging(key, logging, True)
        else:
            if substream == 1:
                update_logging(key, logging, True)
                bindings[key] = bindings[cached_key]
            res = bindings[cached_key]
        new_robot_config, command = res
        if substream == 1:
            old_state = prev_state["robot"][t1]
            prev_state["robot"][t2] = (old_state[0], new_robot_config)
            prev_state = propagate_value(prev_state, "robot", t1, prev_state["robot"][t2])
            prev_state["commands"].append(("place", command, t1, t2))
            print("ik place success")
            return prev_state, bindings

        key = ("test_ik", str(command), t1, t2)
        if key not in bindings:
            for name in prev_state["movable_names"]:
                test_object, test_object_pose = prev_state[name][t1]
                if object == test_object:
                    continue
                res = testCfreeTrajPose(
                    command=command, object=test_object, object_pose=test_object_pose
                )
                if not res:
                    print("Collision when placing object", object)
                    update_logging(key, logging, False)
                    return None
            bindings[key] = res
            update_logging(key, logging, True)
        else:
            res = bindings[key]
        if substream == 2:
            print("Testing place ik success")
            return prev_state, bindings

        key = (
            "test_approach",
            str(object),
            str(object_pose),
            str(prev_state["cur_grasp"][t1]),
            t1,
            t2,
        )
        if key not in bindings:
            for name in prev_state["movable_names"]:
                test_object, test_object_pose = prev_state[name][t1]
                if test_object == object:
                    continue
                res = testCfreeApproachPose(
                    object,
                    object_pose,
                    prev_state["cur_grasp"][t1],
                    test_object,
                    test_object_pose,
                )
                if not res:
                    print(
                        "Collision between",
                        object,
                        object_pose,
                        test_object,
                        test_object_pose,
                    )
                    update_logging(key, logging, False)
                    return None
                
            bindings[key] = res
            update_logging(key, logging, True)
        else:
            res = bindings[key]
        if substream == 3:
            print("Testing approach pose success")
            return prev_state, bindings

        key = ("test_placepose", str(object), str(placep), t1, t2)
        if key not in bindings:
            for name in prev_state["movable_names"]:
                test_object, test_pose = prev_state[name][t1]
                if test_object == object:
                    continue
                res = testCfreePosePose(test_object, test_pose, object, placep)
                if not res:
                    print("Collision of place pose and", test_object, test_pose)
                    update_logging(key, logging, False)
                    return None
            bindings[key] = res
            update_logging(key, logging, True)
        else:
            res = bindings[key]
        if substream == 4:
            print("Test Place pose success")
            return prev_state, bindings

        # Generate holding motion path from previous configuration
        key = (
            "holdingmotion",
            str(prev_state["robot"][t1][1]),
            str(new_robot_config),
            str(object),
            str(prev_state["cur_grasp"][t1]),
            t1,
            t2,
        )
        if key not in bindings:
            new_command = get_holding_motion_traj(
                prev_state["robot"][t1][1],
                new_robot_config,
                object,
                prev_state["cur_grasp"][t1],
                prev_state,
                t1
            )
            bindings[key] = new_command
            if new_command is None:
                print("Holding motion trajectory not able to be found")
                update_logging(key, logging, False)
                return None
            bindings[key] = new_command
            update_logging(key, logging, True)
            print("New holding motion sampled success")
        else:
            new_command = bindings[key]
            print("holding motion success from binding")

        prev_state["commands"].append(("move", new_command, t1, t2))
        return prev_state, bindings


class FailCook(coast.Stream):
    name = "Failcook"
    certified_fact = name
    substreams = 0
    substream_names = []

    def get_logging_key(
        self,
        inputs: Sequence[Object],
        substream_id: int,
        prev_state: Dict[Any, Any],
        bindings: Dict[Any, Any],
    ) -> Optional[Any]:
        pass

    def sample(
        self,
        inputs: Sequence[symbolic.Object],
        outputs: Sequence[symbolic.Object],
        stream_state: coast.StreamState,
    ) -> Optional[Dict[str, Object]]:
        return None

    def sample_from_prev(
        self,
        inputs: Sequence[Object],
        outputs: Sequence[Object],
        stream_state: coast.StreamState,
        prev_state: Dict[str, Object],
        bindings: Dict[Any, Any],
        logging: Dict[str, Tuple[int, int]],
        substream: int,
    ) -> Optional[Tuple[Dict[str, Any], Dict[Any, Any]]]:
        update_logging(("cook", None), logging, True)
        return prev_state, bindings


class FailClean(coast.Stream):
    name = "Failclean"
    certified_fact = name
    substreams = 0
    substream_names = []

    def get_logging_key(
        self,
        inputs: Sequence[Object],
        substream_id: int,
        prev_state: Dict[Any, Any],
        bindings: Dict[Any, Any],
    ) -> Optional[Any]:
        pass

    def sample(
        self,
        inputs: Sequence[symbolic.Object],
        outputs: Sequence[symbolic.Object],
        stream_state: coast.StreamState,
    ) -> Optional[Dict[str, Object]]:
        return None

    def sample_from_prev(
        self,
        inputs: Sequence[Object],
        outputs: Sequence[Object],
        stream_state: coast.StreamState,
        prev_state: Dict[str, Object],
        bindings: Dict[Any, Any],
        logging: Dict[str, Tuple[int, int]],
        substream: int,
    ) -> Optional[Tuple[Dict[str, Any], Dict[Any, Any]]]:
        update_logging(("clean", None), logging, True)
        return prev_state, bindings
