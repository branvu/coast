from multiprocessing.connection import wait
from pybullet_tools.pr2_primitives import iterate_approach_path, get_tool_from_root
from pybullet_tools.utils import (
    pairwise_collision,
    get_bodies,
    get_distance,
    multiply,
    set_pose,
    interpolate_poses,
    invert,
    wait_if_gui,
    set_renderer
)

BASE_CONSTANT = 1
BASE_VELOCITY = 0.25


def distance_fn(q1, q2):
    distance = get_distance(q1.values[:2], q2.values[:2])
    return BASE_CONSTANT + distance / BASE_VELOCITY


def move_cost_fn(t):
    distance = t.distance(distance_fn=lambda q1, q2: get_distance(q1[:2], q2[:2]))
    return BASE_CONSTANT + distance / BASE_VELOCITY


def get_fixed(robot, movable):
    rigid = [body for body in get_bodies() if body != robot]
    fixed = [body for body in rigid if body not in movable]
    return fixed


#######################################################


def get_cfree_pose_pose_test(collisions=True, **kwargs):
    def test(b1, p1, b2, p2):
        if not collisions or (b1 == b2):
            return True
        p1.assign()
        p2.assign()
        res = not pairwise_collision(b1, b2, **kwargs)
        return res  # , max_distance=0.001)

    return test


def get_cfree_grasp_pose_test(problem):
    gripper = problem.get_gripper()
    tool_from_root = get_tool_from_root(problem.robot, problem.arms[0])

    def test(b1, p1, g1):
        p1.assign()
        grasp_pose = multiply(multiply(p1.value, invert(g1.value)), tool_from_root)
        approach_grasp_pose = multiply(multiply(p1.value, invert(g1.approach)), tool_from_root)
        set_pose(gripper, grasp_pose)
        res = not pairwise_collision(gripper, b1)
        set_pose(gripper, approach_grasp_pose)
        res2 = not pairwise_collision(gripper, b1)
        return res and res2

    return test


def get_cfree_obj_approach_pose_test(collisions=True):
    def test(b1, p1, g1, b2, p2):
        if not collisions or (b1 == b2):
            return True
        p2.assign()
        grasp_pose = multiply(p1.value, invert(g1.value))
        approach_pose = multiply(p1.value, invert(g1.approach), g1.value)
        for obj_pose in interpolate_poses(grasp_pose, approach_pose):
            # set_pose(b1, obj_pose)
            if pairwise_collision(b1, b2):
                wait_if_gui("collision on approach")
                return False
        return True

    return test


def get_cfree_approach_pose_test(problem, collisions=True):
    # TODO: apply this before inverse kinematics as well
    arm = "left"
    gripper = problem.get_gripper()

    def test(b1, p1, g1, b2, p2):
        if not collisions or (b1 == b2):
            return True, None
        p2.assign()
        for _ in iterate_approach_path(problem.robot, arm, gripper, p1, g1, body=b1):
            if pairwise_collision(b1, b2) or pairwise_collision(gripper, b2):
                # set_renderer(enable=True)
                # wait_if_gui(f"false {b1} {b2}")
                # set_renderer(enable=False)
                return False, (b1, b2)
        # set_renderer(enable=True)
        # wait_if_gui(f"checking {b1} {b2}")
        # set_renderer(enable=False)
        return True, None

    return test


def get_cfree_traj_pose_test(robot, collisions=True):
    def test(c, b2, p2):
        # TODO: infer robot from c
        if not collisions:
            return True, None
        state = c.assign()
        if b2 in state.attachments:
            return True, None
        p2.assign()
        for _ in c.apply(state):
            state.assign()
            for b1 in state.attachments:
                if pairwise_collision(b1, b2):
                    # set_renderer(enable=True)
                    # wait_if_gui(f"collide with {b2}")
                    # set_renderer(enable=False)
                    return False, (b1, b2)
            if pairwise_collision(robot, b2):
                # set_renderer(enable=True)
                # wait_if_gui(f"colllide with robot and {b2}")
                # set_renderer(enable=False)
                return False, None
        # TODO: just check collisions with moving links
        return True, None

    return test


def get_cfree_traj_grasp_pose_test(problem, collisions=True):
    def test(c, a, b1, g, b2, p2):
        raise NotImplementedError()
        if not collisions or (b1 == b2):
            return True
        state = c.assign()
        if (b1 in state.attachments) or (b2 in state.attachments):
            return True
        p2.assign()
        grasp_attachment = g.get_attachment(problem.robot, a)
        for _ in c.apply(state):
            state.assign()
            grasp_attachment.assign()
            if pairwise_collision(b1, b2):
                return False
            if pairwise_collision(problem.robot, b2):
                return False
        return True

    return test
