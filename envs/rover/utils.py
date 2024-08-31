from pybullet_tools.pr2_primitives import Command, Pose, Conf, Trajectory, State, \
    create_trajectory, Attach, Detach, get_target_path, SELF_COLLISIONS
from pybullet_tools.pr2_problems import get_fixed_bodies
from pybullet_tools.pr2_utils import HEAD_LINK_NAME, get_visual_detections, \
    visible_base_generator, inverse_visibility, get_kinect_registrations, get_detection_cone, get_viewcone, \
    MAX_KINECT_DISTANCE, plan_scan_path, get_group_joints, get_group_conf, set_group_conf
from pybullet_tools.utils import link_from_name, create_mesh, set_pose, get_link_pose, \
    wait_for_duration, unit_pose, remove_body, is_center_stable, get_body_name, get_name, point_from_pose, \
    plan_waypoints_joint_motion, pairwise_collision, plan_direct_joint_motion, BodySaver, set_joint_positions, \
    INF, get_length, multiply, wait_for_user, LockRenderer, set_color, RED, GREEN, apply_alpha, dump_body, \
    get_link_subtree, child_link_from_joint, get_link_name, get_name

OTHER = 'other'
LOCALIZED_PROB = 0.99
class BeliefState(State):
    def __init__(self, task, b_on={}, registered=tuple(), **kwargs):
        super(BeliefState, self).__init__(**kwargs)
        self.task = task
        self.b_on = b_on
        #self.localized = set(localized)
        self.registered = set(registered)
        # TODO: store configurations
        """
        for body in task.get_bodies():
            if not self.is_localized(body):
                #self.poses[body] = None
                self.poses[body] = object()
                #del self.poses[body]
            #elif body not in registered:
            #    point, quat = self.poses[body].value
            #    self.poses[body] = Pose(body, (point, None))
        """
    def is_localized(self, body):
        for surface in self.b_on[body].support():
            if (surface != OTHER) and (LOCALIZED_PROB <= self.b_on[body].prob(surface)):
                return True
        return False
    def __repr__(self):
        items = []
        for b in sorted(self.b_on.keys()):
            d = self.b_on[b]
            support_items = ['{}: {:.2f}'.format(s, d.prob(s)) for s in sorted(d.support(), key=str)]
            items.append('{}: {{{}}}'.format(b, ', '.join(support_items)))
        return '{}({},{})'.format(self.__class__.__name__,
                                  #self.b_on,
                                  '{{{}}}'.format(', '.join(items)),
                                  list(map(get_name, self.registered)))
class Register(Command):
    _duration = 1.0

    def __init__(self, robot, body, camera_frame=HEAD_LINK_NAME, max_depth=MAX_KINECT_DISTANCE):
        self.robot = robot
        self.body = body
        self.camera_frame = camera_frame
        self.max_depth = max_depth
        self.link = link_from_name(robot, camera_frame)

    def control(self, **kwargs):
        # TODO: filter for target object and location?
        return get_kinect_registrations(self.robot)

    def apply(self, state, **kwargs):
        # TODO: check if actually can register
        mesh, _ = get_detection_cone(self.robot, self.body, camera_link=self.camera_frame, depth=self.max_depth)
        if mesh is None:
            wait_for_user()
        assert(mesh is not None) # TODO: is_visible_aabb(body_aabb, **kwargs)=False sometimes without collisions
        with LockRenderer():
            cone = create_mesh(mesh, color=apply_alpha(GREEN, 0.5))
            set_pose(cone, get_link_pose(self.robot, self.link))
            wait_for_duration(1e-2)
        wait_for_duration(self._duration)
        remove_body(cone)
        wait_for_duration(1e-2)
        state.registered.add(self.body)
        yield

    def __repr__(self):
        return '{}({},{})'.format(self.__class__.__name__, get_body_name(self.robot),
                                  get_name(self.body))
class Scan(Command):
    _duration = 0.5

    def __init__(self, robot, surface, detect=True, camera_frame=HEAD_LINK_NAME):
        self.robot = robot
        self.surface = surface
        self.camera_frame = camera_frame
        self.link = link_from_name(robot, self.camera_frame)
        self.detect = detect

    def apply(self, state, **kwargs):
        # TODO: identify surface automatically
        with LockRenderer():
            cone = get_viewcone(color=apply_alpha(RED, 0.5))
            set_pose(cone, get_link_pose(self.robot, self.link))
            wait_for_duration(1e-2)
        wait_for_duration(self._duration) # TODO: don't sleep if no viewer?
        remove_body(cone)
        wait_for_duration(1e-2)

        if self.detect:
            # TODO: the collision geometries are being visualized
            # TODO: free the renderer
            head_joints = get_group_joints(self.robot, 'head')
            exclude_links = set(get_link_subtree(self.robot, child_link_from_joint(head_joints[-1])))
            detections = get_visual_detections(self.robot, camera_link=self.camera_frame, exclude_links=exclude_links,)
                                               #color=apply_alpha(RED, 0.5))
            print('Detections:', detections)
            for body, dist in state.b_on.items():
                obs = (body in detections) and (is_center_stable(body, self.surface))
                if obs or (self.surface not in state.task.rooms):
                    # TODO: make a command for scanning a room instead?
                    dist.obsUpdate(get_observation_fn(self.surface), obs)
            #state.localized.update(detections)
        # TODO: pose for each object that can be real or fake
        yield

    def __repr__(self):
        return '{}({})'.format(self.__class__.__name__, get_body_name(self.surface))

