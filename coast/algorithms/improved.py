import abc
import collections
import queue
import itertools
from typing import (
    Any,
    Callable,
    Dict,
    List,
    Mapping,
    Optional,
    Sequence,
    Set,
    Tuple,
    Type,
    Union,
)

import symbolic

from coast.algorithms.base import Log
from coast.algorithms.binding import PrioritizedBindings
from coast.action import Action, ActionCall
from coast.object import Object, StreamObject
from coast.algorithms import base, common
from coast.constraint import Constraint
from coast.formula import _P
from coast.stream import Stream, StreamConstraint, StreamInstance
from coast import utils


def infer_action_arguments(
    stream_state: Set[str],
    task_plan: Sequence[symbolic.PlannerNode],
    geometric_actions: Dict[str, Action],
    objects: Mapping[str, Object],
) -> Tuple[List[ActionCall], List[Dict[str, None]], Dict[str, Object]]:
    state = {s: None for s in stream_state}
    new_objects = dict(objects)
    action_calls = []
    states = [state]
    for t, node in enumerate(task_plan[1:]):
        action_call = node.action
        action = geometric_actions[symbolic.parse_head(action_call)]
        args_map = action.map_inputs_to_objects(action_call, state)

        for output in action.outputs:
            new_output = StreamObject(f"{output.name[1:]}_{t+1}", output.type)
            args_map[output.name] = new_output.name
            new_objects[new_output.name] = new_output

        action_call = ActionCall(action, args_map)
        action_calls.append(action_call)
        state = action_call.apply(state, new_objects)
        states.append(state)

    return action_calls, states, new_objects


def sort_stream_plan_by_descendants(
    stream_plan: List[StreamInstance],
) -> List[StreamInstance]:
    def count_descendants(
        stream_instance: StreamInstance,
        dependency_graph: Dict[str, List[StreamInstance]],
        num_descendants: Dict[StreamInstance, int],
    ) -> int:
        try:
            return num_descendants[stream_instance]
        except KeyError:
            children = set(
                itertools.chain.from_iterable(
                    dependency_graph[arg.name]
                    for arg in stream_instance.outputs.values()
                )
            )
            num_descendants[stream_instance] = sum(
                1 + count_descendants(child, dependency_graph, num_descendants)
                for child in children
            )
        return num_descendants[stream_instance]

    # Compute dependency graph.
    dependency_graph: Dict[str, List[StreamInstance]] = collections.defaultdict(list)
    for stream_instance in stream_plan:
        all_inputs = itertools.chain(
            stream_instance.inputs.values(), stream_instance.fluent_inputs.values()
        )
        for arg in all_inputs:
            dependency_graph[arg.name].append(stream_instance)

    num_descendants: Dict[StreamInstance, int] = {}
    sorted_stream_plan = sorted(
        (
            -count_descendants(stream_instance, dependency_graph, num_descendants),
            idx_plan,
            stream_instance,
        )
        for idx_plan, stream_instance in enumerate(stream_plan)
    )

    stream_plan = [stream_instance[2] for stream_instance in sorted_stream_plan]

    return stream_plan


def sort_stream_plan(
    stream_plan: List[StreamInstance],
    key: Callable[[StreamInstance], Any],
    idx_start: int = 0,
) -> List[StreamInstance]:
    ancestors_map: Dict[StreamInstance, Set[StreamInstance]] = {}

    def get_ancestors(
        stream_instance: StreamInstance, output_map: Dict[str, StreamInstance]
    ) -> Set[StreamInstance]:
        try:
            return ancestors_map[stream_instance]
        except KeyError:
            ancestors = set()
            all_inputs = itertools.chain(
                stream_instance.inputs.values(), stream_instance.fluent_inputs.values()
            )
            for arg in all_inputs:
                if not isinstance(arg, StreamObject):
                    continue
                parent = output_map[arg.name]
                ancestors |= get_ancestors(parent, output_map)
                ancestors.add(parent)
            
            ancestors_map[stream_instance] = ancestors
        return ancestors_map[stream_instance]

    def sorted_plan(
        stream_plan: List[StreamInstance],
        output_map: Dict[str, StreamInstance],
        key: Callable[[StreamInstance], Any],
        visited: Set[StreamInstance] = set(),
    ) -> List[StreamInstance]:
        if len(stream_plan) <= 1:
            return stream_plan
        # mina = -1
        # for s in stream_plan:
        #     cost = key(s)
        #     # print(s, min)
        #     # input(cost)
        #     if mina == -1 or cost < mina:
        #         mina = cost
        #         next_stream = s
        # print("Next stream", next_stream, key(next_stream))
        next_stream = min(stream_plan, key=key)
        # assert a == next_stream
        visited.add(next_stream)

        ancestors = get_ancestors(next_stream, output_map)
        ancestor_plan = [s for s in stream_plan if s not in visited and s in ancestors]
        remaining_plan = [
            s for s in stream_plan if s not in visited and s not in ancestors
        ]

        return (
            sorted_plan(ancestor_plan, output_map, key, visited)
            + [next_stream]
            + sorted_plan(remaining_plan, output_map, key, visited)
        )

    # Compute ancestor graph.
    output_map: Dict[str, StreamInstance] = {}
    for stream_instance in stream_plan:
        for arg in stream_instance.outputs.values():
            # print(arg.name, output_map)
            assert arg.name not in output_map
            output_map[arg.name] = stream_instance
    # import pdb; pdb.set_trace()
    sorted_stream_plan = stream_plan[:idx_start] + sorted_plan(
        stream_plan[idx_start:], output_map, key
    )

    return sorted_stream_plan


def sort_stream_plan_by_inputs(
    stream_plan: List[StreamInstance], idx_start: int = 0
) -> List[StreamInstance]:
    cache: Dict[str, int] = {}

    def num_inputs(stream_instance: StreamInstance) -> int:
        key = str(stream_instance)
        try:
            return cache[key]
        except KeyError:
            
            cache[key] = len( # -1 
                [
                    arg
                    for arg in itertools.chain(
                        stream_instance.inputs, stream_instance.fluent_inputs
                    )
                    #if isinstance(arg, StreamObject)
                ]
            )  # - len(s.outputs),
            
            return cache[key]
    return sort_stream_plan(stream_plan, num_inputs, idx_start)


def sort_stream_plan_by_failures(
    stream_plan: List[StreamInstance], log: Log, idx_start: int = 0
) -> List[StreamInstance]:
    return sort_stream_plan(stream_plan, lambda s: log.success_rate(s), idx_start)


def sort_stream_plan_by_expected_time(
    stream_plan: List[StreamInstance], log: Log, idx_start: int = 0
) -> List[StreamInstance]:
    cache: Dict[str, Tuple[bool, float]] = {}

    def expected_time(stream_instance: StreamInstance) -> Tuple[bool, float]:
        key = stream_instance.stream.name
        try:
            return cache[key]
        except KeyError:
            p = log._success_rate(key)
            cache[key] = (p == 1.0, p * log._average_time(key))
            return cache[key]

    return sort_stream_plan(stream_plan, expected_time, idx_start)


def retrace(
    stream_state: Set[str],
    task_plan: Sequence[symbolic.PlannerNode],
    actions_map: Dict[str, Action],
    streams_map: Dict[str, Stream],
    predicates_map: Dict[str, _P],
    objects: Mapping[str, Object],
    sort_streams: bool = True
) -> Tuple[List[StreamInstance], List[ActionCall]]:
    action_plan, states, new_objects = infer_action_arguments(
        stream_state, task_plan, actions_map, objects
    )

    stream_plan = []
    for action_call, pre_state, post_state in zip(action_plan, states[:-1], states[1:]):
        state = {prop: None for prop in pre_state if prop in post_state}
        certified_facts = action_call.get_certified_facts(
            state, list(new_objects.values())
        )
        for fact in certified_facts:
            pred, args = symbolic.parse_proposition(fact)
            stream = streams_map[pred]
            inputs = {param.name[1:]: param for param in stream.inputs}
            outputs = {
                param.name[1:]: param for param in stream.outputs
            }  # TODO: Remove ? from param name
            fact_template = stream.certified
            # input(f"{fact_template.args} {args}")
            assert len(fact_template.args) == len(args)
            for param, arg in zip(fact_template.args, args):
                param = param[1:]  # TODO: Remove ? from param name
                if param in inputs:
                    inputs[param] = new_objects.get(arg, arg)
                elif param in outputs:
                    outputs[param] = new_objects[arg]
            fluents = {
                pred: [
                    {
                        param: new_objects[arg]
                        for param, arg in zip(
                            predicates_map[pred].args, symbolic.parse_args(prop)
                        )
                    }
                    for prop in state
                    if symbolic.parse_head(prop) == pred
                ]
                for pred in stream.fluents
            }

            stream_instance = stream.create_instance(
                inputs=inputs,
                outputs=outputs,
                action_call=action_call,
                fluents=fluents,
            )
            stream_plan.append(stream_instance)
    if sort_streams:
        sorted_stream_plan = sort_stream_plan_by_inputs(stream_plan)
    else:
        sorted_stream_plan = stream_plan
    return sorted_stream_plan, action_plan

def rep_constraints(constraints):
    s = ""
    for i in range(len(constraints)):
        s += str(constraints[i])
    return s
class Improved(base.Algorithm, abc.ABC):
    def __init__(
        self,
        max_level: int,
        search_sample_ratio: float,
        timeout: float,
        use_cache: bool = False,
    ):
        super().__init__(max_level=max_level)
        self.search_sample_ratio = search_sample_ratio
        self.timeout = timeout
        self.use_cache = use_cache
        

    def plan(
        self,
        pddl: symbolic.Pddl,
        stream_pddl: symbolic.Pddl,
        streams: Sequence[Stream],
        actions: Sequence[Action],
        geometric_predicates: Sequence[_P],
        stream_state: Set[str],
        objects: Sequence[Object],
        world: Any,
        constraint_cls: Type[Constraint],
        sort_streams: bool = True
    ) -> base.Plan:
        objects_map = {obj.name: obj for obj in objects}
        actions_map = {type(action).__name__.lower(): action for action in actions}
        streams_map = {stream.certified.predicate: stream for stream in streams}
        predicates_map = {p.predicate: p for p in geometric_predicates}

        constraints: List[Constraint] = []
        constrained_pddl = pddl

        log = Log()
        timer = utils.Timer()
        timer.tic("plan")
        # Store queue of successful task plan and current constraints
        plan_queue: queue.PriorityQueue[Tuple[
                Union[List[symbolic.PlannerNode], symbolic.Pddl, List[StreamConstraint]]
            ]] = queue.PriorityQueue()
        plan_queue.put((0, 0, timer.toc("plan"), [], pddl, constraints[:]))
        if self.use_cache:
            cache: Dict = {}
        else:
            cache = None
        constraint_map = collections.defaultdict(int)
        for idx_iter in itertools.count():
            action_plan: Optional[List[ActionCall]] = None
            maybe_bindings: Optional[
                Union[Mapping[str, Object], StreamConstraint]
            ] = None
            log.set_iteration(idx_iter)

            if timer.toc("plan") > self.timeout:
                print("[improved.plan] Failure: timeout")
                break

            print(f"\n[improved.plan] ========== Iter: {idx_iter} ==========")
            count, _, _, optimistic_plan, new_pddl, constraints = plan_queue.get()
            print(f"[improved.plan] Dequeing task plan count: {count} len_constraints: {len(constraints)} empty? {plan_queue.empty()}")
            # input(constraint_map)
            constraint_map[rep_constraints(constraints)] += 1
            # input(constraint_map)
            constrained_pddl = new_pddl
            if len(constraints) > 0:
                constrained_pddl = Constraint.constrain_pddl(pddl, constraints)
            print("[improved.plan] State:", constrained_pddl.initial_state)
            with log.profile("search"):
                optimistic_plan, new_pddl = common.search(
                    constrained_pddl, constrained_pddl.initial_state, use_symbolic=False
                )
                
            print(f"[improved.plan] Search: {log.get_last('search')}s")
            
            if optimistic_plan is None:
                print("[improved.plan] Search Failed")
                log.num_fail_task_plans += 1
                if plan_queue.empty():
                    print("Empty Queue")
                    break
                else:
                    continue
            else:
                log.num_success_task_plans += 1
            
            print(
                f"[improved.plan] Plan: {[node.action for node in optimistic_plan[1:]]}"
            )
            assert len(optimistic_plan) > 1

            with log.profile("retrace"):
                stream_plan, action_plan = retrace(
                    stream_state=stream_state,
                    task_plan=optimistic_plan,
                    actions_map=actions_map,
                    streams_map=streams_map,
                    predicates_map=predicates_map,
                    objects=objects_map,
                    sort_streams=sort_streams
                )
                if sort_streams:
                    stream_plan = sort_stream_plan_by_expected_time(stream_plan, log)
                for stream_instance in stream_plan:
                    stream_instance.clear_count()
            print(
                f"[improved.retrace] Retrace: {log.get_last('retrace')}s", stream_plan
            )
            '''
            (1 - self.search_sample_ratio)
                        / self.search_sample_ratio
                        * log.compute_sum("search")
                        - log.compute_sum("process")
            '''
            with log.profile("process"):
                maybe_bindings, visited_bindings = self.process_streams(
                    stream_plan=stream_plan,
                    optimistic_plan=optimistic_plan[1:],
                    timeout=min(
                        (self.search_sample_ratio * log.compute_sum("search") - log.compute_sum("process")),
                        self.timeout - timer.toc("plan"),
                    ),
                    world=world,
                    bindings=objects_map,
                    pddl=new_pddl,
                    log=log,
                    cache=cache
                )
            print(f"[improved.plan] Process: {log.get_last('process')}s")
            # Add to queue but without the constraint
            plan_queue.put((constraint_map[rep_constraints(constraints)], -len(constraints), -timer.toc("plan"), optimistic_plan, new_pddl, constraints[:]))
            print("Put in queue a", len(constraints))
            # input("check constraints")
            if (
                isinstance(maybe_bindings, StreamConstraint)
                and maybe_bindings is not None
            ):
                constraint_type = maybe_bindings.constraint_type
                constraints.append(
                    constraint_type(
                        pddl=new_pddl,
                        task_plan=optimistic_plan,
                        action_plan=action_plan,
                        stream_plan=stream_plan,
                        objects=objects,
                        visited_bindings=visited_bindings,
                        world=world,
                        constraint=maybe_bindings.constraint,
                    )
                )
                # Add plan, pddl, and new constraints to queue
                plan_queue.put((constraint_map[rep_constraints(constraints)], -len(constraints), -timer.toc("plan"), optimistic_plan, new_pddl, constraints[:]))
                print("Put in queue b", len(constraints))
                continue
            if maybe_bindings is not None:
                return base.Plan(action_plan, maybe_bindings, log)
            
            constraints.append(
                constraint_cls(
                    pddl=new_pddl,
                    task_plan=optimistic_plan,
                    action_plan=action_plan,
                    stream_plan=stream_plan,
                    objects=objects,
                    visited_bindings=visited_bindings,
                    world=world,
                )
            )
            plan_queue.put((constraint_map[rep_constraints(constraints)], -len(constraints), -timer.toc("plan"), optimistic_plan, new_pddl, constraints[:]))
            print("Put in queue c", len(constraints))
        print("[improved.plan] Failure: unable to solve motion plan")

        return base.Plan(action_plan, maybe_bindings, log)

    @classmethod
    def process_streams(
        cls,
        stream_plan: List[StreamInstance],
        optimistic_plan: Sequence[symbolic.PlannerNode],
        timeout: float,
        world: Any,
        bindings: Mapping[str, Object],
        pddl: symbolic.Pddl,
        log: Log,
        cache: Any
    ) -> Tuple[
        Optional[Union[Mapping[str, Object], StreamConstraint]],
        List[PrioritizedBindings],
    ]:
        """Adaptive Algorithm."""
        bindings_queue: queue.PriorityQueue[PrioritizedBindings] = queue.PriorityQueue()
        # If we want to cache, then we just have to look at the task plan and
        # get the current list of bindings
        if cache and optimistic_plan in cache:
            # Add the bindings to the queue
            for binding in cache[optimistic_plan]:
                bindings_queue.put(binding)
        else:
            bindings_queue.put(
                PrioritizedBindings.create(
                    bindings=bindings,
                    stream_plan=stream_plan,
                    idx_plan=0,
                    time_since_start=0.0,
                )
            )

        visited_bindings = []
        timer = utils.Timer()
        timer.tic("process_streams")
        while not bindings_queue.empty():
            prioritized_binding = bindings_queue.get()
            

            stream_instance = stream_plan[prioritized_binding.idx_plan]
            if stream_instance.count > 0 and timer.toc("process_streams") > timeout:
                timer_out = timer.toc("process_streams") > timeout
                print(
                    f"[improved.plan] Timeout exceeded {timer_out} and ran out of streams {stream_instance.count > 0}! Breaking on next failure!"
                )
                break
            visited_bindings.append(prioritized_binding)# visited bindings
            print(stream_instance, stream_instance.action_call)
            # Sample stream.
            with log.profile_stream(stream_instance):
                print(f"Instance: {stream_instance}")
                maybe_bindings = stream_instance.next(prioritized_binding.bindings)
            log.log(stream_instance, maybe_bindings)

            # if isinstance(maybe_bindings, StreamConstraint):
            #    return maybe_bindings, visited_bindings

            if maybe_bindings is not None and not isinstance(
                maybe_bindings, StreamConstraint
            ):
                new_bindings = maybe_bindings

                # print(
                #     "[improved.plan] Sample Success:",
                #     stream_instance,
                #     "Action:",
                #     stream_instance.action_call,
                # )
                if prioritized_binding.idx_plan >= len(stream_plan) - 1:
                    # Last stream successfully sampled.
                    return new_bindings, visited_bindings

                # Queue next stream in stream plan.
                bindings_queue.put(
                    PrioritizedBindings.create(
                        bindings=new_bindings,
                        stream_plan=stream_plan,
                        idx_plan=prioritized_binding.idx_plan + 1,
                        time_since_start=timer.toc("process_streams"),
                    )
                )
            else:
                print(
                    "[improved.plan] Sample failed:",
                    stream_instance,
                    "Action:",
                    stream_instance.action_call,
                )
                # input()
                if stream_instance.stream.resample(False):
                    bindings_queue.put(
                        PrioritizedBindings.create(
                            bindings=prioritized_binding.bindings,
                            stream_plan=stream_plan,
                            idx_plan=prioritized_binding.idx_plan,
                            time_since_start=timer.toc("process_streams"),
                        )
                    )

            # if maybe_bindings is None:
            #     return None, visited_bindings

            if stream_instance.stream.resample(True) or stream_instance.count == 0:
                # Queue current stream again.
                bindings_queue.put(
                    PrioritizedBindings.create(
                        bindings=prioritized_binding.bindings,
                        stream_plan=stream_plan,
                        idx_plan=prioritized_binding.idx_plan,
                        time_since_start=timer.toc("process_streams"),
                    )
                )
        if bindings_queue.empty():
            print("Bindings finished")
        # save current task plan and current successful bindings
        if cache:
            cache[optimistic_plan] = visited_bindings[:-1]
        if isinstance(maybe_bindings, StreamConstraint):
            return maybe_bindings, visited_bindings
        return None, visited_bindings
