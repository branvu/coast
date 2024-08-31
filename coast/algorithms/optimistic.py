import abc
from typing import Dict, List, Optional, Sequence, Set

import symbolic  # type: ignore

from coast.object import StreamObject
from coast.algorithms import base, common
from coast.stream import Stream, StreamInstance, StreamState
from coast import utils


def optimistic_output(stream_instance: StreamInstance) -> Dict[str, StreamObject]:
    return stream_instance.lazy_sample()


def retrace(
    stream_state: StreamState,
    optimistic_state: StreamState,
    propositions: Set[str],
    objects: Set[str],
) -> List[StreamInstance]:
    visited_streams: Set[StreamInstance] = set()

    def _retrace(propositions: Set[str], objects: Set[str]) -> List[StreamInstance]:
        stream_plan = []
        for obj in objects:
            if obj in stream_state.objects:
                continue
            stream_instance = optimistic_state.objects[obj].stream_instance
            if stream_instance is None or stream_instance in visited_streams:
                continue

            visited_streams.add(stream_instance)
            stream_plan += _retrace(
                stream_instance.domain, set(map(str, stream_instance.inputs))
            ) + [stream_instance]

        for prop in propositions:
            if prop in stream_state.propositions:
                continue
            try:
                stream_instance = optimistic_state.propositions[prop].stream_instance

            except KeyError:
                # Skip propositions not in the initial state, since these are
                # not certified facts.
                continue
            if stream_instance is None or stream_instance in visited_streams:
                continue

            visited_streams.add(stream_instance)
            stream_plan += _retrace(
                stream_instance.domain, set(map(str, stream_instance.inputs))
            ) + [stream_instance]

        return stream_plan

    return _retrace(propositions, objects)


class Optimistic(base.Algorithm, abc.ABC):
    def plan(
        self,
        pddl: symbolic.Pddl,
        stream_pddl: symbolic.Pddl,
        streams: Sequence[Stream],
    ) -> base.Plan:
        task_plan = None
        stream_plan = None
        stream_state = StreamState(pddl)
        idx_iter = 0
        timer = utils.Profiler()
        timer.tic("start")
        for level in range(self.max_level):
            while task_plan is None and timer.toc("start") < self.timeout:
                print(
                    f"[optimistic.plan] ========== Level: {level}, Iter: {idx_iter} =========="
                )
                idx_iter += 1
                optimistic_state = common.apply_streams(
                    stream_pddl, streams, stream_state, level, optimistic_output
                )

                with timer.profile("search"):
                    optimistic_plan, new_pddl = common.search(
                        pddl, optimistic_state, use_symbolic=False
                    )
                print(f"[optimistic.plan] Search: {timer.get_last('search')}s\n", new_pddl)

                if optimistic_plan is None:
                    break

                with timer.profile("retrace"):
                    stream_plan = retrace(
                        stream_state,
                        optimistic_state,
                        *common.preimage(optimistic_plan, new_pddl),
                    )
                print(f"[optimistic.plan] Retrace: {timer.get_last('retrace')}s")

                with timer.profile("process"):
                    task_plan = self.process_streams(
                        stream_state,
                        stream_plan,
                        optimistic_plan,
                        timeout=max(
                            0.0, timer.compute_sum("search") - timer.compute_sum("process")
                        ),
                    )
                print(f"[optimistic.plan] Process: {timer.get_last('process')}s")

            if task_plan is not None:
                break

        action_skeleton = (
            None if task_plan is None else [node.action for node in task_plan[1:]]
        )
        objects = {
            obj.name: obj
            for obj in stream_state.pddl_objects()
            if obj.value is not None
        }
        print("[optimistic.plan] Stream plan:", stream_plan)
        return base.Plan(action_skeleton, objects, {})

    @classmethod
    @abc.abstractmethod
    def process_streams(
        cls,
        stream_state: StreamState,
        stream_plan: Sequence[StreamInstance],
        optimistic_plan: Sequence[symbolic.PlannerNode],
        timeout: float,
    ) -> Optional[Sequence[symbolic.PlannerNode]]:
        pass
