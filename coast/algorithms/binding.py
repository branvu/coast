from typing import Any, Dict, List, Mapping, NamedTuple, Optional, Sequence, Tuple

import symbolic  # type: ignore

from coast.algorithms import common, optimistic
from coast.stream import StreamInstance, StreamState
from coast.object import Object


def update_bindings(
    stream_state: StreamState, bindings: Dict[str, Any], stream_instance: StreamInstance
) -> Optional[Dict[str, Any]]:
    new_inputs = [
        bindings[input.name] if input.name in bindings else input
        for input in stream_instance.inputs
    ]
    new_stream_instance = StreamInstance.create_with_new_inputs(
        stream_instance, new_inputs
    )
    sampled_objects = common.add_certified(stream_state, new_stream_instance, next)
    if sampled_objects is None:
        return None

    new_bindings = dict(bindings)
    for s, object in zip(stream_instance.outputs, sampled_objects):
        new_bindings[s.name] = object.value

    return new_bindings


def apply_bindings(
    stream_state: StreamState,
    bindings: Dict[str, Any],
    optimistic_plan: Sequence[symbolic.PlannerNode],
) -> Tuple[Sequence[symbolic.PlannerNode], StreamState]:
    new_stream_state = stream_state.copy()
    for s in bindings:
        new_stream_state.objects[s].fact.value = bindings[s]

    return optimistic_plan, new_stream_state


class PrioritizedBindings(NamedTuple):
    stream_instance_count: int
    stream_plan_steps_to_go: int
    recency: float
    idx_plan: int
    bindings: Mapping[str, Object]
    # stream_plan: List[StreamInstance]

    @staticmethod
    def create(
        bindings: Mapping[str, Object],
        stream_plan: List[StreamInstance],
        idx_plan: int,
        time_since_start: float,
    ) -> "PrioritizedBindings":
        return PrioritizedBindings(
            stream_instance_count=stream_plan[idx_plan].count,
            stream_plan_steps_to_go=len(stream_plan) - idx_plan,
            recency=-time_since_start,
            idx_plan=idx_plan,
            bindings=bindings,
            # stream_plan=stream_plan,
        )


class Binding(optimistic.Optimistic):
    @classmethod
    def process_streams(
        cls,
        stream_state: StreamState,
        stream_plan: Sequence[StreamInstance],
        optimistic_plan: Sequence[symbolic.PlannerNode],
        timeout: float,
    ) -> Optional[Tuple[Sequence[symbolic.PlannerNode], StreamState]]:

        """
        Binding Algorithm
        """
        bindings: Dict[str, Any] = {}
        for stream_instance in stream_plan:
            maybe_bindings = update_bindings(stream_state, bindings, stream_instance)
            if maybe_bindings is None:
                return None
            bindings = maybe_bindings

        return apply_bindings(stream_state, bindings, optimistic_plan)
