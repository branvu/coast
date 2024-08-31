import pathlib
import pprint
import importlib.util
from typing import Any, List, Sequence, Set, Type

import symbolic

from coast import algorithms
from coast.action import Action
from coast.constraint import Constraint
from coast.formula import _P
from coast.stream import Object, Stream


def load_streams(streams_py: str, world: Any) -> List[Stream]:
    spec = importlib.util.spec_from_file_location(
        pathlib.Path(streams_py).stem, streams_py
    )
    if spec is None or spec.loader is None:
        raise ImportError(f"Could not import {streams_py}")
    streams_module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(streams_module)
    streams = []
    # stream_pddl = symbolic.Pddl(streams_pddl, problem_pddl)
    streams = [
        cls(world)
        for cls in streams_module.__dict__.values()
        if isinstance(cls, type) and issubclass(cls, Stream)
    ]
    return streams


def load_geometric_actions(streams_py: str, world: Any) -> List[Action]:
    # TODO: Load actions from pddl instead of streams.py
    spec = importlib.util.spec_from_file_location(
        pathlib.Path(streams_py).stem, streams_py
    )
    if spec is None or spec.loader is None:
        raise ImportError(f"Could not import {streams_py}")
    streams_module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(streams_module)
    # stream_pddl = symbolic.Pddl(streams_pddl, problem_pddl)
    actions = [
        cls(world)
        for cls in streams_module.__dict__.values()
        if isinstance(cls, type) and issubclass(cls, Action)
    ]
    return actions


def load_geometric_predicates(streams_py: str) -> List[_P]:
    # TODO: Load actions from pddl instead of streams.py
    spec = importlib.util.spec_from_file_location(
        pathlib.Path(streams_py).stem, streams_py
    )
    if spec is None or spec.loader is None:
        raise ImportError(f"Could not import {streams_py}")
    streams_module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(streams_module)
    geometric_predicates = streams_module.GEOMETRIC_PREDICATES
    return geometric_predicates


def load_algorithm(name: str, **kwargs) -> algorithms.Algorithm:
    algs = {
        "improved": algorithms.Improved,
    }
    return algs[name](**kwargs)


def plan(
    domain_pddl: str,
    problem_pddl: str,
    streams_pddl: str,
    streams_py: str,
    algorithm: str,
    max_level: int,
    search_sample_ratio: float,
    timeout: int,
    experiment: int,
    stream_state: Set[str],
    objects: Sequence[Object],
    random_seed: int,
    world: Any,
    constraint_cls: Type[Constraint],
    use_cache: bool,
    sort_streams: bool = True,
) -> algorithms.Plan:

    streams = load_streams(streams_py, world)
    actions = load_geometric_actions(streams_py, world)
    geometric_predicates = load_geometric_predicates(streams_py)
    alg = load_algorithm(
        algorithm,
        max_level=max_level,
        search_sample_ratio=search_sample_ratio,
        timeout=timeout,
        use_cache=use_cache
    )
    pddl = symbolic.Pddl(domain_pddl, problem_pddl)
    stream_pddl = symbolic.Pddl(streams_pddl, problem_pddl)
    plan = alg.plan(
        pddl=pddl,
        stream_pddl=stream_pddl,
        streams=streams,
        actions=actions,
        geometric_predicates=geometric_predicates,
        stream_state=stream_state,
        objects=objects,
        world=world,
        constraint_cls=constraint_cls,
        sort_streams=sort_streams
    )

    if plan.objects is not None:
        print("[coast.main.plan]", "Objects:")
        pprint.pprint(list(plan.objects.values()))
    else:
        print("[coast.main.plan]", "Objects: None")
    plan.log.print()
    if plan.action_plan is not None:
        print("[coast.main.plan]", "Action plan:")
        pprint.pprint(plan.action_plan)

    return plan
