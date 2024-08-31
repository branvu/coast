from typing import Any, Sequence, Set, Tuple

import symbolic
from symbolic import problem
from symbolic import _and, _not, _or, _P, _when

from coast.action import ActionCall
from coast.algorithms.binding import PrioritizedBindings
from coast.constraint import Constraint
from coast.object import Object
from coast.stream import StreamInstance
from coast import utils


def create_location_implication(
    pddl: symbolic.Pddl,
    task_plan: Sequence[symbolic.PlannerNode],
    index_failed: int,
    stream_plan: Sequence[StreamInstance],
    objects: Sequence[Object],
    visited_bindings: Sequence[PrioritizedBindings],
    constraint
) -> str:
    stream_instance = stream_plan[visited_bindings[-1].idx_plan]
    task_plan = task_plan[1:]  # To account for the first "" action
    # for prop in task_plan[index_failed].state:
    #     print(prop)
    #     if problem.parse_head(prop) == "ontable" and problem.parse_args(prop)[0] == stream_instance.inputs["b"]:
    #         print("[pooopo", prop)
    # 
    # print(task_plan[index_failed].state)
    # input(task_plan[index_failed - 1].state)
    # index_failed -= 1
    
    cf_action_params = [
        str(p) for p in problem.parse_args(task_plan[index_failed].action)
    ]

    current_action_params = [
        str(p) for p in problem.parse_args(task_plan[index_failed - 1].action)
    ]
    if constraint == "" or constraint is None:
        loc = next(
            problem.parse_args(prop)[1]
            for prop in task_plan[index_failed].state
            if problem.parse_head(prop) == "ontable"
            and problem.parse_args(prop)[0] == stream_instance.inputs["b"]
        )
    else:
        loc = next(
            problem.parse_args(prop)[1]
            for prop in task_plan[index_failed].state
            if problem.parse_head(prop) == "ontable"
            and problem.parse_args(prop)[0] == constraint
        )
    params = next(
        a
        for a in pddl.actions
        if a.name == problem.parse_head(task_plan[index_failed - 1].action)
    ).parameters
    action = problem.parse_head(task_plan[index_failed].action).lower()
    num_blocks = len(
        [obj for obj in objects if obj.type == "block" and obj.name[0] == "b"]
    )
    implies = [
        _P(
            f"Fail{action}",
            f"b{block}",
            cf_action_params[1],
        )
        for block in range(1, num_blocks + 1)
    ]
    condition = ""
    # If the collision block's location is DIFFERENT than our current block
    if loc != current_action_params[1]:
        # Just include the not clear constraint

        equality = _P("=", f"?{params[1].name}", loc)
        if action == "place": # Pick action, add to complement
            complement_condition = _or(_P("clear", loc), equality)
            condition = _and(_not(_P("clear", loc)), _not(equality))
        else:
            complement_condition = _and(_P("clear", loc), _not(equality))
            condition = _or(_not(_P("clear", loc)), equality)
    else:
        # Else don't use the not clear, just use the equality conditions
        if index_failed == 0:
            condition += f"(and (= ?{params[0]} {cf_action_params[0]}) (= ?{params[1]} {cf_action_params[1]}))"
        else:
            condition += f"(and (= ?{params[0]} {current_action_params[0]}) (= ?{params[1]} {current_action_params[1]}))"
        complement_condition = _or(
            _not(_P("=", f"?{params[0].name}", current_action_params[0])),
            _not(_P("=", f"?{params[1].name}", current_action_params[1])),
        )

    # Create implication
    implication = ""
    if index_failed != 0:
        implication = problem.pprint_formula(_when(condition, _and(*implies)))
    implication += problem.pprint_formula(
        _when(complement_condition, _and(*[_not(p) for p in implies], ""))
    )
    # print(cf_action_params, current_action_params)
    # input(implication)
    return implication


def create_location_prop(actions: Sequence[symbolic.Action], index_failed: int) -> str:
    assert index_failed == 0
    action_name, args = problem.parse_proposition(actions[index_failed])
    cf_action_params = [str(p) for p in args]
    implies = utils.serialize_proposition(
        f"fail{action_name.lower()}", cf_action_params
    )

    return implies


class LocationConstraint(Constraint):
    def __init__(
        self,
        pddl: symbolic.Pddl,
        task_plan: Sequence[symbolic.PlannerNode],
        action_plan: Sequence[ActionCall],
        stream_plan: Sequence[StreamInstance],
        objects: Sequence[Object],
        visited_bindings: Sequence[PrioritizedBindings],
        world: Any,
        constraint: Any=None, # Extra info to sneak in more information
    ):
        super().__init__(
            pddl=pddl,
            task_plan=task_plan,
            action_plan=action_plan,
            stream_plan=stream_plan,
            objects=objects,
            visited_bindings=visited_bindings,
            world=world,
        )

        stream_instance = stream_plan[visited_bindings[-1].idx_plan]
        action_call = stream_instance.action_call
        self.index_failed = action_plan.index(action_call)
        self.actions = [node.action for node in task_plan[1:]]

        if self.index_failed == 0:
            self._edit = (create_location_prop(self.actions, self.index_failed),
                create_location_implication(
                    pddl,
                    task_plan,
                    self.index_failed,
                    stream_plan,
                    objects,
                    visited_bindings,
                    constraint
                )
            )
        else:
            self._edit = tuple([create_location_implication(
                pddl,
                task_plan,
                self.index_failed,
                stream_plan,
                objects,
                visited_bindings,
                constraint
                ) , "\t"])
    def apply(self, domain_str: str) -> Tuple[str, Set[str]]:
        initial_state = set()
        the_edit = self._edit[0]

        if self.index_failed == 0:
            initial_state = set([self._edit[0]])
            the_edit = self._edit[1]
        # Finding action needed
        action_index = domain_str.index(
            f":action {problem.parse_head(self.actions[self.index_failed - 1])}"
        )

        # Locate the postconditions
        effect_index = domain_str[action_index:].index("effect") + action_index

        # Find last parentheses
        current_effect, first, last = utils.parse_paren(domain_str[effect_index:])
        first += effect_index
        last += effect_index

        # Create new effect predicate by inserting within the and
        domain_str = domain_str.replace(
            current_effect,
            current_effect[: len(current_effect) - 1] + the_edit + ")",
        )
        return domain_str, initial_state
    def __eq__(self, other) -> bool:
        return isinstance(other, LocationConstraint) and self._edit == other._edit

    def __neq__(self, other) -> bool:
        return not (self == other)

    def __hash__(self) -> int:
        return hash(self._edit)

    def __str__(self) -> str:
        if isinstance(self._edit, tuple):
            return ",".join([str(e).replace("\n", "").replace("\t", "") for e in self._edit])
        else:
            return str(self._edit[0])
