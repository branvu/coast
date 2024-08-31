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


def apply_constraint(pddl: symbolic.Pddl, plan, index_failed, implies, domain_str=None):
    if domain_str is None:
        filepath = pddl.domain_pddl
        txt = open(filepath, "r")
        domain_str = open(filepath, "r").read()
        txt.close()
    actions = plan

    # Finding action needed
    action_index = domain_str.index(
        f":action {problem.parse_head(actions[index_failed - 1])}"
    )

    current_action_params = [
        str(p) for p in problem.parse_args(actions[index_failed - 1])
    ]

    # Locate the postconditions
    effect_index = domain_str[action_index:].index("effect") + action_index

    # Find last parentheses
    current_effect, first, last = utils.parse_paren(domain_str[effect_index:])
    first += effect_index
    last += effect_index
    list_log_actions = []

    if index_failed > 1:
        # Then get list of actions as (logpick...)
        for i in range(len(actions)):
            if i < index_failed - 1:
                a = actions[i]
                list_log_actions.append(
                    problem._P(
                        f"log{problem.parse_head(a)}",
                        " ".join([str(p) for p in problem.parse_args(a)]),
                    )
                )

    # Create condition for current action arguments
    condition = ""
    for a in pddl.actions:
        if a.name == problem.parse_head(actions[index_failed - 1]):
            params = a.parameters
            chained_and = ""
            for i in range(len(params)):
                chained_and += _P(
                    "=", "?" + str(params[i]), str(current_action_params[i])
                )
                if i != len(params) - 1:
                    chained_and += " "
            for log in list_log_actions:
                chained_and += log
            condition = _and(chained_and, "")

    # Add / Create condition for previous action history

    # Create implication
    condition = problem.pprint_formula(condition)
    # p = _when(condition, implies)
    # print(p)
    # implication = problem.pprint_formula(_when(condition, implies))
    implication = f"(when {condition} {implies})"
    print(implication)
    # Create new effect predicate by inserting within the and
    domain_str = domain_str.replace(
        current_effect,
        current_effect[: len(current_effect) - 1] + implication + ")",
        1,
    )
    return domain_str


class CfreeConstraint(Constraint):
    def __init__(
        self,
        pddl: symbolic.Pddl,
        task_plan: Sequence[symbolic.PlannerNode],
        action_plan: Sequence[ActionCall],
        stream_plan: Sequence[StreamInstance],
        objects: Sequence[Object],
        visited_bindings: Sequence[PrioritizedBindings],
        world: Any,
        constraint: str,
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
        self.implies = constraint
        self._edit = apply_constraint(
            pddl, self.actions, self.index_failed, self.implies
        )

    def apply(self, domain_str: str) -> Tuple[str, Set[str]]:
        new_state = self.pddl.initial_state
        new_domain = domain_str
        if self.index_failed == 0:
            # format is (constraint ?1 ?2) but it needs to be constraint(?1 ?2)
            if "and" in self.implies:
                # Trim off ()
                new_one = self.implies[4:-1].strip().split(")")
                for a in new_one[:-1]:
                    a = a + ")"
                    a = a.strip()
                    new_state.add(utils.rewrite_prop(a))
            else:
                new_state.add(utils.rewrite_prop(self.implies))
        # return new_domain, new_state
        new_domain = apply_constraint(
            self.pddl,
            self.actions,
            self.index_failed,
            self.implies,
            domain_str=domain_str,
        )

        return new_domain, new_state

    def __eq__(self, other) -> bool:
        return isinstance(other, CfreeConstraint) and self._edit == other._edit

    def __neq__(self, other) -> bool:
        return not (self == other)

    def __hash__(self) -> int:
        return hash(self._edit)

    def __str__(self) -> str:
        if isinstance(self._edit, tuple):
            return ",".join(
                [str(e).replace("\n", "").replace("\t", "") for e in self._edit]
            )
        else:
            return str(self._edit)
