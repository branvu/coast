from typing import Any, Sequence, Set, Tuple

import symbolic
from symbolic import problem
from symbolic import _and, _P, _when

from coast.action import ActionCall
from coast.object import Object
from coast.stream import StreamInstance
from coast import utils


class Constraint:
    def __init__(
        self,
        pddl: symbolic.Pddl,
        task_plan: Sequence[symbolic.PlannerNode],
        action_plan: Sequence[ActionCall],
        stream_plan: Sequence[StreamInstance],
        objects: Sequence[Object],
        visited_bindings,  #: Sequence[PrioritizedBindings],
        world: Any,
    ):
        stream_instance = stream_plan[visited_bindings[-1].idx_plan]
        action_call = stream_instance.action_call
        self.index_failed = action_plan.index(action_call)
        self.actions = [node.action for node in task_plan[1:]]
        self.pddl = pddl
    @staticmethod
    def _apply_problem_constraint_no_time(constraint):
        plan, index_failed = constraint
        assert index_failed == 0
        actions = plan
        cf_action_params = [str(p) for p in problem.parse_args(actions[index_failed])]
        implies = (
            f"fail{problem.parse_head(actions[index_failed]).lower()}"
            + "("
            + ", ".join(cf_action_params)
            + ")"
        )

        return set([implies])

    def _apply_domain_constraint(pddl: symbolic.Pddl, constraint, domain_str=None):
        if domain_str is None:
            filepath = pddl.domain_pddl
            txt = open(filepath, "r")
            domain_str = open(filepath, "r").read()
            txt.close()
        plan, index_failed = constraint
        actions = plan

        cf_action_params = [str(p) for p in problem.parse_args(actions[index_failed])]

        implies = problem._P(
            f"Fail{problem.parse_head(actions[index_failed]).lower()}",
            *cf_action_params,
        )

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
                    chained_and += _P("=", '?' + str(params[i]), str(current_action_params[i]))
                    if i != len(params) - 1:
                        chained_and += " "
                for log in list_log_actions:
                    chained_and += log
                condition = _and(chained_and, "")

        # Add / Create condition for previous action history

        # Create implication
        implication = problem.pprint_formula(_when(condition, implies))
        # print(implication)
        # Create new effect predicate by inserting within the and
        domain_str = domain_str.replace(
            current_effect,
            current_effect[: len(current_effect) - 1] + implication + ")",
            1,
        )
        return domain_str
    def get_implication(self, pddl, constraint, domain_str=None):
     
        plan, index_failed = constraint
        actions = plan

        cf_action_params = [str(p) for p in problem.parse_args(actions[index_failed])]

        implies = problem._P(
            f"Fail{problem.parse_head(actions[index_failed]).lower()}",
            *cf_action_params,
        )

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
                    chained_and += _P("=", '?' + str(params[i]), str(current_action_params[i]))
                    if i != len(params) - 1:
                        chained_and += " "
                for log in list_log_actions:
                    chained_and += log
                condition = _and(chained_and, "")

        # Add / Create condition for previous action history

        # Create implication
        implication = problem.pprint_formula(_when(condition, implies))
        return implication
    def __str__(self):
        constraint = (self.actions, self.index_failed)
        if self.index_failed == 0:
            new_prop = Constraint._apply_problem_constraint_no_time(
                constraint
            )
            return f"0 {new_prop}"
        # implication = self.get_implication(
        #     self.pddl, constraint
        # )
        return f"{self.index_failed} {self.actions}"
    def __repr__(self):
        return str(self)
    def apply(self, domain_str: str) -> Tuple[str, Set[str]]:
        constraint = (self.actions, self.index_failed)
        new_state = self.pddl.initial_state
        if self.index_failed == 0:
            new_prop = Constraint._apply_problem_constraint_no_time(
                constraint
            )
            new_state |= new_prop
        new_domain = Constraint._apply_domain_constraint(
            self.pddl, constraint, domain_str=domain_str
        )
        return new_domain, new_state

    @staticmethod
    def constrain_pddl(
        pddl: symbolic.Pddl, constraints: Sequence["Constraint"]
    ) -> symbolic.Pddl:
        with open(pddl.domain_pddl, "r") as f:
            domain_str = f.read()

        initial_state = set(pddl.initial_state)
        for constraint in set(constraints):
            domain_str, initial_props = constraint.apply(domain_str)
            initial_state |= initial_props  # TODO: Does this lead to issues with reversing logic
            
        constrained_pddl = symbolic.Pddl(pddl.domain_pddl, pddl.problem_pddl)
        constrained_pddl = utils.write_domain_file_from_str(
            domain_str, constrained_pddl
        )
        constrained_pddl.initial_state = initial_state
        print(initial_state)

        return constrained_pddl

class ParameterConstraint:
    def __init__(
        self,
        pddl: symbolic.Pddl,
        task_plan: Sequence[symbolic.PlannerNode],
        action_plan: Sequence[ActionCall],
        stream_plan: Sequence[StreamInstance],
        objects: Sequence[Object],
        visited_bindings,  #: Sequence[PrioritizedBindings],
        world: Any,
    ):
        stream_instance = stream_plan[visited_bindings[-1].idx_plan]
        action_call = stream_instance.action_call
        self.index_failed = action_plan.index(action_call)
        self.actions = [node.action for node in task_plan[1:]]
        self.pddl = pddl

    @staticmethod
    def _apply_problem_constraint_no_time(constraint):
        plan, index_failed = constraint
        # assert index_failed == 0
        actions = plan
        cf_action_params = [str(p) for p in problem.parse_args(actions[index_failed])]
        implies = (
            f"fail{problem.parse_head(actions[index_failed]).lower()}"
            + "("
            + ", ".join(cf_action_params)
            + ")"
        )

        return set([implies])
    def __str__(self):
        constraint = (self.actions, self.index_failed)
        new_prop = ParameterConstraint._apply_problem_constraint_no_time(
            constraint
        )
        return f"{self.index_failed} {new_prop.pop()}"
    def __repr__(self):
        return str(self)
    def apply(self, domain_str: str) -> Tuple[str, Set[str]]:
        constraint = (self.actions, self.index_failed)
        new_state = self.pddl.initial_state
        new_prop = ParameterConstraint._apply_problem_constraint_no_time(
            constraint
        )
        new_state |= new_prop
        return domain_str, new_state
