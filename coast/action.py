from typing import Any, Dict, Iterable, List, Mapping, Set

import symbolic

from coast.formula import Formula, _P, _and
from coast.object import Object


def find_closest_match(prop: str, state: Iterable[str]) -> str:
    pred, args = symbolic.parse_proposition(prop)
    best_num_matches = -1
    best_prop = None
    for state_prop in state:
        state_pred, state_args = symbolic.parse_proposition(state_prop)
        if state_pred != pred:
            continue

        num_matches = len(set(args).intersection(state_args))
        if num_matches > best_num_matches:
            best_num_matches = num_matches
            best_prop = state_prop

    if best_prop is None:
        raise ValueError("Matching predicate not found.")

    return best_prop


class Action:
    parameters: List[Object]
    """PDDL action parameters."""

    inputs: List[Object]
    """Geometric action inputs."""

    outputs: List[Object]
    """Geometric action outputs."""

    precondition: Formula
    """Geometric preconditions.

    If the geometric state doesn't satisfy these preconditions, then stream
    planning for this action won't be attempted.
    """

    effect: Formula
    """Geometric effects.

    Changes to the geometric state.
    """

    certified: Formula
    """Certified facts.

    In contrast to the geometric state, certified facts don't change during the
    execution of a plan. Their values are determined by streams.
    """

    def __init__(self, world: Any):
        pass

    def map_inputs_to_objects(
        self, action_call: str, state: Iterable[str]
    ) -> Dict[str, str]:
        """Infers action args from satisfied preconditions in the geometric state."""
        assert isinstance(self.precondition, _and)
        # Map action parameters to action call args.
        action_args = symbolic.parse_args(action_call)
        inputs_map = {
            param.name: arg for param, arg in zip(self.parameters, action_args)
        }
        for pre_prop in self.precondition.children:
            if not isinstance(pre_prop, _P):
                raise NotImplementedError
            # Apply parameter mapping to preconditions.
            partial_args = [inputs_map.get(param, param) for param in pre_prop.args]
            partial_prop = f"{pre_prop.predicate}({', '.join(partial_args)})"
            prop = find_closest_match(partial_prop, state)

            for pre_arg, arg in zip(pre_prop.args, symbolic.parse_args(prop)):
                if pre_arg in inputs_map:
                    if arg != inputs_map[pre_arg]:
                        raise RuntimeError(
                            f"Mapping conflict: {pre_arg} already mapped to {inputs_map[pre_arg]}, unable to map to {arg}."
                        )
                else:
                    inputs_map[pre_arg] = arg
        if any(input.name not in inputs_map for input in self.inputs):
            raise RuntimeError(
                f"Unmapped input: inputs {self.inputs}, map {inputs_map}"
            )

        return inputs_map

    def execute_state(
        self,
        state: Any,
        problem: Any,
        robot: Any,
        arm: Any,
        inputs: Dict[str, Any],
        outputs: Dict[str, Any],
        objects,
        args_map,
    ) -> bool:
        return False

    def execute(self, inputs: Dict[str, Any], outputs: Dict[str, Any]) -> bool:
        return False

    def __repr__(self) -> str:
        return (
            f"{type(self).__name__}({self.inputs}; {self.parameters}) -> {self.outputs}"
        )


class ActionCall:
    def __init__(self, action: Action, args_map: Dict[str, str]):
        self.action = action
        self.args_map = args_map

    def apply(
        self,
        state: Dict[str, None],
        objects: List[Object],
    ) -> Dict[str, None]:
        state = dict(state)
        self.action.effect.apply(self.args_map, state, objects)
        return state

    def get_certified_facts(
        self, state: Dict[str, None], objects: List[Object]
    ) -> Dict[str, None]:
        new_state = dict(state)
        self.action.certified.apply(self.args_map, new_state, objects)
        new_state = {k: None for k in new_state if k not in state}
        return new_state

    @property
    def input_names(self) -> Mapping[str, str]:
        return {param.name: self.args_map[param.name] for param in self.action.inputs}

    @property
    def parameter_names(self) -> Mapping[str, str]:
        return {
            param.name: self.args_map[param.name] for param in self.action.parameters
        }

    @property
    def output_names(self) -> Mapping[str, str]:
        return {param.name: self.args_map[param.name] for param in self.action.outputs}

    def inputs(self, objects: Mapping[str, Object]) -> Mapping[str, Any]:
        return {param: objects[arg].value for param, arg in self.input_names.items()}

    def outputs(self, objects: Mapping[str, Object]) -> Mapping[str, Any]:
        return {param: objects[arg].value for param, arg in self.output_names.items()}

    def __repr__(self) -> str:
        return f"{type(self.action).__name__}({', '.join(self.input_names.values())}; {', '.join(self.parameter_names.values())})"
        return f"ActionCall(action={self.action}, args_map={self.args_map})"

    def execute_state(self, state, problem, robot, arm, objects: Mapping[str, Object]) -> bool:
        # TODO: Remove ? from param name
        inputs = {param[1:]: arg for param, arg in self.inputs(objects).items()}
        outputs = {param[1:]: arg for param, arg in self.outputs(objects).items()}

        return self.action.execute_state(
            state, problem, robot, arm, inputs, outputs, objects, self.args_map
        )

    def execute(self, objects: Mapping[str, Object]) -> bool:
        # TODO: Remove ? from param name
        inputs = {param[1:]: arg for param, arg in self.inputs(objects).items()}
        outputs = {param[1:]: arg for param, arg in self.outputs(objects).items()}
        return self.action.execute(inputs, outputs)
