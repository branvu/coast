import abc
from typing import Dict, List, Set
from coast.object import Object


class Formula(abc.ABC):
    @abc.abstractmethod
    def evaluate(
        self, args_map: Dict[str, str], state: Dict[str, None], objects: List[Object]
    ) -> bool:
        pass

    @abc.abstractmethod
    def apply(
        self, args_map: Dict[str, str], state: Dict[str, None], objects: List[Object]
    ) -> None:
        pass


class _and(Formula):
    def __init__(self, *children: Formula):
        self.children = children

    def evaluate(
        self, args_map: Dict[str, str], state: Dict[str, None], objects: List[Object]
    ) -> bool:
        return all(child.evaluate(args_map, state, objects) for child in self.children)

    def apply(
        self, args_map: Dict[str, str], state: Dict[str, None], objects: List[Object]
    ) -> None:
        for child in self.children:
            child.apply(args_map, state, objects)


class _P(Formula):
    def __init__(self, predicate: str, *args: str):
        self.predicate = predicate
        self.args = args

    def to_string(self, args_map: Dict[str, str]) -> str:
        # input(f"{args_map} and {self.args}")
        return f"{self.predicate}({', '.join(args_map[param] for param in self.args)})"

    def evaluate(
        self, args_map: Dict[str, str], state: Dict[str, None], objects: List[Object]
    ) -> bool:
        return self.to_string(args_map) in state

    def apply(
        self, args_map: Dict[str, str], state: Dict[str, None], objects: List[Object]
    ) -> None:
        state[self.to_string(args_map)] = None
        
    def __repr__(self) -> str:
        return f"{self.predicate}({', '.join(self.args)})"


class _not(Formula):
    def __init__(self, child: Formula):
        self.child = child

    def evaluate(
        self, args_map: Dict[str, str], state: Dict[str, None], objects: List[Object]
    ) -> bool:
        return not self.child.evaluate(args_map, state, objects)

    def apply(
        self, args_map: Dict[str, str], state: Dict[str, None], objects: List[Object]
    ) -> None:
        if not isinstance(self.child, _P):
            raise NotImplementedError
        del state[self.child.to_string(args_map)]


class _forall(Formula):
    def __init__(self, arg: str, arg_type: str, child: Formula):
        self.arg = Object(f"?{arg}", arg_type)
        self.child = child

    def evaluate(
        self, args_map: Dict[str, str], state: Dict[str, None], objects: List[Object]
    ) -> bool:
        for obj in objects:
            if self.arg.type is None or self.arg.type == obj.type:
                forall_args_map = dict(args_map)
                forall_args_map[self.arg.name] = obj.name
                if not self.child.evaluate(forall_args_map, state, objects):
                    return False
        return True

    def apply(
        self, args_map: Dict[str, str], state: Dict[str, None], objects: List[Object]
    ) -> None:
        for obj in objects:
            if self.arg.type is None or self.arg.type == obj.type:
                forall_args_map = dict(args_map)
                forall_args_map[self.arg.name] = obj.name
                self.child.apply(forall_args_map, state, objects)


class _when(Formula):
    def __init__(self, condition: Formula, effect: Formula):
        self.condition = condition
        self.effect = effect

    def evaluate(
        self, args_map: Dict[str, str], state: Dict[str, None], objects: List[Object]
    ) -> bool:
        if self.condition.evaluate(args_map, state, objects):
            return self.effect.evaluate(args_map, state, objects)
        return True

    def apply(
        self, args_map: Dict[str, str], state: Dict[str, None], objects: List[Object]
    ) -> None:
        if self.condition.evaluate(args_map, state, objects):
            self.effect.apply(args_map, state, objects)
