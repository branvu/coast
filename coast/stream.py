import abc
import pprint
import random
from typing import (
    Dict,
    Generic,
    Mapping,
    Optional,
    Set,
    Tuple,
    TypeVar,
    Any,
    List,
    Union,
)

import symbolic
from coast.action import ActionCall
from coast.object import Object, StreamObject
from coast.formula import _P


class StreamConstraint(abc.ABC):
    def __init__(self, constraint, constraint_type):
        self.constraint = constraint
        self.constraint_type = constraint_type


class Stream(abc.ABC):
    name: str
    inputs: List[symbolic.Object]
    outputs: List[symbolic.Object]
    fluents: List[str]

    def __init__(
        self,
        world: Any,
        resample_p: float = 0,
        resample_p_fail: float = 0
        # stream_pddl: symbolic.Pddl,
        # apply_all: bool = False,
    ):
        # self._inputs = self.domain(stream_pddl).parameters
        # inputs = set(param.name for param in self._inputs)
        # self._outputs = [
        #     param
        #     for param in self.certified(stream_pddl).parameters
        #     if param.name not in inputs
        # ]

        self._stream_instances: Dict[Tuple[str, ...], StreamInstance] = {}
        self._resample_p = resample_p  # Resample probability on success
        self._resample_p_fail = resample_p_fail
        # self.apply_all = apply_all
        self.certified = _P(
            self.name,
            *[arg.name for arg in self.inputs],
            *[arg.name for arg in self.outputs],
        )

    def resample(self, success=True) -> bool:
        # TODO: Verify if this is true
        # return len(self.outputs) == 0
        if success:
            return random.random() < self._resample_p
        else:
            return random.random() < self._resample_p_fail

    @abc.abstractmethod
    def sample(
        self, inputs: Dict[str, Any], fluents: Dict[str, List[Dict[str, Any]]]
    ) -> Optional[Union[Dict[str, Any], StreamConstraint]]:
        return None

    def create_instance(
        self,
        inputs: Dict[str, Object],
        outputs: Dict[str, Object],
        action_call: ActionCall,
        fluents: Dict[str, List[Dict[str, Object]]],
        # stream_pddl: symbolic.Pddl,
    ) -> "StreamInstance":
        # Return stream instance from cache if it has already been created.
        # idx_stream_instance = tuple(obj.name for obj in inputs.values())
        # try:
        #     return self._stream_instances[idx_stream_instance]
        # except KeyError:
        #     pass

        # Create a new stream instance.
        stream_instance = StreamInstance(
            self, inputs, outputs, action_call, fluents
        )
        # self._stream_instances[idx_stream_instance] = stream_instance

        return stream_instance

    # def action_name(self, action_type: str) -> str:
    #     if action_type not in ("pre", "post"):
    #         raise ValueError("Action type must be pre or post")
    #     return f"{self.name}_{action_type}"
    #
    # def domain(self, stream_pddl: symbolic.Pddl) -> symbolic.Action:
    #     return next(
    #         a
    #         for a in stream_pddl.actions
    #         if a.name.lower() == self.action_name("pre").lower()
    #     )

    # def domain_call(self, inputs: Sequence[Object]) -> str:
    #     return f"{self.action_name('pre')}({', '.join(map(str, inputs))})"

    # def certified(self, stream_pddl: symbolic.Pddl) -> symbolic.Action:
    #     return next(
    #         a
    #         for a in stream_pddl.actions
    #         if a.name.lower() == self.action_name("post").lower()
    #     )
    #
    # def certified_call(
    #     self, inputs: Sequence[Object], outputs: Sequence[Object]
    # ) -> str:
    #     args = list(map(str, inputs)) + list(map(str, outputs))
    #     return f"{self.action_name('post')}({', '.join(args)})"

    # def valid_inputs(
    #     self, state: Set[str], stream_pddl: symbolic.Pddl
    # ) -> Iterator[Tuple[symbolic.Object]]:
    #     for inputs in self.domain(stream_pddl).parameter_generator:
    #         if stream_pddl.is_valid_action(state, self.domain_call(inputs)):
    #             yield inputs


class StreamInstance:
    def __init__(
        self,
        stream: Stream,
        inputs: Dict[str, Object],
        outputs: Dict[str, Object],
        action_call: ActionCall,
        fluents: Dict[str, List[Dict[str, Object]]],
        # stream_pddl: symbolic.Pddl,
        # stream_state: "StreamState",
        # apply_all: bool = False,
    ):
        self._stream = stream
        self._inputs = inputs
        self._outputs = outputs
        self._action_call = action_call
        self._fluents = fluents
        self._fluent_inputs: Dict[str, Object] = {}
        for fluent_args in fluents.values():
            for args_map in fluent_args:
                for arg in args_map.values():
                    self._fluent_inputs[arg.name] = arg
        # self._stream_state = stream_state
        # self.apply_all = apply_all

        # TODO: was commented out?
        # pre_dnf = symbolic.DisjunctiveFormula.normalize_preconditions(
        #     stream_pddl, stream.domain_call(inputs).lower()
        # )
        # assert pre_dnf is not None and len(pre_dnf.conjunctions) <= 1
        # if len(pre_dnf.conjunctions) > 0:
        #     pre_conj = pre_dnf.conjunctions[0]
        #     self._domain = set(
        #         prop
        #         for prop in pre_conj.pos | pre_conj.neg
        #         if symbolic.parse_head(prop) != "eq"
        #     )
        # else:
        #     self._domain = set()

        """
        # TODO: Adding pddl objects dynamically is prone to segfaults.
        added_objects = []
        for obj in outputs:
            if obj in stream_pddl.objects:
                continue
            stream_pddl.add_object(obj.name, obj.type)
            added_objects.append(obj)
        post_dnf = symbolic.DisjunctiveFormula.normalize_postconditions(
            stream_pddl,
            stream.certified_call(inputs, outputs).lower(),
        )
        assert post_dnf is not None and len(post_dnf.conjunctions) <= 1
        if len(post_dnf.conjunctions) > 0:
            post_conj = post_dnf.conjunctions[0]
            self._certified = post_conj.pos | post_conj.neg
        else:
            self._certified = set()
        for obj in added_objects:
            stream_pddl.remove_object(obj.name)
        """
        self._count = 0
        self._lazy = False

    # @staticmethod
    # def create_with_new_inputs(
    #     stream_instance: "StreamInstance", inputs: Sequence[Object]
    # ) -> "StreamInstance":
    #     new_stream_instance = copy.copy(stream_instance)
    #     new_stream_instance._inputs = inputs
    #     return new_stream_instance

    @property
    def stream(self) -> Stream:
        return self._stream

    @property
    def inputs(self) -> Dict[str, Object]:
        return self._inputs

    @property
    def fluent_inputs(self) -> Dict[str, Object]:
        return self._fluent_inputs

    @property
    def outputs(self) -> Dict[str, Object]:
        return self._outputs

    @property
    def action_call(self) -> ActionCall:
        return self._action_call

    # @property
    # def stream_state(self) -> "StreamState":
    #     return self._stream_state

    @property
    def count(self) -> int:
        return self._count

    # @property
    # def domain(self) -> Set[str]:
    #     return self._domain

    # @property
    # def certified(self) -> Set[str]:
    #     return self._certified

    def clear_count(self):
        self._count = 0

    def __repr__(self) -> str:
        try:
            return self._repr  # type: ignore
        except AttributeError:
            self._repr = f"{self._stream.name}({', '.join(map(repr, self.inputs.values()))}) -> {list(self.outputs.values())}"
            return self._repr

    def __eq__(self, other: object) -> bool:
        return str(self) == str(other)

    def __ne__(self, other: object) -> bool:
        return not (self == other)

    def __hash__(self) -> int:
        try:
            return self._hash  # type: ignore
        except AttributeError:
            self._hash = hash(str(self))
            return self._hash

    def __iter__(self):
        return self

    # def __next__(self) -> Optional[Dict[str, Object]]:
    #     # Increment count only if not already incremented by lazy_sample().
    #     if not self._lazy and not self.apply_all:
    #         self._count += 1
    #     self._lazy = False
    #
    #     return self._stream.sample(self.inputs, self._outputs, self._stream_state)

    """
    This next function is intended for passing a previous state to the sample
    function
    """

    def next(
        self, objects: Mapping[str, Object]
    ) -> Optional[Union[Mapping[str, Object], StreamConstraint]]:
        self._count += 1
        self._lazy = False

        # for object in objects.values():
        #     print("OBJECTS", object.name, object.value)
        # print(self.inputs)
        outputs = self._stream.sample(
            inputs={
                param.name[1:]: objects[self.inputs[param.name[1:]].name].value
                for param in self._stream.inputs
                # TODO: Remove ? from param name
            },
            fluents={
                pred: [
                    {param[1:]: objects[arg.name].value for param, arg in args.items()}
                    for args in fluent_args
                ]
                for pred, fluent_args in self._fluents.items()
            },
        )
        if isinstance(outputs, StreamConstraint):
            return outputs
        if outputs is None:
            return None

        new_objects = dict(objects)
        for param, value in outputs.items():
            arg = self.outputs[param]
            new_objects[arg.name] = StreamObject(
                name=arg.name, object_type=arg.type, value=value
            )
        return new_objects

    # def lazy_sample(self) -> Dict[str, StreamObject]:
    #     # Increment count only if the last sample was not also lazy.
    #     if not self._lazy and not self.apply_all:
    #         self._count += 1
    #     self._lazy = True
    #
    #     return {
    #         obj.name: StreamObject(obj.name, obj.type, self) for obj in self._outputs
    #     }


T = TypeVar("T")


class Certified(Generic[T]):
    def __init__(
        self,
        fact: T,
        level: int = 0,
        stream_instance: Optional[StreamInstance] = None,
    ):
        self.fact = fact
        self.level = level
        self.stream_instance = stream_instance

    def __repr__(self) -> str:
        return (
            "Certified(\n"
            f"  fact: {self.fact}\n"
            f"  level: {self.level}\n"
            f"  stream_instance: {self.stream_instance}\n"
            ")"
        )


class CertifiedFact(Certified[str]):
    pass


class CertifiedObject(Certified[StreamObject]):
    pass


class StreamState:
    def __init__(self, pddl: Optional[symbolic.Pddl] = None):
        if pddl is not None:
            self._propositions = {
                prop: CertifiedFact(prop) for prop in pddl.initial_state
            }
            self._objects = {
                obj.name: CertifiedObject(StreamObject(obj.name, obj.type))
                for obj in pddl.objects
            }
        else:
            self._propositions = {}
            self._objects = {}

    @property
    def propositions(self) -> Dict[str, CertifiedFact]:
        return self._propositions

    @property
    def objects(self) -> Dict[str, CertifiedObject]:
        return self._objects

    def pddl_state(self) -> Set[str]:
        return set(sorted(self.propositions.keys()))

    def pddl_objects(self) -> Set[StreamObject]:
        return set(sorted([obj.fact for obj in self.objects.values()]))

    # def copy(self) -> "StreamState":
    #     stream_state = StreamState()
    #     for prop, fact in self.propositions.items():
    #         stream_state.propositions[prop] = CertifiedFact(
    #             prop, fact.level, fact.stream_instance
    #         )
    #     for obj in self.objects.values():
    #         stream_state.objects[obj.fact.name] = CertifiedObject(
    #             obj.fact.copy(), obj.level, obj.stream_instance
    #         )
    #     return stream_state

    # def satisfies(self, stream_instance: StreamInstance) -> bool:
    #     for obj in stream_instance.inputs:
    #         if obj not in self.objects:
    #             return False
    #     for prop in stream_instance.domain:
    #         if prop not in self.propositions:
    #             return False
    #     return True

    def __repr__(self) -> str:
        pp = pprint.PrettyPrinter(indent=4)
        return (
            "StreamState(\n"
            f"  objects: {len(self.objects.keys())} {pp.pformat(set(self.objects.keys()))}\n"
            f"  propositions: {len(self.pddl_state())} {pp.pformat(self.pddl_state())}\n"
            ")"
        )
