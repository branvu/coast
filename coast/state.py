import copy
import collections
from typing import Any, Iterable, Sequence, Tuple


class GeometricState:
    def __init__(self):
        self._state = collections.defaultdict(dict)

    def set(self, key: str, timestep: int, value: Any) -> None:
        self._state[key][timestep] = value

    def get(self, key: str, timestep: int) -> Any:
        return self._state[key][timestep]

    def get_timesteps(self, key: str) -> Sequence[int]:
        return sorted(self._state[key].keys())

    def propagate_value(self, key: str, timestep: int, value: Any) -> None:
        for t in reversed(self.get_timesteps(key)):
            if t < timestep:
                break
            if self.get(key, t) == self.get(key, timestep):
                self.set(key, t, value)

    def keys(self) -> Iterable[str]:
        return self._state.keys()

    def items(self, timestep: int) -> Iterable[Tuple[str, Any]]:
        return ((key, val[timestep]) for key, val in self._state.items() if timestep in val)

    def copy(self) -> "GeometricState":
        return copy.deepcopy(self)
