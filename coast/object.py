from typing import Any
import abc


class GeoParam(abc.ABC):
    name: str

    def __init__(self):
        pass


class Object:
    def __init__(self, name: str, object_type: str, value: Any = None):
        self.name = name
        self.type = object_type
        self._value = value

    @property
    def value(self) -> Any:
        return self._value

    # @value.setter
    # def value(self, value):
    #     self._value = value

    def __str__(self) -> str:
        return f"{self.name}"

    def __repr__(self) -> str:
        return f"{self.name}: {self.type}"
        return f"{self.name}: {self.type} = {self.value}"

    def __eq__(self, other: object) -> bool:
        return str(self) == str(other)

    def __ne__(self, other: object) -> bool:
        return str(self) != str(other)

    def __lt__(self, other: object) -> bool:
        return str(self) < str(other)

    def __le__(self, other: object) -> bool:
        return str(self) <= str(other)

    def __hash__(self) -> int:
        return hash(str(self))

    def copy(self) -> "Object":
        return Object(self.name, self.type)  # , self._value)


"""
Note: Instead of using stream objects to hold values, we will just use objects
"""


class StreamObject(Object):
    def __init__(
        self,
        name: str,
        object_type: str,
        value: Any = None,
    ):
        super().__init__(name=name, object_type=object_type, value=value)

    def __repr__(self) -> str:
        return f"{super().__repr__()} = {self.value}"

    def copy(self) -> "StreamObject":
        return StreamObject(self.name, self.type, self.value)
