"""Lazy resource lifecycle for mutually exclusive driving and parking modes."""

from enum import Enum
from typing import Callable, Protocol


class Runtime(Protocol):
    def start(self) -> None: ...
    def stop(self) -> None: ...
    def close(self) -> None: ...


class Mode(Enum):
    DRIVE = "drive"
    PARK = "park"


class ModeCoordinator:
    def __init__(self, factories: dict[Mode, Callable[[], Runtime]]) -> None:
        self._factories = factories
        self._runtime: Runtime | None = None
        self.mode: Mode | None = None

    def select(self, mode: Mode) -> None:
        if mode not in self._factories:
            raise KeyError(f"no runtime factory for {mode}")
        self.close()
        self._runtime = self._factories[mode]()
        self.mode = mode

    def start(self) -> None:
        if self._runtime is None:
            raise RuntimeError("select a mode first")
        self._runtime.start()

    def stop(self) -> None:
        if self._runtime is not None:
            self._runtime.stop()

    def close(self) -> None:
        if self._runtime is not None:
            self._runtime.stop()
            self._runtime.close()
        self._runtime = None
        self.mode = None

