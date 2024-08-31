import collections
import signal
import time
from typing import Dict, List, Sequence
import symbolic
import numpy as np  # type: ignore
import pprint

is_running = None


def validate_plan(domain_file: str, problem_file: str, plan_file: str):
    # Create symbolic Pddl object first
    my_pddl = symbolic.Pddl(domain_file, problem_file)
    initial_state = my_pddl.initial_state
    # Parse actions in plan
    f = open(plan_file, "r")
    plan = f.readlines()
    # Apply each action in plan to pddl
    for action in plan:
        print("Checking", action)
        pprint.pprint(my_pddl.initial_state)
        if not my_pddl.is_valid_action(my_pddl.initial_state, action):
            print("Preconditions not satisfied")
            return
        else:
            # Apply action to pddl
            initial_state = my_pddl.apply_actions(initial_state, [action])
            my_pddl.initial_state = initial_state

    print("Plan succeeds")

def replace_domain_constants(domain_file: str, new_constants: str):
    with open(domain_file, 'r') as f:
        domain = f.read()
        import re
        new_domain = re.sub(r'constants.*?\)', new_constants, domain, flags=re.S)
    with open(f"{domain_file[:-5]}-edit.pddl", 'w') as file:
        file.write(new_domain)
    return f"{domain_file[:-5]}-edit.pddl"

def serialize_proposition(head: str, args: List[str]) -> str:
    return f"{head}({', '.join(args)})"


def write_domain_file_from_str(domain: str, pddl: symbolic.Pddl) -> symbolic.Pddl:
    # Write the new domain to a file
    path_to_temp_domain = "envs/temp/temp_domain.pddl"
    with open(path_to_temp_domain, "w") as f:
        f.write(domain)

    # Create new pddl object
    return symbolic.Pddl(path_to_temp_domain, pddl.problem_pddl)

def rewrite_prop(prop: str):
    clean_paren = prop[1:-1]
    parts = clean_paren.split(" ")
    return f"{parts[0]}({' '.join(parts[1:])})"
def parse_paren(input: str):
    first_paren = input.index("(")
    open = 0
    last_paren = 0
    for i in range(first_paren, len(input)):
        ch = input[i]
        if ch == "(":
            open += 1
        elif ch == ")":
            open -= 1
        if open == 0:
            last_paren = i
            break
    return input[first_paren : last_paren + 1], first_paren, last_paren


def create_signal_handler():
    global is_running

    if is_running is not None:
        return is_running

    is_running = [True]
    signal.signal(signal.SIGINT, lambda *_: is_running.clear())
    return is_running


class Timer:
    """Timer to keep track of timing intervals for different keys."""

    def __init__(self):
        self._tics = {}

    def keys(self) -> Sequence[str]:
        """Timer keys."""
        return self._tics.keys()

    def tic(self, key: str) -> float:
        """Starts timing for the given key.

        Args:
            key: Time interval key.

        Returns:
            Current time.
        """
        self._tics[key] = time.time()
        return self._tics[key]

    def toc(self, key: str, set_tic: bool = False, stdout: bool = False) -> float:
        """Returns the time elapsed since the last tic for the given key.

        Args:
            key: Time interval key.
            set_tic: Reset the tic to the current time.
            stdout: Print the recorded time.

        Returns:
            Time elapsed since the last tic.
        """
        toc = time.time()
        tic = self._tics[key]
        if set_tic:
            self._tics[key] = toc
        dt = toc - tic
        if stdout:
            print(f"{key}: {dt}s")
        return dt


class Profiler(Timer):
    """Profiler to keep track of average time interval for different keys."""

    class ProfilerContext:
        """Context manager for timing code inside a `with` block."""

        def __init__(self, profiler: "Profiler", key: str):
            self.profiler = profiler
            self.key = key

        def __enter__(self) -> float:
            self.tic = time.time()
            return self.tic

        def __exit__(self, type, value, traceback) -> None:
            tictoc = time.time() - self.tic
            self.profiler._tictocs[self.key].append(tictoc)

    def __init__(self, disabled: bool = False):
        """Initializes the profiler with the given status.

        Args:
            disabled: Disable the profiler.
        """
        super().__init__()
        self._disabled = disabled
        self._tictocs: Dict[str, List[float]] = collections.defaultdict(list)

    def disable(self) -> None:
        """Disables the profiler so that tic and toc do nothing."""
        self._disabled = True

    def enable(self) -> None:
        """Enables the profiler."""
        self._disabled = False

    def tic(self, key: str) -> float:
        """Starts timing for the given key.

        Args:
            key: Time interval key.

        Returns:
            Current time.
        """
        if self._disabled:
            return 0.0
        return super().tic(key)

    def toc(self, key: str, set_tic: bool = False, stdout: bool = False) -> float:
        """Returns the time elapsed since the last tic for the given key.

        Args:
            key: Time interval key.
            set_tic: Reset the tic to the current time.

        Returns:
            Time elapsed since the last tic.
        """
        if self._disabled:
            return 0.0
        tictoc = super().toc(key, set_tic)
        self._tictocs[key].append(tictoc)
        if stdout:
            print(f"{key}: {tictoc}s")
        return tictoc

    def profile(self, key: str) -> ProfilerContext:
        """Times the code inside a `with` block for the given key.

        Args:
            key: Time interval key.

        Returns:
            Profiler context.
        """
        return Profiler.ProfilerContext(self, key)

    def get_last(self, key: str) -> float:
        return self._tictocs[key][-1]

    def compute_average(self, key: str, reset: bool = False) -> float:
        """Computes the average time interval for the given key.

        Args:
            key: Time interval key.
            reset: Reset the collected time intervals.

        Returns:
            Average time interval.
        """
        mean = float(np.mean(self._tictocs[key]))
        if reset:
            self._tictocs[key] = []
        return mean

    def compute_sum(self, key: str, reset: bool = False) -> float:
        """Computes the sum of all time intervals for the given key.
        Args:
            key: Time interval key.
            reset: Reset the collected time intervals.

        Returns:
            Sum of time intervals
        """
        res = np.sum(self._tictocs[key])
        if reset:
            self._tictocs[key] = []
        return res

    def collect_profiles(self) -> Dict[str, float]:
        """Collects and resets the average time intervals for all keys.

        Returns:
            Dict mapping from key to average time interval.
        """
        return {
            key: self.compute_average(key, reset=True)
            for key, tictoc in self._tictocs.items()
            if len(tictoc) > 0
        }
