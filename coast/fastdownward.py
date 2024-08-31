from typing import List, Optional, NamedTuple, Set

from os.path import exists
import subprocess

import symbolic


class PlannerNode(NamedTuple):
    action: str
    state: Set[str]


def get_lama(**kwargs):
    return [
        "--if-unit-cost",
        "--evaluator",
        "hlm=lmcount(lm_reasonable_orders_hps(lm_rhw()),pref={pref})".format(**kwargs),
        "--evaluator",
        "hff=ff()",
        "--search",
        """iterated([
                         lazy_greedy([hff,hlm],preferred=[hff,hlm]),
                         lazy_wastar([hff,hlm],preferred=[hff,hlm],w=5),
                         lazy_wastar([hff,hlm],preferred=[hff,hlm],w=3),
                         lazy_wastar([hff,hlm],preferred=[hff,hlm],w=2),
                         lazy_wastar([hff,hlm],preferred=[hff,hlm],w=1)
                         ],repeat_last=true,continue_on_fail=true,bound=16)""",
        "--if-non-unit-cost",
        "--evaluator",
        "hlm1=lmcount(lm_reasonable_orders_hps(lm_rhw()),transform=adapt_costs(one),pref={pref})".format(
            **kwargs
        ),
        "--evaluator",
        "hff1=ff(transform=adapt_costs(one))",
        "--evaluator",
        "hlm2=lmcount(lm_reasonable_orders_hps(lm_rhw()),transform=adapt_costs(plusone),pref={pref})".format(
            **kwargs
        ),
        "--evaluator",
        "hff2=ff(transform=adapt_costs(plusone))",
        "--search",
        """iterated([
                         lazy_greedy([hff1,hlm1],preferred=[hff1,hlm1], cost_type=one,reopen_closed=false),
                         lazy_greedy([hff2,hlm2],preferred=[hff2,hlm2], reopen_closed=false),
                         lazy_wastar([hff2,hlm2],preferred=[hff2,hlm2],w=5),
                         lazy_wastar([hff2,hlm2],preferred=[hff2,hlm2],w=3),
                         lazy_wastar([hff2,hlm2],preferred=[hff2,hlm2],w=2),
                         lazy_wastar([hff2,hlm2],preferred=[hff2,hlm2],w=1)
                         ],repeat_last=true,continue_on_fail=true,bound=16),""",
        # Append --always to be on the safe side if we want to append
        # additional options later.
        "--always",
    ]
def get_ff(task_horizon=50):
    return [
        '--evaluator',
         "h=ff(transform=adapt_costs(cost_type=PLUSONE))",
            '--search',
            f"eager(alt([single(sum([g(), weight(h,2)])), single(sum([g(),weight(h,2)]),pref_only=true)]), preferred=[h],cost_type=PLUSONE,bound={task_horizon})"
    ]

def get_lama_first():
    return [
        "--evaluator",
        "hlm=lmcount(lm_factory=lm_reasonable_orders_hps(lm_rhw()),transform=adapt_costs(one),pref=false)",
        "--evaluator",
        "hff=ff(transform=adapt_costs(one))",
        "--search",
        """lazy_greedy([hff,hlm],preferred=[hff,hlm],
                               cost_type=one,reopen_closed=false, bound=50)""",
    ]


def get_seq_sat_fd_autotune_1():
    return [
        "--evaluator",
        "hff=ff(transform=adapt_costs(one))",
        "--evaluator",
        "hcea=cea()",
        "--evaluator",
        "hcg=cg(transform=adapt_costs(plusone))",
        "--evaluator",
        "hgc=goalcount()",
        "--evaluator",
        "hAdd=add()",
        "--search",
        """iterated([
lazy(alt([single(sum([g(),weight(hff,10)])),
          single(sum([g(),weight(hff,10)]),pref_only=true)],
         boost=2000),
     preferred=[hff],reopen_closed=false,cost_type=one),
lazy(alt([single(sum([g(),weight(hAdd,7)])),
          single(sum([g(),weight(hAdd,7)]),pref_only=true),
          single(sum([g(),weight(hcg,7)])),
          single(sum([g(),weight(hcg,7)]),pref_only=true),
          single(sum([g(),weight(hcea,7)])),
          single(sum([g(),weight(hcea,7)]),pref_only=true),
          single(sum([g(),weight(hgc,7)])),
          single(sum([g(),weight(hgc,7)]),pref_only=true)],
         boost=1000),
     preferred=[hcea,hgc],reopen_closed=false,cost_type=one),
lazy(alt([tiebreaking([sum([g(),weight(hAdd,3)]),hAdd]),
          tiebreaking([sum([g(),weight(hAdd,3)]),hAdd],pref_only=true),
          tiebreaking([sum([g(),weight(hcg,3)]),hcg]),
          tiebreaking([sum([g(),weight(hcg,3)]),hcg],pref_only=true),
          tiebreaking([sum([g(),weight(hcea,3)]),hcea]),
          tiebreaking([sum([g(),weight(hcea,3)]),hcea],pref_only=true),
          tiebreaking([sum([g(),weight(hgc,3)]),hgc]),
          tiebreaking([sum([g(),weight(hgc,3)]),hgc],pref_only=true)],
         boost=5000),
     preferred=[hcea,hgc],reopen_closed=false,cost_type=normal),
eager(alt([tiebreaking([sum([g(),weight(hAdd,10)]),hAdd]),
           tiebreaking([sum([g(),weight(hAdd,10)]),hAdd],pref_only=true),
           tiebreaking([sum([g(),weight(hcg,10)]),hcg]),
           tiebreaking([sum([g(),weight(hcg,10)]),hcg],pref_only=true),
           tiebreaking([sum([g(),weight(hcea,10)]),hcea]),
           tiebreaking([sum([g(),weight(hcea,10)]),hcea],pref_only=true),
           tiebreaking([sum([g(),weight(hgc,10)]),hgc]),
           tiebreaking([sum([g(),weight(hgc,10)]),hgc],pref_only=true)],
          boost=500),
      preferred=[hcea,hgc],reopen_closed=true,cost_type=normal, bound=25)
],repeat_last=true,continue_on_fail=true)""",
    ]


def task_plan(
    problem_state: str, pddl: symbolic.Pddl, task_horizon: int = 50
) -> Optional[List[symbolic.PlannerNode]]:

    path_to_problem = "envs/temp/temp_problem.pddl"

    problem_file = open(path_to_problem, "w")
    problem_file.write(problem_state)
    problem_file.close()
    standard_config = [
        "python3",
        "external/downward/fast-downward.py",
        "--plan-file",
        "envs/temp/final_plan.txt",
        "--log-level",
        "warning",
        #  "/scr/vubt/og-pddlstream/pddlstream/temp/domain.pddl",
        #  "/scr/vubt/og-pddlstream/pddlstream/temp/problem.pddl"
        pddl.domain_pddl,
        path_to_problem,
    ]
            
    # standard_config.extend(get_lama(pref="true"))
    # standard_config.extend(get_lama_first())
    standard_config.extend(get_ff(task_horizon=task_horizon))
    # standard_config.extend(get_seq_sat_fd_autotune_1())
    # standard_config.extend(["--search","lazy_greedy([ff()], preferred=[ff()])"])
    # standard_config.extend(["--heuristic", "h=ff(transform=adapt_costs(cost_type=PLUSONE))", "--search","astar(h,cost_type=PLUSONE,bound=50)"])
    # standard_config.extend(["--alias", "seq-sat-fdss-2", "--evaluator", "hcea=cea()", "--search", "lazy_greedy([hcea], preferred=[hcea], bound=18)", "--overall-time-limit", "30m"])
    subprocess.run(
        standard_config,
        # "--evaluator",
        # "hcea=cea()",
    # "--search",
        # "lazy_greedy([hcea], preferred=[hcea], bound=18)"
        # "--cleanup",
        # "--alias",
        # "seq-sat-fdss-2",
        # "seq-sat-lama-2011",
        # "--overall-time-limit",
        # "30m",
        stdout=subprocess.DEVNULL,
        # stderr=subprocess.DEVNULL,
    )

    plan_found = exists("envs/temp/final_plan.txt")
    if not plan_found:
        return None

    plan_file = open("envs/temp/final_plan.txt", "r")  # Not sure why it adds a .1
    actions = plan_file.readlines()
    state = pddl.initial_state
    task_plan = [PlannerNode("", state)]
    for a in actions[:-1]:
        all_symbols = a.strip()[1:-1].split(" ")
        a = f"{all_symbols[0]}({','.join(all_symbols[1:])})"
        state = pddl.next_state(state, a)
        task_plan.append(PlannerNode(a, state))

    return task_plan
