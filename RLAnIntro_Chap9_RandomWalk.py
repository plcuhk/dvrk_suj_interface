import numpy as np
import matplotlib.pyplot as plt
from tqdm import tqdm

N_STATES = 1000
STATES = np.arange(1, N_STATES + 1)
START_STATE = 500
END_STATES =[0, N_STATES + 1]

ACTION_LEFT = -1
ACTION_RIGHT = 1
ACTIONS = [ACTION_LEFT, ACTION_RIGHT]

STEP_RANGE = 100

def compute_true_value():
    # A guess of the true value function,
    true_value = np.arange(-1001, 1003, 2) / 1001

    # Dynamic programming to get the value function. Import assumption is that the
    # values for 2 terminal state is given, other reward is 0.
    while True:
        old_value = np.copy(true_value)
        for state in STATES:
            true_value[state] = 0
            for action in ACTIONS:
                for step in range(1, STEP_RANGE + 1):
                    step *= action
                    next_state = state + step
                    next_state = max(min(next_state, N_STATES + 1), 0)
                    true_value[state] += 1.0 / (2 * STEP_RANGE) * true_value[next_state]
        error = np.sum(np.abs(old_value - true_value))
        if error < 1e-2:
            break

    true_value[0] = true_value[-1] = 0

    return true_value


def step(state, action):
    step = np.random.randint(1, STEP_RANGE + 1)
    step *= action
    state += step
    state = max(min(state, N_STATES + 1), 0)

    if state == 0:
        reward = -1
    elif state == N_STATES + 1:
        reward = 1
    else:
        reward = 0
    return state, reward

def get_action():
    if np.random.binomial(1, 0.5) == 1:
        return 1
    return -1


class ValueFunction:
    def __init__(self, num_of_groups):
        self.num_of_groups = num_of_groups
        self.group_size = N_STATES // num_of_groups

        self.params = np.zeros(num_of_groups)

    def value(self, state):
        if state in END_STATES:
            return 0
        group_index = (state - 1) // self.group_size
        return self.params[group_index]

    def update(self, delta, state):
        group_index = (state - 1) // self.group_size
        self.params[group_index] += delta

