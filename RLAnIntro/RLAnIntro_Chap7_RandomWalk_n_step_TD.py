import numpy as np
import matplotlib.pyplot as plt
from tqdm import tqdm

STATES_NUM = 19
STATES = np.arange(1, STATES_NUM + 1)
STATE_START = 10
STATE_TERMINAL = [0, STATES_NUM + 1]
VALUES = np.arange(1, STATES_NUM + 1) / (STATES_NUM + 1)

GAMMA = 0.8

def temporal_difference(value, n, alpha):
    state = STATE_START
    states = [state]
    rewards = [0]

    T = float('inf')
    time = 0

    while True:
        if time < T:
            if np.random.binomial(1, 0.5) == 1:
                next_state = state + 1
            else:
                next_state = state - 1
            if next_state not in STATE_TERMINAL:
                reward = 0
            else:
                reward = 1 if next_state == STATE_TERMINAL[1] else 0
                T = time + 1
            state = next_state
            states.append(next_state)
            rewards.append(reward)
            time += 1


        update_time = time - n  # + 1
        if update_time >= 0:
            returns = 0
            for i in range(update_time + 1, min(update_time + n, T) + 1):
                returns += pow(GAMMA, i - update_time - 1) * rewards[i]
            if update_time + n < T:
                returns = returns + pow(GAMMA, n) * value[states[update_time + n]]
            value[states[update_time]] += alpha * (returns - value[states[update_time]])
        if not time < T:
            time += 1
        if update_time == T - 1:
            break


if __name__ == '__main__':
    runs = 1000
    values = np.zeros(STATES_NUM + 2)
    for ep in tqdm(range(0, runs)):
        temporal_difference(values, 4, 0.4)

    plt.plot(values[1:-1])
    plt.show()