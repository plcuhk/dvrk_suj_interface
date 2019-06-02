import numpy as np
import matplotlib.pyplot as plt
from tqdm import tqdm

VALUES = np.zeros(7)
VALUES[1:6] = 0.5
VALUES[6] = 1

TRUE_VALUES = np.arange(0, 7) / 6
ACTION_LEFT = 0
ACTION_RIGHT = 1


def temporal_difference(values, alpha=0.1, gamma=1, batch=False):
    current_state = 3
    trajectory = [current_state]
    rewards = [0]
    action_reward = 0
    while True:
        old_state = current_state
        if np.random.binomial(1, 0.5) == ACTION_LEFT:
            current_state -= 1
        else:
            current_state += 1

        if current_state == 6:
            action_reward = 1
        trajectory.append(current_state)
        rewards.append(action_reward)
        if not batch:
            values[old_state] += alpha * (action_reward + gamma * values[current_state]
                                          - values[old_state])
        if current_state == 6 or current_state == 0:
            break

        return trajectory, rewards


def monte_carlo(values, alpha=0.1, batch=False):
    current_state = 3
    trajectory = [current_state]

    while True:
        if np.random.binomial(1, 0.5) == ACTION_LEFT:
            current_state -= 1
        else:
            current_state += 1
        trajectory.append(current_state)
        if current_state == 6:
            returns = 1
        elif current_state == 0:
            returns = 0
            break

        if not batch:
            for state in trajectory[:-1]:
                values[state] += alpha * (returns - values[states])

        return trajectory, [returns] * (len(trajectory) - 1)


def compute_state_value():
    episodes = [0, 1, 10, 100]
    current_values = np.copy(VALUES)
    plt.figure(1)
    for i in range(episodes[-1] + 1):
        if i in episodes:
            plt.plot(current_values, label=str(i) + ' episodes')
        temporal_difference(current_values)
    plt.plot(TRUE_VALUES, label='true values')
    plt.xlabel('state')
    plt.ylabel('estimated value')
    plt.legend()


if __name__ == '__main__':
    plt.figure(figsize=(20, 20))
    compute_state_value()

    plt.savefig('./images/figure_6_1.png')
    plt.close()

