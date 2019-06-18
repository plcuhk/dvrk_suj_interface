import numpy as np
import matplotlib.pyplot as plt
from tqdm import tqdm

WORLD_HEIGHT = 4
WORLD_WIDTH = 11
START = [0, 3]
END = [10, 3]
REWARD = -1

ACTION_UP = 0
ACTION_DOWN = 1
ACTION_LEFT = 2
ACTION_RIGHT = 3
ACTIONS = [ACTION_UP, ACTION_DOWN, ACTION_LEFT, ACTION_RIGHT]
EPSILON = 0.1
ALPHA = 0.5
GAMMA = 1

CLIFF = np.ones((1, 9, 2))
CLIFF[:, :, 0] = np.arange(1, 10)
CLIFF[:, :, 1] = np.ones((1, 9)) * 3


def step(state, action):
    u, v = state

    if action == ACTION_UP:
        return [u, max(v - 1, 0)]
    elif action == ACTION_DOWN:
        return [u, min(v + 1, WORLD_HEIGHT - 1)]
    elif action == ACTION_LEFT:
        return [max(u - 1, 0), v]
    else:
        return [min(u + 1, WORLD_WIDTH - 1), v]


def cliff_check(state):
    u, v = state
    return (u in CLIFF[:, :, 0]) and (v in CLIFF[:, :, 1])


if __name__ == '__main__':
    q_value = np.zeros((WORLD_WIDTH, WORLD_HEIGHT, len(ACTIONS)))
    episodes_limit = 500
    rewards_sum = []
    episode = 1

    while episode <= episodes_limit:

        state = START
        rewards_sum_ = 0

        while state != END:
            u, v = state
            if np.random.binomial(1, EPSILON) == 1:
                action = np.random.choice(ACTIONS)
            else:
                q_value_ = q_value[u, v, :]
                action = np.random.choice([action_ for action_, value_ in enumerate(q_value_) if value_ == max(q_value_)])

            state = step(state, action)
            state = START if cliff_cleck(state) else state
            q_value[u, v, action] += ALPHA * (REWARD + GAMMA * max(q_value[state[0], state[1], :]) - q_value[u, v, action])
            rewards_sum_ += REWARD

        print(rewards_sum_)
        rewards_sum.append(rewards_sum_)
        episode += 1

plt.plot(range(0, episodes_limit), rewards_sum)
plt.xlabel('Episodes')
plt.ylabel('Sum of rewards in an episode')
plt.savefig('./images/figure_6_4.png')
plt.close()
