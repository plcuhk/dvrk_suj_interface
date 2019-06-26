import numpy as np
import matplotlib.pyplot as plt
from tqdm import tqdm
import copy

STATE_A = 0
STATE_B = 1
STATE_TERMINAL = 2
STATE_START = STATE_A

ACTION_A_RIGHT = 0
ACTION_A_LEFT = 1

EPSILON = 0.1
ALPHA = 0.1
GAMMA = 1

# take 10 actions in B
ACTIONS_B = range(0, 10)
STATE_ACTIONS = [[ACTION_A_RIGHT, ACTION_A_LEFT], ACTIONS_B]
INITIAL_Q = [np.zeros(2), np.zeros(len(ACTIONS_B)), np.zeros(1)]

TRANSITION = [[STATE_TERMINAL, STATE_B], [STATE_TERMINAL] * len(ACTIONS_B)]

def choose_action(state, q_value):
    if np.random.binomial(1, EPSILON) == 1:
        action = np.random.choice(STATE_ACTIONS[state])
    else:
        values = q_value[state]
        action = np.random.choice([action_ for action_, value_ in enumerate(values) if value_ == np.max(values)])
    return action

def take_action(state, action):
    if state == STATE_A:
        return 0
    return np.random.normal(-0.1, 1)

def q_learning(q1, q2=None):
    state = STATE_START

    left_count = 0
    while state != STATE_TERMINAL:
        if q2 is None:
            action = choose_action(state, q1)
        else:
            action = choose_action(state, [item1 + item2 for item1, item2 in zip(q1, q2)])

        if state == STATE_START and action == ACTION_A_LEFT:
            left_count += 1
        reward = take_action(state, action)
        next_state = TRANSITION[state][action]
        if q2 is None:
            active_q = q1
            target = np.max(active_q[next_state])
        else:
            if np.random.binomial(1, 0.5) == 1:
                active_q = q1
                target_q = q2
            else:
                active_q = q2
                target_q = q1
            best_action = np.random.choice([action_ for action_, value_ in enumerate(target_q[next_state])
                                            if value_ == np.max(target_q[next_state])])
            target = target_q[next_state][best_action]

        active_q[state][action] += ALPHA * (reward + GAMMA * target - active_q[state][action])
        state = next_state
    return left_count

def figure_6_7():
    episodes = 300
    runs = 500
    left_counts_q = np.zeros((runs, episodes))
    left_counts_double_q = np.zeros((runs, episodes))

    for run in tqdm(range(runs)):
        q = copy.deepcopy(INITIAL_Q)
        q1 = copy.deepcopy(INITIAL_Q)
        q2 = copy.deepcopy(INITIAL_Q)
        for ep in range(0, episodes):
            left_counts_double_q[run, ep] = q_learning(q)
            left_counts_double_q[run, ep] = q_learning(q1, q2)

    left_counts_q = left_counts_q.mean(axis=0)
    left_counts_double_q = left_counts_double_q.mean(axis=0)

    plt.plot(left_counts_q, label='Q-learning')
    plt.plot(left_counts_double_q, label='Double Q-learning')
    plt.plot(np.ones(episodes) * 0.05, label='Optimal')
    plt.xlabel('Episodes')
    plt.ylabel('% left actions from A')
    plt.legend()

    plt.savefig('./images/figure_6_7.png')
    plt.close()

if __name__ == '__main__':
    figure_6_7()