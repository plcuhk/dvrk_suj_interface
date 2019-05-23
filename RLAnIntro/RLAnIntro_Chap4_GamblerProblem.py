import numpy as np
import matplotlib.pyplot as plt

Goal = 100
States = np.arange(1, Goal + 1)
Head_Prob = 0.4


def figure_4_3():
    state_value = np.zeros(Goal)
    state_value[Goal - 1] = 1.0

    while True:
        delta = 0.0
        for state in States[0:-1]:
            actions = np.arange(min(state, Goal - state) + 1)
            action_returns = []
            for action in actions:
                action_returns.append(Head_Prob * state_value[state + action - 1]
                                      + (1 - Head_Prob) * state_value[state - action - 1])
            new_value = np.max(action_returns)
            delta += np.abs(new_value - state_value[state-1])
            state_value[state-1] = new_value
        if delta < 1e-9:
            break

    policy = np.zeros(Goal)
    for state in States[0:-1]:
        actions = np.arange(min(state, Goal - state) + 1)
        action_returns = []
        for action in actions:
            action_returns.append(Head_Prob * state_value[state + action - 1]
                                  + (1 - Head_Prob) * state_value[state - action - 1])
        policy[state - 1] = actions[np.argmax(action_returns)]

    plt.figure(figsize=(10, 20))
    plt.subplot(2, 1, 1)
    plt.plot(state_value)
    plt.xlabel('Capital')
    plt.ylabel('Value estimates')

    plt.subplot(2, 1, 2)
    plt.scatter(States, policy)
    plt.xlabel('Capital')
    plt.ylabel('Final policy (stake)')

    plt.savefig('./images/figure_4_3.png')
    plt.close()


if __name__ == '__main__':
    figure_4_3()

