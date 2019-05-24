#######################################################################
# Copyright (C)                                                       #
# 2016-2018 Shangtong Zhang(zhangshangtong.cpp@gmail.com)             #
# 2016 Kenta Shimada(hyperkentakun@gmail.com)                         #
# Permission given to modify the code as long as you keep this        #
# declaration at the top                                              #
#######################################################################
#############################################################
# Based on scripts of Shangtong Zhang, Tian Jun,            #
# Artem Oboturov, Kenta Shimada                             #
# Reinforcement Learning: An Introduction Chapter 4-Dynamic #
# Programming - Gambler's Problem, Value Iteration          #
# Comments added                                            #
#############################################################
import numpy as np
import matplotlib.pyplot as plt

Goal = 100
# All states including 0 and 100. This is needed and different from the problem description
# where states = {1, 2, ... , 99} *** because in value iteration, the state could become 0
# or 100 and corresponding rewards need to be added. ***
States = np.arange(Goal + 1)
Head_Prob = 0.4


def figure_4_3():
    state_value = np.zeros(Goal + 1)
    # state_value[Goal] is set to be 1 and all other states are set
    # to be zero. Remember the reward is for what the agent wants to
    # achieve, not how to achieve it.
    state_value[Goal] = 1.0

    # value iteration
    while True:
        delta = 0.0
        for state in States[1:-1]:  # from state{1} to state{99}
            actions = np.arange(min(state, Goal - state) + 1)
            action_returns = []
            for action in actions:
                action_returns.append(Head_Prob * state_value[state + action]
                                      + (1 - Head_Prob) * state_value[state - action])
            new_value = np.max(action_returns)
            delta += np.abs(new_value - state_value[state])
            state_value[state] = new_value
        if delta < 1e-9:
            break

    # Get optimal policy
    policy = np.zeros(Goal + 1)
    for state in States[1:-1]:
        actions = np.arange(min(state, Goal - state) + 1)
        action_returns = []
        for action in actions:
            action_returns.append(Head_Prob * state_value[state + action]
                                  + (1 - Head_Prob) * state_value[state - action])
        # This line makes the output resemble Sutton's book, which actually ignore the
        # action of {0}. But the real optimal policy allows {0} follow. i.e.,
        # policy[state] = actions[np.argmax(action_returns)]
        policy[state] = actions[np.argmax(np.round(action_returns[1:], 5)) + 1]

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

