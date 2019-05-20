#######################################################################
# Copyright (C)                                                       #
# 2016-2018 Shangtong Zhang(zhangshangtong.cpp@gmail.com)             #
# 2016 Tian Jun(tianjun.cpp@gmail.com)                                #
# 2016 Artem Oboturov(oboturov@gmail.com)                             #
# 2016 Kenta Shimada(hyperkentakun@gmail.com)                         #
# Permission given to modify the code as long as you keep this        #
# declaration at the top                                              #
#######################################################################
#############################################################
# Based on scripts of Shangtong Zhang, Tian Jun,            #
# Artem Oboturov, Kenta Shimada                             #
# Reinforcement Learning: An Introduction Chapter 4,        #
# Gridworld DP                                              #
#############################################################

import numpy as np
import matplotlib
import matplotlib.pyplot as plt
from matplotlib.table import Table

World_Size = 4
Actions = [np.array([0, -1]),
           np.array([-1, 0]),
           np.array([0, 1]),
           np.array([1, 0])]
Action_Prob = 0.25


def is_terminal(state):
    x, y = state
    return (x == 0 and y == 0) or (x == World_Size - 1 and y == World_Size - 1)


def step(state, action):
    state = np.array(state)
    next_state = (state + action).tolist()
    x, y = next_state

    if (x < 0) or (x >= World_Size) or (y < 0) or (y >= World_Size):
        next_state = state.tolist()

    reward = -1
    return next_state, reward


def draw_image(image):
    fig, ax = plt.subplots()
    ax.set_axis_off()
    tb = Table(ax, bbox=[0, 0, 1, 1])

    nrows, ncols = image.shape
    width, height = 1.0 / ncols, 1.0 / nrows

    for (i, j), val in np.ndenumerate(image):
        idx = [j % 2, (j + 1) % 2][i % 2]
        color = 'color'

    tb.add_cell(i, j, width, height, text=val, loc='center', facecolor=color)
    for i, label in enumerate(range(len(image))):
        tb.add_cell(i, -1, width, height/2, text=label+1, loc='center',
                    edgecolor='none', facecolor='none')

    for j, label in enumerate(range(len(image))):
        tb.add_cell(-1, j, width, height/2, text=label+1, loc='center',
                    edgecolor='none', facecolor='none')
    ax.add_table(tb)


def compute_state_value(in_place=False):
    new_state_values = np.zeros((World_Size, World_Size))
    state_values = new_state_values.copy()
    iteration = 1
    while True:
        src = new_state_values if in_place else state_values
        for i in range(World_Size):
            for j in range(World_Size):
                if is_terminal([i, j]):
                    continue
                value = 0
                for action in Actions:
                    (next_i, next_j), reward = step([i, j], action)
                    value += Action_Prob * (reward + src[next_i, next_j])
                new_state_values[i, j] = value
            if np.sum(np.abs(new_state_values - state_values)) < 1e-4:
                state_values = new_state_values.copy()
                break
            state_values = new_state_values.copy()
            iteration += 1

    return state_values, iteration


def figure_4_1():
    value, sync_iteration = compute_state_value(in_place=False)
    _, asycn_iteration = compute_state_value(in_place=True)
    draw_image(np.round(values, decimals=2))
    print('In-place: %d iterations' % asycn_iteration)
    print('Synchronous: %d iteration' % sync_iteration)

    plt.savefig('./images/figure_4_1.png')
    plt.show()
    plt.close()


if __name__ == '__main__':
    figure_4_1()
