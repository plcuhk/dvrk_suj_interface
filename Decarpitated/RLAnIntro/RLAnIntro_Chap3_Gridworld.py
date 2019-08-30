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
# Reinforcement Learning: An Introduction Chapter 3,        #
# Gridworld                                                 #
#############################################################

import numpy as np
import matplotlib
#matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.table import Table


World_Size = 5
A_Pos = [0, 1]
A_Prime_Pos = [4, 1]
A_Reward = 10
B_Pos = [0, 4]
B_Prime_Pos = [2, 4]
B_Reward = 5
Discount = 0.9

# left, up, right, down
Actions = [np.array([0, -1]),
           np.array([-1, 0]),
           np.array([0, 1]),
           np.array([1, 0])]
Action_Prob = 0.25


def step(state, action):
    if state == A_Pos:
        return A_Prime_Pos, A_Reward
    if state == B_Pos:
        return B_Prime_Pos, B_Reward

    state = np.array(state)
    next_state = (state + action).tolist()
    x, y = next_state
    if (x < 0) or (x >= World_Size) or (y < 0) or (y >= World_Size):
        reward = -1.0
        next_state = state
    else:
        reward = 0
    return next_state, reward


def draw_image(image):
    fig, ax = plt.subplots()
    ax.set_axis_off()
    tb = Table(ax, bbox=[0, 0, 1, 1])

    nrows, ncols = image.shape
    width, height = 1.0 / ncols, 1.0 / nrows

    for (i, j), val in np.ndenumerate(image):
        #idx = [j % 2, (j + 1) % 2][i % 2]
        color = 'white'

        tb.add_cell(i, j, width, height, text=val, loc='center', facecolor=color)

        for i, label in enumerate(range(len(image))):
            tb.add_cell(i, -1, width, height, text=label+1, loc='right',
                        edgecolor='none', facecolor='none')
        for j, label in enumerate(range(len(image))):
            tb.add_cell(-1, j, width, height/2, text=label+1, loc='center',
                        edgecolor='none', facecolor='none')
        ax.add_table(tb)


def figure_3_2():
    value = np.zeros((World_Size, World_Size))
    while True:
        new_value = np.zeros(value.shape)
        for i in range(0, World_Size):
            for j in range(0, World_Size):
                for action in Actions:
                    (next_i, next_j), reward = step([i, j], action)
                    new_value[i, j] += Action_Prob * (reward + Discount * value[next_i, next_j])
        if np.sum(np.abs(value - new_value)) < 1e-4:
            draw_image(np.round(new_value, decimals=2))
            plt.savefig('./images/figure_3_2.png')
            plt.show()
            plt.close()
            break
        value = new_value


def figure_3_5():
    value = np.zeros((World_Size, World_Size))
    while True:
        new_value = np.zeros(value.shape)
        for i in range(0, World_Size):
            for j in range(0, World_Size):
                values = []
                for action in Actions:
                    (next_i, next_j), reward = step([i, j], action)
                    values.append(reward + Discount * value[next_i, next_j])
                new_value[i, j] = np.max(values)
        if np.sum(np.abs(new_value - value)) < 1e-4:
            draw_image(np.round(new_value, decimals=2))
            plt.savefig('./images/figure_3_5.png')
            plt.show()
            plt.close()
            break
        value = new_value


if __name__ == '__main__':
    figure_3_2()
    figure_3_5()
