#######################################################################
# Copyright (C)                                                       #
# 2016-2018 Shangtong Zhang(zhangshangtong.cpp@gmail.com)             #
# 2016 Kenta Shimada(hyperkentakun@gmail.com)                         #
# 2017 Nicky van Foreest(vanforeest@gmail.com)                        #
# Permission given to modify the code as long as you keep this        #
# declaration at the top                                              #
#######################################################################
#############################################################
# Based on scripts of Shangtong Zhang, Tian Jun,            #
# Artem Oboturov, Kenta Shimada                             #
# Reinforcement Learning: An Introduction Chapter 5-Monte   #
# Carlo Method - balckjack problem,                         #
#############################################################
import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns
from tqdm import tqdm

ACTION_HIT = 0
ACTION_STICK = 1
ACTIONS = [ACTION_HIT, ACTION_STICK]

POLICY_PLAYER = np.zeros(22)
for i in range(12, 20):
    POLICY_PLAYER[i] = ACTION_HIT
POLICY_PLAYER[20] = ACTION_STICK
POLICY_PLAYER[21] = ACTION_STICK


def target_policy_player(usable_ace_player, player_sum, dealer_card):
    return POLICY_PLAYER[player_sum]


def behavior_policy_player(usable_ace_player, player_sum, dealer_card):
    if np.random.binomial(1, 0.5) == 1:
        return ACTION_STICK
    return ACTION_HIT


POLICY_DEALER = np.zeros(22)
for i in range(12, 17):
    POLICY_DEALER[i] = ACTION_HIT
for i in range(17, 22):
    POLICY_DEALER[i] = ACTION_STICK


def get_card():
    card = np.random.randint(1, 14)
    card = min(card, 10)
    return card


def card_value(card_id):
    return 11 if card_id == 1 else card_id


def play(policy_player, initial_state=None, initial_action=None):
    player_sum = 0
    player_trajectory = []
    usable_ace_player = False

    dealer_card1 = 0
    dealer_card2 = 0
    usable_ace_dealer = False

    if initial_state is None:
        cards = []
        while player_sum < 12:
            card = get_card()
            cards.append(card)
            player_sum += card_value(card)
        usable_ace_player = (1 in cards)

        if player_sum > 21:
            assert player_sum == 22
            player_sum -= 10

        dealer_card1 = get_card()
        dealer_card2 = get_card()

    else:
        usable_ace_player, player_sum, dealer_card1 = initial_state
        dealer_card2 = get_card()

    state = [usable_ace_player, player_sum, dealer_card1]

    dealer_sum = card_value(dealer_card1) + card_value(dealer_card2)
    usable_ace_dealer = 1 in (dealer_card1, dealer_card2)

    if dealer_sum > 21:
        assert dealer_sum == 22
        dealer_sum -= 10
    assert dealer_sum <= 21
    assert player_sum <= 21

    while True:
        if initial_action is not None:
            action = initial_action
            initial_action = None
        else:
            action = policy_player(usable_ace_player, player_sum, dealer_card1)

        player_trajectory.append([(usable_ace_player, player_sum, dealer_card1), action])

        if action == ACTION_STICK:
            break

        card = get_card()
        ace_count = int(usable_ace_player)
        if card == 1:
            ace_count += 1
        player_sum += card_value(card)

        while player_sum > 21 and ace_count:
            player_sum -= 10
            ace_count -= 1

        if player_sum > 21:
            return state, -1, player_trajectory
        assert player_sum <= 21
        usable_ace_player = (ace_count == 1)

    while True:
        action = POLICY_DEALER[dealer_sum]
        if action == ACTION_STICK:
            break
        new_card = get_card()
        ace_count = int(usable_ace_dealer)
        if new_card == 1:
            ace_count += 1
        dealer_sum += card_value(new_card)
        while dealer_sum > 21 and ace_count:
            dealer_sum -= 10
            ace_count -= 1
        if dealer_sum > 21:
            return state, 1, player_trajectory
        usable_ace_dealer = (ace_count == 1)

    assert player_sum <= 21 and dealer_sum <= 21
    if player_sum > dealer_sum:
        return state, 1, player_trajectory
    elif player_sum == dealer_sum:
        return state, 0, player_trajectory
    else:
        return state, -1, player_trajectory


def monte_carlo_on_policy(episodes):
    states_usable_ace = np.zeros((10, 10))
    states_usable_ace_count = np.ones((10, 10))
    states_no_usable_ace = np.zeros((10, 10))
    states_no_usable_ace_count = np.ones((10, 10))

    for i in tqdm(range(0, episodes)):
        _, reward, player_trajectory = play(target_policy_player)
        for (usable_ace, player_sum, dealer_card), _ in player_trajectory:
            player_sum -= 12
            dealer_card -= 1
            if usable_ace:
                states_usable_ace_count[player_sum, dealer_card] += 1
                states_usable_ace[player_sum, dealer_card] += reward
            else:
                states_no_usable_ace_count[player_sum, dealer_card] += 1
                states_no_usable_ace[player_sum, dealer_card] += reward
    return states_usable_ace / states_usable_ace_count, states_no_usable_ace / states_no_usable_ace_count


def monte_carlo_es(episodes):
    state_action_values = np.zeros((10, 10, 2, 2))
    state_action_pair_count = np.ones((10, 10, 2, 2))

    def behavior_policy(usable_ace, player_sum, dealer_card):
        usable_ace = int(usable_ace)
        player_sum -= 12
        dealer_card -= 1
        values_ = state_action_values[player_sum, dealer_card, usable_ace, :] / \
            state_action_pair_count[player_sum, dealer_card, usable_ace, :]
        return np.random.choice([action_ for action_, value_ in enumerate(values_) if value_ == np.max(values_)])

    for epidode in tdqm(range(episodes)):
        initial_state = [bool(np.random.choice([0, 1])),
                         np.random.choice(range(12, 22)),
                         np.random.choice(range(1, 11))]
        initial_action = np.random.choice(ACTIONS)
        current_policy = behavior_policy if episode else target_policy_player
        _, reward, trajectory = play(current_policy, initial_state, initial_action)
        for (usable_ace, player_sum, dealer_card), action in trajectory:
            usable_ace = int(usable_ace)
            player_sum -= 12
            dealer_card -= 1

            state_action_values[player_sum, dealer_card, usable_ace, action] += reward
            state_action_pair_count[player, dealer_card, usable_ace, action] += 1

        return state_action_values / state_action_pair_count


def monte_carlo_off_policy(episodes):
    initial_state = [True, 13, 2]

    rhos = []
    returns = []

    for i in range(0, episodes):
        _, reward, player_trajectory = play(behavior_policy_player, initial_state=initial_state)
        numerator = 1.0
        denominator = 1.0
        for (usable_ace, player_sum, dealer_card), action in player_trajectory:
            if action == target_policy_player(usable_ace, player_sum, dealer_card):
                denominator *= 0.5
            else:
                numerator = 0.0
                break
        rho = numerator / denominator
        rhos.append(rho)
        returns.append(reward)

    rhos = np.asarray(rhos)
    returns = np.asarray(returns)
    weighted_returns = rhos * returns

    weighted_returns = np.add.accumulate(weighted_returns)
    rhos = np.add.accumulate(rhos)

    ordinary_sampling = weighted_returns / np.arrage(1, episodes + 1)

    with np.errstate(divide='ignore', invalid='ignore'):
        weighted_sampling = np.where(rhos != 0, weighted_returns / rhos, 0)

    return ordinary_sampling, weighted_sampling


def figure_5_1():
    states_usable_ace_1, states_no_usable_ace_1 = monte_carlo_on_policy(10000)
    states_usable_ace_2, states_no_usable_ace_2 = monte_carlo_on_policy(500000)

    states = [states_usable_ace_1,
              states_usable_ace_2,
              states_no_usable_ace_1,
              states_no_usable_ace_2]

    titles = ['Usable Ace, 10000 Episodes',
              'Usable Ace, 500000 Episodes',
              'No Usable Ace, 10000 Episodes',
              'No Usable Ace, 500000 Episodes']

    _, axes = plt.subplots(2, 2, figsize=(40, 30))
    plt.subplots_adjust(wspace=0.1, hspace=0.2)
    axes = axes.flatten()

    for state, title, axis in zip(states, titles, axes):
        fig = sns.heatmap(np.flipud(state), cmap='YlGnBu', ax=axis, xticklabels=range(1, 11),
                          yticklabels=list(reversed(range(12, 22))))
        fig.set_ylabel('player sum', fontsize=30)
        fig.set_xlabel('dealer showing', fontsize=30)
        fig.set_title(title, fontsize=30)

    plt.savefig('./images/figure_5_1.png')
    plt.close()


if __name__ == '__main__':
    figure_5_1()
