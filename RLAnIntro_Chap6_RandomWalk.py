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

        # if current_state == 6:
        #    action_reward = 1
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
            break
        elif current_state == 0:
            returns = 0
            break

    if not batch:
        for state in trajectory[:-1]:
            values[state] += alpha * (returns - values[state])

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


def rms_error():
    td_alpha = [0.15, 0.1, 0.05]
    mc_alpha = [0.01, 0.02, 0.03, 0.04]
    episodes = 100 + 1
    runs = 100
    for i, alpha in enumerate(td_alpha + mc_alpha):
        total_errors = np.zeros(episodes)
        if i < len(td_alpha):
            method = 'TD'
            linestyle = 'solid'
        else:
            method = 'MC'
            linestyle = 'dashdot'
        for r in tqdm(range(runs)):
            errors = []
            current_values = np.copy(VALUES)
            for i in range(0, episodes):
                errors.append(np.sqrt(np.sum(np.power(TRUE_VALUES - current_values, 2)) / 5.0))
                if method == 'TD':
                    temporal_difference(current_values, alpha=alpha)
                else:
                    monte_carlo(current_values, alpha=alpha)
            total_errors += np.asarray(errors)
        total_errors /= runs
        plt.plot(total_errors, linestyle=linestyle, label=method + ', alpha = %.02f' % alpha)
    plt.xlabel('episodes')
    plt.ylabel('RMS')
    plt.legend()


def batch_updating(method, episodes, alpha=0.001):
    runs = 100
    total_errors = np.zeros(episodes)
    for r in tqdm(range(0, runs)):
        current_values = np.copy(VALUES)
        errors = []
        trajectories = []
        rewards = []
        for ep in range(episodes):
            if method == 'TD':
                trajectories_, rewards_ = temporal_difference(current_values, batch=True)
            else:
                trajectories_, rewards_ = monte_carlo(current_values, batch=True)
            trajectories.append(trajectories_)
            rewards.append(rewards_)
            while True:
                updates = np.zeros(7)
                for trajectory_, rewards_ in zip(trajectories, rewards):
                    for i in range(0, len(trajectory_) - 1):
                        if method == 'TD':
                            updates[trajectory_[i]] += rewards_[i] + current_values[trajectory_[i + 1]] \
                                                        - current_values[trajectory_[i]]
                        else:
                            updates[trajectory_[i]] += rewards_[i] - current_values[trajectory_[i]]
                updates *= alpha
                if np.sum(np.abs(updates)) < 1e-3:
                    break
                current_values += updates
            errors.append(np.sqrt(np.sum(np.power(current_values - TRUE_VALUES, 2)) / 5))
        total_errors += np.asarray(errors)
    total_errors /= runs
    return total_errors


def example_6_2():
    plt.figure(figsize=(20, 20))
    plt.subplot(2, 1, 1)
    compute_state_value()

    plt.subplot(2, 1, 2)
    rms_error()
    plt.tight_layout()
    plt.savefig('./images/figure_6_1.png')
    plt.close()


def figure_6_2():
    episodes = 100 + 1
    td_errors = batch_updating('TD', episodes)
    mc_errors = batch_updating('MC', episodes)

    plt.plot(td_errors, label='TD')
    plt.plot(mc_errors, label='MC')
    plt.xlabel('episodes')
    plt.ylabel('RMS error')
    plt.legend()

    plt.savefig('./images/figure_6_2.png')
    plt.close()


if __name__ == '__main__':
    example_6_2()
    figure_6_2()
