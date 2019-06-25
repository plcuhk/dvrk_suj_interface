import numpy as np
import matplotlib.pyplot as plt

class MaxBiasRL():
    def __init__(self, EPSILON=0.1, ALPHA = 0.1, GAMMA=1, double_Q_learning=True):
        self.EPSILON = EPSILON
        self.ALPHA = ALPHA
        self.GAMMA = GAMMA
        self.state = np.arange(3)
        self.q_value1 = np.zeros((3, 2))
        self.q_value2 = np.zeros((3, 2))
        self.start = 1
        self.current_state = 1
        self.terminal_state = [0, 2]
        self.episode = 0
        self.double_Q_learning = double_Q_learning
        self.left_counter = 0
        self.percentage = []

    def take_action(self):
        if np.random.binomial(1, self.EPSILON) == 1:
            action = np.random.choice(np.arange(2))
        elif self.double_Q_learning:
            q_value_state1 = self.q_value1[self.current_state, :]
            q_value_state2 = self.q_value2[self.current_state, :]
            q_value_state = q_value_state1 if max(q_value_state1) > max(q_value_state2) else q_value_state2
            action = np.random.choice([action_ for action_, value_ in enumerate(q_value_state)
                                       if value_ == max(q_value_state)])
        else:
            q_value_state = self.q_value1[self.current_state, :]
            action = np.random.choice([action_ for action_, value_ in enumerate(q_value_state)
                                       if value_ == max(q_value_state)])
        return action

    def steps(self):
        action = self.take_action()
        if self.current_state == 1 and action == 0:
           next_state = 0
           reward = 0
           self.left_counter += 1
        elif self.current_state == 1 and action == 1:
            next_state = 2
            reward = 0
        elif self.current_state == 0:
            next_state = self.start
            reward = np.random.normal(-0.1, 1)
            self.episode += 1
            self.percentage.append(self.left_counter / self.episode)
        else:
            next_state = self.start
            reward = 0
            self.episode += 1
            self.percentage.append(self.left_counter / self.episode)

        if self.double_Q_learning:
            if np.random.binomial(1, 0.5) == 1:
                action_next = np.random.choice([action_ for action_, q_value_ in enumerate(self.q_value1[next_state, :])
                                                if q_value_ == max(self.q_value1[next_state, :])])
                self.q_value1[self.current_state, action] += self.ALPHA * (reward + self.GAMMA *
                                    self.q_value2[next_state, action_next] - self.q_value1[self.current_state, action])
            else:
                action_next = np.random.choice([action_ for action_, q_value_ in enumerate(self.q_value2[next_state, :])
                                                if q_value_ == max(self.q_value2[next_state, :])])
                self.q_value2[self.current_state, action] += self.ALPHA * (reward + self.GAMMA *
                                    self.q_value1[next_state, action_next] - self.q_value2[self.current_state, action])

        self.current_state = next_state


if __name__ == '__main__':
    episodes = 500
    RL = MaxBiasRL()

    while RL.episode <= episodes:
        RL.steps()
   # print(RL.percentage)

    plt.plot(RL.percentage)
    plt.show()