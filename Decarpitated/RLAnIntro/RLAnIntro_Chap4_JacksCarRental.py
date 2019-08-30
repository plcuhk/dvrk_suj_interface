import numpy as np
import matplotlib
import matplotlib.pyplot as plt
from math import exp, factorial
import seaborn as sns

Max_Cars = 20
Max_Cars_Move = 5
Rental_Request_First = 3
Rental_Request_Second = 4
Returns_First = 3
Returns_Second = 2
Discount = 0.9
Rental_Credit = 10
Move_Car_Cost = 2
Poisson_Upper_Bound = 11

Actions = np.arange(-Max_Cars_Move, Max_Cars_Move + 1)

poisson_cache = dict()


def poisson(n, lam):
    global poisson_cache
    key = n * 10 + lam
    if key not in poisson_cache.keys():
        poisson_cache[key] = exp(-lam) * pow(lam, n) / factorial(n)
    return poisson_cache[key]


def expected_return(state, action, state_value, constant_returned_cars):
    returns = 0.0
    returns -= Move_Car_Cost * abs(action)

    for rental_request_first_loc in range(0, Poisson_Upper_Bound):
        for rental_request_second_loc in range(0, Poisson_Upper_Bound):
            num_of_cars_first_loc = int(min(state[0] - action, Max_Cars))
            num_of_cars_second_loc = int(min(state[1] + action, Max_Cars))

            real_rental_first_loc = min(num_of_cars_first_loc, rental_request_first_loc)
            real_rental_second_loc = min(num_of_cars_second_loc, rental_request_second_loc)

            reward = (real_rental_first_loc + real_rental_second_loc) * Rental_Credit
            num_of_cars_first_loc -= real_rental_first_loc
            num_of_cars_second_loc -= real_rental_second_loc

            prob = poisson(rental_request_first_loc, Rental_Request_First) * \
                poisson(rental_request_second_loc, Rental_Request_Second)
            if constant_returned_cars:
                returned_cars_first_loc = Returns_First
                returned_cars_second_loc = Returns_Second
                num_of_cars_first_loc = min(num_of_cars_first_loc + returned_cars_first_loc, Max_Cars)
                num_of_cars_second_loc = min(num_of_cars_second_loc + returned_cars_second_loc, Max_Cars)
                returns += prob * (reward + Discount * state_value[num_of_cars_first_loc, num_of_cars_second_loc])
            else:
                for returned_cars_first_loc in range(0, Poisson_Upper_Bound):
                    for returned_cars_second_loc in range(0, Poisson_Upper_Bound):
                        num_of_cars_first_loc = min(num_of_cars_first_loc + returned_cars_first_loc, Max_Cars)
                        num_of_cars_second_loc = min(num_of_cars_second_loc + returned_cars_second_loc, Max_Cars)
                        prob = poisson(returned_cars_first_loc, Returns_First) * \
                            poisson(returned_cars_second_loc, Returns_Second) * prob
                        returns += prob * (reward + Discount * state_value[num_of_cars_first_loc, num_of_cars_second_loc])
    return returns


def figure_4_2(constant_returned_cars=True):
    value = np.zeros((Max_Cars + 1, Max_Cars + 1))
    policy = np.zeros(value.shape, dtype=np.int)

    iterations = 0
    _, axes = plt.subplots(2, 3, figsize=(40, 20))
    plt.subplots_adjust(wspace=0.1, hspace=0.2)
    axes = axes.flatten()
    while True:
        fig = sns.heatmap(np.flipud(policy), cmap="YlGnBu", ax=axes[iterations])
        fig.set_ylabel('# cars at first location', fontsize=30)
        fig.set_yticks(list(reversed(range(Max_Cars + 1))))
        fig.set_xlabel('# cars at second location', fontsize=30)
        fig.set_title('policy %d' % iterations, fontsize=30)

        while True:
            new_value = np.copy(value)
            for i in range(Max_Cars + 1):
                for j in range(Max_Cars + 1):
                    new_value[i, j] = expected_return([i, j], policy[i, j], new_value, constant_returned_cars)

            value_change = np.abs((new_value - value)).sum()
            print('value change %f' % value_change)
            value = new_value
            if value_change < 1e-4:
                break

        new_policy = np.copy(policy)
        for i in range(Max_Cars):
            for j in range(Max_Cars):
                action_returns = []
                for action in Actions:
                    if (i >= action >= 0) or (action < 0 and j >= abs(action)):
                        action_returns.append(expected_return([i, j], action, value, constant_returned_cars))
                    else:
                        action_returns.append(-float('inf'))
                new_policy[i, j] = Actions[np.argmax(action_returns)]

        policy_change = (new_policy != policy).sum()
        print('policy changed in %d states' % policy_change)
        policy = new_policy
        if policy_change == 0:
            fig = sns.heatmap(np.flipud(value), cmap='YlGnBu', ax=axes[-1])
            fig.set_ylabel('# cars at first location', fontsize=30)
            fig.set_yticks(list(reversed(range(Max_Cars + 1))))
            fig.set_xlabel('# cars at second location', fontsize=30)
            fig.set_title('optimal value', fontsize=30)
            break

        iterations += 1
    plt.savefig('./images/figure_4_2.png')
    plt.close()


if __name__ == '__main__':
    figure_4_2()
