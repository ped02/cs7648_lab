import matplotlib.pyplot as plt
import os  # noqa: F401
import copy
import numpy as np

from nn_numpy import create_neural_net, forward_pass, backprop  # noqa: F401
from utils import loadObject, saveNetworks, add_dim_last  # noqa: F401
from wam_environment.wam_env import WAMEnv
from wam_environment.utils import load_and_preprocess


def update_policy_with_bc(nn_readonly, inputs, labels, learning_rate):
    nn_new = copy.deepcopy(nn_readonly)
    loss = 0
    ##### INSERT CODE HERE
    # Training loop goes here
    # 1) iterate over samples and labels in your dataset (states and demonstration actions)
    # 1.1) Optionally, filter out undesired states
    # 2) Obtain predicted action from your network, calculate loss w.r.t expert action, get gradients for params
    # 2.1) Calculate gradient for a single sample, NOT a batch of inputs
    # 3) Update your network's params according to their gradients
    # 4) Update loss metric that you wish to plot
    # NOTE: Use nn_new for all computations, nn_readonly should not be used

    num_samples = inputs.shape[0]

    for i in range(num_samples):
        x_sample = inputs[i].reshape(-1, 1)
        y_sample = labels[i].reshape(-1, 1)

        y_pred = forward_pass(nn_new, x_sample, final_softmax=False, full_return=False)

        loss += np.mean((y_pred - y_sample) ** 2)

        grad = backprop(nn_new, x_sample, y_sample, loss='MSE')

        for l in range(len(nn_new)):
            nn_new[l][0] -= learning_rate * grad[l][0]
            nn_new[l][1] -= learning_rate * grad[l][1]

    loss = loss / num_samples

    #####
    return nn_new, loss


def plot_learned_result(
    nn, observations, actions, show=False, print_example_output=False
):
    actions_pred = np.transpose(
        forward_pass(nn, np.transpose(observations), False, False)
    )

    # if print_example_output:
    #     print("predicted", output_mean[50:60])
    #     print("gt_action", actions[50:60])
    #     print("log_std", output_log_std[50:51])
    #     print("act_diff", output_mean - actions)
    for i in range(6):
        plt.plot(actions_pred[:, i], label='predicted')
        plt.plot(actions[:, i], label='truth')
        # plt.plot(output_mean[:64, i] - sampled_gt_actions[:, i], label="difference")
        plt.legend()
        plt.title('j%d' % (i + 1))
        plt.savefig('figs/j%d.png' % (i + 1))
        if show:
            plt.show()
        plt.clf()
    print('Saved figures for each joint action in figs/')


if __name__ == '__main__':
    # Demo specification
    participant_id = 'team_1'
    repeat = 1

    # Initialize Env
    env = WAMEnv()

    # Load demo
    save_path = './demo/TA_demo_1.pkl'
    expert = load_and_preprocess(
        save_path
    )  # see load_and_preprocess documentation for an understanding of the state and action spaces
    print(expert['observations'].shape)

    observations = expert['observations'][: env.horizon, :]
    actions = expert['actions'][: env.horizon, :7]
    print(observations[0:5], 'observations')
    print(actions[0:5], 'actions')

    ##### MODIFY CODE HERE (Optional)
    # init network
    hidden_dims = [64, 64]
    numInputDims = 19
    numOutputDims = 7
    nn = create_neural_net(
        numNodesPerLayer=hidden_dims,
        numInputDims=numInputDims,
        numOutputDims=numOutputDims,
    )
    # training
    num_epochs = 100
    learning_rate = 1e-3
    #####

    acc = []
    for epoch in range(num_epochs):
        nn, accuracy = update_policy_with_bc(
            nn, observations, actions, learning_rate=learning_rate
        )
        print(f'Percent correct_preds: {accuracy}')
        acc.append(accuracy)

    plt.figure()
    plt.plot(acc, 'b-')
    plt.savefig('figs/progress_bc.png')
    print('Saved figs/progress_bc.png')

    saveNetworks(nn, None, 'trained_bc.pkl')
    print('Saved trained_bc.pkl')

    # Save debug figures
    plot_learned_result(nn, observations, actions)
