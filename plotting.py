import matplotlib.pyplot as plt
import numpy as np


def plotTorques(results: dict):
    """
    results: dict with
        titles of experiment as keys
        item is dict {"torques": log_torques, "ee_tracking_errors": ee_errors}
            where log_torques is joints x time 2d ndarray (for each dt)
            and ee_errors is 1d ndarray of end effector tracking error (for each dt)
    """
    n = len(results)
    fig = plt.figure("Torques comparison")
    for i, key in enumerate(results):
        idx = n * 100 + 10 + (i + 1)
        plt.subplot(idx)
        torques = results[key]["torques"]
        torques_squares_sum = np.round(np.sum(np.square(torques)), 2)
        tracking_err_sum = np.round(np.sum(np.square(results[key]["ee_tracking_errors"])), 4)
        plt.title(f"{key}: torques_squares_sum = {torques_squares_sum}, tracking_err_sum = {tracking_err_sum}")
        [plt.plot(torques[:, j], label="u" + str(j)) for j in range(torques.shape[1])]
        plt.legend()
        plt.xlabel("t, ms")

    plt.subplots_adjust(left=0.05,
                        bottom=0.05,
                        right=0.95,
                        top=0.95,
                        wspace=0.5,
                        hspace=0.5)
    plt.show()
