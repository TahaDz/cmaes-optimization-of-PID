# simulate the cart pendulum system and plot the error of the pendulum's angle

import matplotlib
matplotlib.use('tkagg')
import matplotlib.pyplot as plt
from cart_pendulum import *


def plot_simulation(simulation_time, initial_theta, pid):

    errors, times = Initialize(simulation_time=simulation_time, initial_theta=initial_theta,pid = pid)  # simulate the pendulum

    ##################################################
    # calculating the time of the transient state (rise time or time to stable state)

    rise = 0
    static_error = 0.02  # static error allowed in radian in stable state
    for i in range(0, len(errors)):

        test_rise = False
        if errors[i] > - static_error and errors[i] < static_error:  # error in the interval allowed in stable state
            for j in range(i + 1, len(errors)):
                if errors[j] > static_error or errors[j] < - static_error:
                    break

                elif j == len(errors) - 1:
                    rise = i
                    test_rise = True

        if test_rise:
            break

    print("rise time = ",times[rise]) # print the rise time

    ###############################################
    # plot the simulation

    plt.grid()
    plt.xticks(np.arange(0, simulation_time + 1, 0.25))
    plt.xlim(0, simulation_time)
    plt.ylabel('Erreur angulaire (rad)', {'fontsize': 'large'})
    plt.xlabel('Temps (s)', {'fontsize': 'large'})
    plt.plot(times, errors, linewidth=1, label='Angle = ' + str(f"{round(initial_theta * 180 / math.pi):02d}") + '°')
    plt.legend(fontsize='large')
    plt.show()

if __name__ == "__main__":
    plot_simulation(5,1,[40,1,10])
