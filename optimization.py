
import cma
from cart_pendulum import *
# optimization of pid controller parameters controlling the inverted pendulum using CMA-ES


def cma_optimize(K):
    # the function simulate and return fitness value of solution

    simulation_time = 5
    errors, times = Initialize(simulation_time = simulation_time, initial_theta = random.uniform(-1.25,1.25),pid= K) # simulate the pendulum
    fitness = integrate.simps(np.square(errors), times) # compute the fitness value of K (kp, ki, kd) combination of pid parameters

    return fitness

def main(K, population_size, nbre_iteration) :
    # K(Kp, Ki, Kd) is a vector of parameters of pid controller
    # run the optimisation
    # cma.fmin2(objective function, initial solutions, Initial sigma, options={'maxiter': nbre_iteration, 'bounds': [[0, 0, 0], [50, 50, 50]],'popsize':population_size})
    best_solution, best_fitness = cma.fmin2(cma_optimize, [K[0], K[1],K[2]], 15, options={'maxiter': nbre_iteration, 'bounds': [[0, 0, 0], [50, 50, 50]],'popsize':population_size})
    print("best solution :", best_solution) # print the solutions of the optimization



if __name__ == "__main__":
    main([40,20,20],2,2)

