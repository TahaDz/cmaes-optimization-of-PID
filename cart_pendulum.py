import numpy as np,  math , time , random
import scipy.integrate as integrate
from Cart import *
from Pendulum import *


def find_pid_control_input(error,times,K):
    # Using PID to find control inputs

    Kp = K[0] # proportional action
    Ki = K[1] # Integral action
    Kd = K[2] # derivative action

    derivative = np.diff(error) / np.diff(times) # derivative of error
    integral = integrate.simps(error, times) # integral of error

    Max = 220 # Maximum force applied on cart by PID controller
    Min = -Max # Minimum force applied on cart

    F = - ((Kp * error[-1]) + (Kd * derivative[-1]) + (Ki * integral)) # PID force applied on the cart
    if F > Max :
        return Max
    elif F < Min :
        return Min
    else:
        return F



def derivatives(state, times, F, cart, pendulum, g):
    # state space representation of cart pendulum ( simulation of the cart pendulum)
    # ds is the derivative of state space vector
    ds = np.zeros(len(state)) # initializing the state vector ds[0] = angle derivative, ds[2] = position derivative

    ds[0] = state[1]

    ds[1] = (((cart.mass + pendulum.ball_mass) * g * math.sin(state[0])) + (
            F * math.cos(state[0])) - (pendulum.ball_mass * ((state[1]) ** 2.0) * pendulum.length * math.sin(
        state[0]) * math.cos(state[0]))) / (pendulum.length * (cart.mass + (pendulum.ball_mass * (math.sin(state[0]) ** 2.0))))
    ds[2] = state[3]

    ds[3] = ((pendulum.ball_mass * g * math.sin(state[0]) * math.cos(state[0])) - (
            pendulum.ball_mass * pendulum.length * math.sin(state[0]) * (state[1] ** 2)) + (F)) / (
                           cart.mass + (pendulum.ball_mass * (math.sin(state[0]) ** 2)))

    return ds





def apply_control_input(cart, pendulum, times, state, F, g):
    # Finding x and theta on considering the control inputs and the dynamics of the system

    solution = integrate.odeint(derivatives, state, times[-2:], (F,cart,pendulum, g)) # compute the new vector state
    state = solution[-1, :]
    ths = solution[:, 0]
    xs = solution[:, 2]
    cart.x = xs[-1] # cart position
    pendulum.theta = ths[-1] #pendulum angle

    return state  # the new state vector





def find_error(pendulum):
    # This function returns the error of the pendulum angle

    error = (pendulum.theta % (2 * math.pi)) - 0
    if error > math.pi:
        error = error - (2 * math.pi)
    return error



def find_theta(pendulum) :
    # the function return the pendulum angle to an equivalent value in [-2pi, 2pi]

    theta = pendulum % (2 * math.pi)
    return theta


mass_of_ball = 0.1
mass_of_cart = 1.0
pendulum_length = 1.5
world_size = 600 # window size for simulation
g = 9.81 # gravity




def Initialize_pendulum(mass_ball,mass_cart,length) :
    # set cat pendulum parameters values
    global mass_of_ball, mass_of_cart,pendulum_length
    mass_of_ball = mass_ball
    mass_of_cart = mass_cart
    pendulum_length = length


def print_values():
    # print cat-pendulum's parameters

    print("******************")
    print("mass of pendulum : ",mass_of_ball," mass of the cart : ", mass_of_cart,"pendulum's length : ",pendulum_length)
    print("******************")



def Initialize (simulation_time,initial_theta,pid):
    # Simulation of inverted pendulum
    # initial_theta : angle of pendulum at the beginning of the simulation
    # vector of pid controller parameters (Kp, Ki, Kd)

    errors, force, theta, times, x = [], [], [], [], [] # initializing of variables
    previous_timestamp = time.time() # to calculate derivative of time
    end_time = previous_timestamp + simulation_time # end of simulation

    # initializing cart and pendulum
    cart = Cart(int(0.5 * world_size), mass_of_cart, world_size)
    pendulum = Pendulum(pendulum_length, initial_theta, mass_of_ball)

    start_time = previous_timestamp # to calculate derivative of time

    state = [pendulum.theta,  # initializing state vector of pendulum
             0,
             cart.x,
             0]

    times.append(previous_timestamp - start_time) # time vector
    errors.append(find_error(pendulum)) # error of theta
    x.append(cart.x) # cart position
    theta.append(pendulum.theta) # pendulum angle
    force.append(0) # PID force applied to the cart

    while time.time() <= end_time:
        current_timestamp = time.time()
        times.append(current_timestamp - start_time)
        errors.append(find_error(pendulum))
        theta.append(find_theta(pendulum.theta))
        F = find_pid_control_input(errors, times,pid) # compute the PID forc
        state = apply_control_input(cart, pendulum, times, state, F, g) # compute new state vector
    

    return errors, times
    
    


def simulate (simulation_time,initial_theta,pid):
    # animation of inverted pendulum

    from simulation import display_stuff

    errors, theta, times, x = [], [], [], []
    previous_timestamp = time.time()
    end_time = previous_timestamp + simulation_time
    cart = Cart(int(0.5 * world_size), mass_of_cart, world_size)
    pendulum = Pendulum(pendulum_length, initial_theta, mass_of_ball)
    start_time = previous_timestamp
    state = [pendulum.theta,
             0,
             cart.x,
             0]

    times.append(previous_timestamp - start_time)
    errors.append(find_error(pendulum))
    x.append(cart.x)
    theta.append(pendulum.theta)

    while time.time() <= end_time:
        current_timestamp = time.time()
        times.append(current_timestamp - start_time)
        errors.append(find_error(pendulum))
        theta.append(find_theta(pendulum.theta))
        F = find_pid_control_input(errors, times, K = pid)
        state = apply_control_input(cart, pendulum, times, state, F, g)
        display_stuff(world_size, cart, pendulum)






    
