import cv2, time
from cart_pendulum import *


def display_stuff(world_size, cart, pendulum):
    # This function animate  the pendulum and cart.

    length_for_display = pendulum.length * 100 # for displaying pendulum
    A = np.zeros((world_size, world_size, 3), np.uint8) # initializing the window
    cv2.line(A, (0, int(0.6 * world_size)), (world_size, int(0.6 * world_size)), (255,255,255)) # X axis
    # cart always in the window
    if cart.x - cart.width <= 0 :
       cart.x = 0 + cart.width
    if cart.x + cart.width >= 600 :
       cart.x= 600- cart.width


    cv2.rectangle(A, (int(cart.x) + cart.width, cart.y + cart.hight), (int(cart.x) - cart.width, cart.y - cart.hight), cart.color, -1) # The cart

    pendulum_x_endpoint = int(cart.x - (length_for_display) * math.sin(pendulum.theta)) # pendulum's x value
    pendulum_y_endpoint = int(cart.y - (length_for_display) * math.cos(pendulum.theta)) # pendulum's y value

    cv2.line(A, (int(cart.x), cart.y), (pendulum_x_endpoint, pendulum_y_endpoint), pendulum.color, 4) # pendulum arm
    cv2.circle(A, (pendulum_x_endpoint, pendulum_y_endpoint), 10, (105,105,105), -1) # The mass
    cv2.circle(A, (int(cart.x) + cart.width, cart.y + cart.hight), 8, (105,105,105), -1) # right wheel
    cv2.circle(A, (int(cart.x) - cart.width, cart.y + cart.hight), 8, (105,105,105), -1) # left wheel
    cv2.imshow('WindowName', A)
    cv2.waitKey(5)




def main():

   simulate(simulation_time = 10, initial_theta = -1.25 , pid = [40,20,20]) # run animation


if __name__ == "__main__":

    main()
