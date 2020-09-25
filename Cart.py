class Cart:
    def __init__(self, x, mass, world_size):
        self.x = x # position of the cart
        self.y = int(0.55 * world_size)  # 0.55 was chosen for aesthetic reasons.
        self.mass = mass # mass of the cart
        # for animation
        self.color = (128,128,128)
        self.width = 50
        self.hight = 20
