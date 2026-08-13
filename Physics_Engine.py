from vectogebra import vector


class Physics:
    #Initialising the base properties of particles.
    def __init__(self, axis_of_rotation = None, angle = 0, angular_momentum = None, radius = 0, position = None, momentum=None, net_force = None, mass=1, charge=0):
        self.mass = mass
        self.charge = charge
        self.angle = angle
        self.angular_momentum = angular_momentum if angular_momentum is not None else vector(0,0,0)
        self.radius = radius
        self.axis_of_rotation = axis_of_rotation if axis_of_rotation is not None else vector(0,0,0)
        self.position = position
        self.momentum = momentum if momentum is not None else vector(0,0,0)
        self.velocity = self.momentum/self.mass
        self.net_force = net_force if net_force is not None else vector(0,0,0)
    def add_force(self, force_vector):
        self.net_force = self.net_force + force_vector
class NewtonianMechanics(Physics):
    def __init__(self, mass, position, friction_co, restitution_co, gravity = None):
        super().__init__(mass = mass, position = position)
        self.friction_co = friction_co
        self.gravity = gravity if gravity is not None else vector(0,0,0)
        self.restitution_co = restitution_co

    def frictional_force(self):
        if abs(self.velocity) > 0:
            force_of_friction = (-1)*(self.velocity/abs(self.velocity))*self.mass*abs(self.gravity)
        else:
            force_of_friction = vector(0,0,0)
        self.add_force(force_of_friction)
    def gravitational_force(self):
        force_of_gravity = self.mass * self.gravity
        self.add_force(force_of_gravity)
    def collision(self, boundary_normal):
        initial_momentum = self.momentum
        final_momentum = initial_momentum - (1+self.restitution_co)*(self.momentum*boundary_normal)*(boundary_normal)
        self.momentum = final_momentum
    def update(self, dt):
        self.momentum += self.net_force * dt
        self.velocity = (self.momentum)/self.mass
        self.position = self.position + self.velocity  * dt
        self.net_force = vector(0,0,0) #clears. prevents foce bleed from last frame
class RotationalMechanics(NewtonianMechanics):
    def __init__(self, mass, position, moment_of_inertia_com, friction_co, restitution_co, net_torque = None):
        super().__init__(mass = mass, position = position, friction_co = friction_co, restitution_co = restitution_co)
        self.moment_of_inertia_com = moment_of_inertia_com
        self.net_torque = net_torque if net_torque is not None else vector(0,0,0)
    def add_torque(self, torque_vector):
        self.net_torque += torque_vector
    def frictional_torque(self, contact_point):
        super().frictional_force()
        torque_of_friction = (contact_point - self.position) ^ force_of_friction
        self.add_torque(torque_of_friction)
    def gravitation_torque(self):
        super().gravitational_force()
        torque_of_gravity = (self.position - self.axis_of_rotation) ^ force_of_gravity
        self.add_torque(torque_of_gravity)
    def update(self, dt):
        super().update(dt)
        self.orbital_radius = self.position - self.axis_of_rotation
        self.moment_of_inertia = self.moment_of_inertia_com + self.mass * (self.orbital_radius)**2
        self.angular_momentum += self.net_torque * dt
        self.angular_velocity = (self.angular_momentum.k)/(self.moment_of_inertia)
        self.angle +=  self.angular_velocity * dt
        self.net_torque = vector(0,0,0)
