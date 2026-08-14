from vectogebra.vector import vector


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
class Dynamics(Physics):
    def __init__(self, mass, position, friction_co, restitution_co, gravity = None, charge = 0, electric_field = None):
        super().__init__(mass = mass, position = position, charge = charge)
        self.friction_co = friction_co
        self.gravity = gravity if gravity is not None else vector(0,0,0)
        self.restitution_co = restitution_co
        self.electric_field = electric_field if electric_field is not None else vector(0,0,0)
    def frictional_force(self):
        if abs(self.velocity) > 0:
            self.force_of_friction = (-1)*(self.velocity/abs(self.velocity))*self.mass*abs(self.gravity)
        else:
            self.force_of_friction = vector(0,0,0)
        self.add_force(self.force_of_friction)
    def gravitational_force(self):
        self.force_of_gravity = self.mass * self.gravity
        self.add_force(self.force_of_gravity)
    def electrostatic_force(self):
        self.electric_force = self.electric_field * self.charge
        self.add_force(self.electric_force)
    def collision(self, boundary_normal):
        initial_momentum = self.momentum
        final_momentum = initial_momentum - (1+self.restitution_co)*(self.momentum*boundary_normal)*(boundary_normal)
        self.momentum = final_momentum
    def particle_colllision(self, other, unit_body_normal):
        self.unit_body_normal = (other.position - self.position)/abs(self.position - other.position)
        self.relative_velocity = (self.momentum/self.mass - other.momentum/other.mass)*self.unit_body_normal
        self.distance_between_centre = abs(self.position - other.position)
        penetration = self.radius + other.radius - self.distance_between_centre
        if self.distance_between_centre > (self.radius + other.radius):
            pass
        elif self.distance_between_centre <= (self.radius + other.radius) and self.distance_between_centre != 0 and self.relative_velocity >= 0:
            average_restitution_co = (self.restitution_co + other.restitution_co)/2
            self.future_momentum = (self.mass)*(((self.mass - average_restitution_co*other.mass)*(self.momentum/self.mass) + (((1 + average_restitution_co)*other.momentum))/(self.mass + other.mass)))
            other.momentum = (other.mass)*(((other.mass - average_restitution_co*self.mass)*(other.momentum/other.mass) + (((1 + average_restitution_co)*(self.momentum)))/(self.mass + other.mass)))
            self.momentum = self.future_momentum
            penetration = 0.8 * penetration
            self.penetration_share = (1/self.mass)/(1/self.mass + 1/other.mass)
            other.penetration_share = (1/other.mass)/(1/self.mass + 1/other.mass)
            self.position -= self.penetration_share*penetration*self.unit_body_normal
            other.position += other.penetration_share*penetration*self.unit_body_normal
        elif self.distance_between_centre <= (self.radius + other.radius) and self.relative_velocity < 0:
            pass
        elif self.distance_between_centre == 0:
             pass
    def update(self, dt):
        self.momentum += self.net_force * dt
        self.velocity = (self.momentum)/self.mass
        self.position = self.position + self.velocity  * dt
        self.net_force = vector(0,0,0) #clears. prevents foce bleed from last frame
class RotationalMechanics(Dynamics):
    def __init__(self, mass, position, moment_of_inertia_com, friction_co, restitution_co, net_torque = None, gravity = None, charge = 0, electric_field = None):
        super().__init__(mass = mass, position = position, friction_co = friction_co, restitution_co = restitution_co, gravity = gravity, charge = charge, electric_field = electric_field)
        self.moment_of_inertia_com = moment_of_inertia_com
        self.net_torque = net_torque if net_torque is not None else vector(0,0,0)
    def add_torque(self, torque_vector):
        self.net_torque += torque_vector
    def frictional_torque(self, contact_point):
        self.torque_of_friction = (contact_point - self.position) ^ self.force_of_friction
        self.add_torque(self.torque_of_friction)
    def gravitation_torque(self):
        self.torque_of_gravity = (self.position - self.axis_of_rotation) ^ self.force_of_gravity
        self.add_torque(self.torque_of_gravity)
    def electrical_torque(self):
        self.torque_of_electrostat = (self.position - self.axis_of_rotation) ^ self.electric_force
        self.add_torque(self.torque_of_electrostat)
    def update(self, dt):
        super().update(dt)
        self.orbital_radius = self.position - self.axis_of_rotation
        self.moment_of_inertia = self.moment_of_inertia_com + self.mass * (self.orbital_radius)**2
        self.angular_momentum += self.net_torque * dt
        self.angular_velocity = (self.angular_momentum.k)/(self.moment_of_inertia)
        self.angle +=  self.angular_velocity * dt
        self.net_torque = vector(0,0,0)
