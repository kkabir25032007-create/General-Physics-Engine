# test.py - tkinter: projectile with ground + wall bounces
import tkinter as tk
from Physics_Engine import Dynamics
from vectogebra import vector

# physics world
g = vector(0, -9.81, 0)
dt = 0.02
LEFT, RIGHT, GROUND = 0.0, 27.0, 0.0       # walls in physics units (3x wider)

body = Dynamics(mass=1, position=vector(2, 8, 0),
               friction_co=0.3, restitution_co=0.75, gravity=g)
body.momentum = vector(4, 6, 0)       # initial velocity 100x

# view: 1 physics unit = SCALE pixels; canvas y down, physics y up
W, H = 1900, 1000
SCALE = 34
X0 = (W - RIGHT * SCALE) // 2   # arena centered on screen
Y0 = H - 40
CEILING = (H // 2) / SCALE      # arena height: half the screen, in physics units

def to_screen(p):
    return X0 + p.i * SCALE, Y0 - p.j * SCALE

root = tk.Tk()
root.title("Projectile with collisions")
cv = tk.Canvas(root, width=W, height=H, bg="white")
cv.pack()

trail = []

def draw():
    cv.delete("all")
    # walls
    lx, ly = to_screen(vector(LEFT, GROUND, 0))
    rx, ry = to_screen(vector(RIGHT, GROUND, 0))
    gx, gy = to_screen(vector(0, GROUND, 0))
    cv.create_line(gx, gy, rx, ry, fill="gray")      # ground
    cx, cy = to_screen(vector(0, CEILING, 0))
    cv.create_line(lx, ly, lx, cy, fill="gray")      # left wall
    cv.create_line(rx, ry, rx, cy, fill="gray")      # right wall
    cv.create_line(lx, cy, rx, cy, fill="gray")      # ceiling
    # trail (last 300 points)
    pts = [to_screen(p) for p in trail[-300:]]
    if len(pts) > 1:
        coords = []
        for pt in pts:
            coords.extend(pt)
        cv.create_line(*coords, fill="blue")
    # ball
    x, y = to_screen(body.position)
    cv.create_oval(x - 3, y - 3, x + 3, y + 3, fill="black")

def touching_boundary():
    # on the ground or pressed against a wall
    return (body.position.j <= GROUND or
            body.position.i <= LEFT or
            body.position.i >= RIGHT)


def step():
    body.gravitational_force()
    if touching_boundary():
        body.frictional_force()   # friction only on surface contact
    body.update(dt)

    # boundary checks: engine does the impulse, test.py owns detection + reposition
    if body.position.j <= GROUND and body.velocity.j < 0:
        body.collision(vector(0, 1, 0))
        body.position = vector(body.position.i, GROUND, 0)
    if body.position.j >= CEILING and body.velocity.j > 0:
        body.collision(vector(0, -1, 0))
        body.position = vector(body.position.i, CEILING, 0)
    if body.position.i <= LEFT and body.velocity.i < 0:
        body.collision(vector(1, 0, 0))
        body.position = vector(LEFT, body.position.j, 0)
    if body.position.i >= RIGHT and body.velocity.i > 0:
        body.collision(vector(-1, 0, 0))
        body.position = vector(RIGHT, body.position.j, 0)

    trail.append(body.position)
    draw()
    root.after(20, step)

step()
root.mainloop()
